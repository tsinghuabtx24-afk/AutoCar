/**
  ******************************************************************************
  * @file    car.c
  * @brief   智能小车运动控制实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "car.h"
#include "encoder.h"
#include "speed_ctrl.h"

#include <stdio.h>

/* Private define ------------------------------------------------------------*/
/* 左右两侧电机镜像安装，因此两侧底层极性相反。 */
#define CAR_LEFT_POLARITY          (1)
#define CAR_RIGHT_POLARITY         (-1)

/* 整车前进方向补偿。根据最新实车测试，Car_Drive() 的正输入需要整体取反
   才对应物理车头方向。以后只调整这一项，不再同时改多个方向参数。 */
#define CAR_FORWARD_POLARITY       (-1)

/* Private types -------------------------------------------------------------*/
/* 动作种类。IDLE 之外每一种对应 Car_ActionStep() 里的一个推进分支。 */
typedef enum
{
  CAR_ACT_NONE = 0,
  CAR_ACT_DISTANCE,   /* 按编码器里程 */
  CAR_ACT_ANGLE,      /* 按编码器转角 */
  CAR_ACT_TIMED,      /* 保持当前差速一段时间 */
  CAR_ACT_RAMP,       /* 变速直线 */
  CAR_ACT_BRAKE       /* 制动保持 */
} Car_ActionKind;

typedef struct
{
  Car_ActionKind kind;
  uint32_t start_tick;
  uint32_t last_report_tick;
  uint32_t timeout_ms;

  /* 编码器起始快照。不清零全局计数，用增量判断动作是否达标。 */
  int32_t  start_distance_mm;
  int32_t  start_angle_deg10;

  uint32_t target_distance_mm;   /* DISTANCE */
  uint32_t target_angle_deg10;   /* ANGLE，已含滑移补偿 */
  uint16_t duration_ms;          /* TIMED / RAMP / BRAKE */

  int16_t  ramp_from_eff;        /* RAMP 起始有效速度 */
  int16_t  ramp_to_eff;          /* RAMP 结束有效速度 */

  int8_t   direction;            /* 1=前进/左转，-1=后退/右转，仅用于诊断 */
} Car_Action;

/* Private variables ---------------------------------------------------------*/
static Car_Action car_action;

/* "车体前进" → 各电机 Motor_SetSpeedRaw 的符号。交给 speed_ctrl 用，
   这样极性知识仍然只有本文件一份（CAR_*_POLARITY 是唯一真源）。
   顺序与 Motor_ID 一致：M1/M2 左，M3/M4 右。 */
static const int8_t car_out_polarity[MOTOR_NUM] =
{
  (int8_t)(CAR_FORWARD_POLARITY * CAR_LEFT_POLARITY),
  (int8_t)(CAR_FORWARD_POLARITY * CAR_LEFT_POLARITY),
  (int8_t)(CAR_FORWARD_POLARITY * CAR_RIGHT_POLARITY),
  (int8_t)(CAR_FORWARD_POLARITY * CAR_RIGHT_POLARITY),
};

/* Private function prototypes -----------------------------------------------*/
static uint8_t Car_ToEff(uint8_t speed);
#if !SPEEDCTRL_ENABLE
static int16_t Car_EffToDuty(int16_t eff);
#endif
#if SPEEDCTRL_ENABLE
static int16_t Car_EffToRpm(int16_t eff);
#endif
static void    Car_BrakeAll(void);
static uint16_t Car_AngleCompX1000(uint8_t speed, uint16_t radius_mm,
                                   int8_t direction);
static void Car_RunDistance(uint8_t speed, uint32_t distance_mm, int8_t direction);
static void Car_RunAngle(uint8_t speed, uint16_t radius_mm,
                         uint32_t angle_deg10, int8_t direction);
static Car_ActionStatus Car_ActionFinish(Car_ActionStatus status);
static Car_ActionStatus Car_RunToCompletion(Car_ActionStatus start_status);
static int32_t Car_Abs32(int32_t v);
static uint8_t Car_AngleWheelSpeeds(uint8_t speed, uint16_t radius_mm,
                                    int8_t direction,
                                    int16_t *left, int16_t *right);
static uint8_t Car_ActionBegin(Car_ActionKind kind, int8_t direction);
static Car_ActionStatus Car_StartDistance(uint8_t speed, uint32_t distance_mm,
                                          int8_t direction);
static Car_ActionStatus Car_StepDistance(uint32_t now);
static Car_ActionStatus Car_StepAngle(uint32_t now);
static Car_ActionStatus Car_StepRamp(uint32_t elapsed);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  按车体方向驱动左右两侧车轮
  * @param  left:  左侧有效速度，-CAR_SPEED_SPAN~CAR_SPEED_SPAN，正数向前
  * @param  right: 右侧有效速度，同上
  * @note   入参是**有效速度**（已减掉死区），这里负责加回 CAR_SPEED_BASE
  *         再下发，并通过极性宏把"车体方向"换算成各电机的实际转向。
  *         上层函数只在有效速度刻度上做比例运算，不必关心死区和接线。
  */
static void Car_Drive(int16_t left, int16_t right)
{
#if SPEEDCTRL_ENABLE
  /* 闭环：把有效速度换成目标转速交给 PI，不再直写 PWM。
     占空比由 PI 根据实测转速自己算，电池压降自动补偿。 */
  SpeedCtrl_SetTargetRpm(Car_EffToRpm(left), Car_EffToRpm(right));
#else
  int16_t dl = Car_EffToDuty(left);
  int16_t dr = Car_EffToDuty(right);

  Motor_SetSpeedRaw(MOTOR_1, (int8_t)(car_out_polarity[MOTOR_1] * dl));
  Motor_SetSpeedRaw(MOTOR_2, (int8_t)(car_out_polarity[MOTOR_2] * dl));
  Motor_SetSpeedRaw(MOTOR_3, (int8_t)(car_out_polarity[MOTOR_3] * dr));
  Motor_SetSpeedRaw(MOTOR_4, (int8_t)(car_out_polarity[MOTOR_4] * dr));
#endif
}

/**
  * @brief  交还闭环控制权并制动
  *
  * @note   顺序很关键：必须先 SpeedCtrl_Disable() 再 Motor_BrakeAll()。
  *         反过来的话，SysTick 里的 SpeedCtrl_Step() 可能在制动之后又写一次
  *         PWM，把制动状态冲掉，车会继续滑行。本文件所有制动点统一走这里。
  */
static void Car_BrakeAll(void)
{
#if SPEEDCTRL_ENABLE
  SpeedCtrl_Disable();
#endif
  Motor_BrakeAll();
}

#if SPEEDCTRL_ENABLE
/**
  * @brief  有效速度 → 目标转速 rpm
  * @param  eff: 有效速度 -CAR_SPEED_SPAN~CAR_SPEED_SPAN，正数向前
  * @retval 车体系目标转速，前进为正
  *
  * @note   线性映射：有效速度满量程 CAR_SPEED_SPAN 对应 ENC_MAX_RPM。
  *         这里不需要再管死区 —— 有效速度已经减掉了死区，而闭环的前馈
  *         会把死区加回到占空比上，两边各管一段，不重不漏。
  */
static int16_t Car_EffToRpm(int16_t eff)
{
  int32_t mag = (eff >= 0) ? eff : -(int32_t)eff;
  int32_t rpm;

  if (mag == 0)
  {
    return 0;
  }
  if (mag > (int32_t)CAR_SPEED_SPAN)
  {
    mag = (int32_t)CAR_SPEED_SPAN;
  }

  rpm = mag * (int32_t)ENC_MAX_RPM / (int32_t)CAR_SPEED_SPAN;

  return (eff >= 0) ? (int16_t)rpm : (int16_t)(-rpm);
}
#endif /* SPEEDCTRL_ENABLE */

/**
  * @brief  占空比刻度 → 有效速度
  * @param  speed: 占空比刻度 0~100
  * @retval 有效速度 0~CAR_SPEED_SPAN
  *
  * @note   减去死区。不超过 CAR_SPEED_BASE 的输入一律返回 0，因为那一段
  *         占空比电机根本不转，抬到 BASE+1 反而变成"用户要停我却在动"。
  */
static uint8_t Car_ToEff(uint8_t speed)
{
  if (speed <= (uint8_t)CAR_SPEED_BASE)
  {
    return 0U;
  }
  if (speed >= (uint8_t)MOTOR_SPEED_MAX)
  {
    return (uint8_t)CAR_SPEED_SPAN;
  }
  return (uint8_t)(speed - CAR_SPEED_BASE);
}

#if !SPEEDCTRL_ENABLE
/**
  * @brief  有效速度 → 占空比刻度（仅开环模式使用）
  * @param  eff: 有效速度，-CAR_SPEED_SPAN~CAR_SPEED_SPAN，正数向前
  * @retval 占空比刻度，符号保留
  *
  * @note   加回死区。有效速度 0 输出 0 而不是 BASE：输出 BASE 电机
  *         照样不转，却白白通电发热，还让被拖动的轮子多了阻力。
  *
  * @note   闭环模式下死区由 speed_ctrl 的前馈处理，本函数不参与编译。
  *         保留它是为了 SPEEDCTRL_ENABLE 置 0 时能原样退回开环。
  */
static int16_t Car_EffToDuty(int16_t eff)
{
  int16_t mag = (eff >= 0) ? eff : (int16_t)(-eff);

  if (mag == 0)
  {
    return 0;
  }
  if (mag > (int16_t)CAR_SPEED_SPAN)
  {
    mag = (int16_t)CAR_SPEED_SPAN;
  }
  mag = (int16_t)(mag + (int16_t)CAR_SPEED_BASE);

  return (eff >= 0) ? mag : (int16_t)(-mag);
}
#endif /* !SPEEDCTRL_ENABLE */

/**
  * @brief  获取角度动作的编码器目标放大系数
  * @retval 千分比系数，1000 表示不补偿
  */
static uint16_t Car_AngleCompX1000(uint8_t speed, uint16_t radius_mm,
                                   int8_t direction)
{
#if CAR_ANGLE_COMP_ENABLE
  if (radius_mm != 0U)
  {
    return (direction > 0) ? CAR_TURN_LEFT_COMP_X1000
                           : CAR_TURN_RIGHT_COMP_X1000;
  }

  if (speed <= 70U)
  {
    return CAR_ROT_COMP_70_X1000;
  }
  if (speed <= 80U)
  {
        return (uint16_t)((int32_t)CAR_ROT_COMP_70_X1000
                      + ((int32_t)CAR_ROT_COMP_80_X1000
                         - (int32_t)CAR_ROT_COMP_70_X1000)
                        * (int32_t)(speed - 70U) / 10L);
  }
  if (speed < 100U)
  {
        return (uint16_t)((int32_t)CAR_ROT_COMP_80_X1000
                      + ((int32_t)CAR_ROT_COMP_100_X1000
                         - (int32_t)CAR_ROT_COMP_80_X1000)
                        * (int32_t)(speed - 80U) / 20L);
  }
  return CAR_ROT_COMP_100_X1000;
#else
  (void)speed;
  (void)radius_mm;
  (void)direction;
  return 1000U;
#endif
}

/**
  * @brief  按转弯半径算出内侧轮的有效速度
  * @param  outer:     外侧轮**有效速度** 0~CAR_SPEED_SPAN
  * @param  radius_mm: 转弯半径（车体中心线到圆心），单位 mm
  * @retval 内侧轮有效速度 0~CAR_SPEED_SPAN
  *
  * @note   差速运动学：两轮走同心圆、角速度相同，故
  *             v₂ / (R + D/2) = v₁ / (R - D/2)
  *         整理即 2R(v₂ - v₁) = D(v₁ + v₂)，解出内轮速
  *             v₁ = v₂ · (2R - D) / (2R + D)
  *         R = D/2 时 v₁ = 0（绕内侧轮打转），R → ∞ 时 v₁ → v₂（直线）。
  *
  * @note   v 必须是**有效速度**而不是占空比。占空比含 50% 的死区偏置，
  *         按占空比取比例会把偏置也按比例缩放，速比失真、半径偏大。
  */
static int16_t Car_InnerSpeed(uint8_t outer, uint16_t radius_mm)
{
  int32_t d  = (int32_t)CAR_WHEEL_TRACK_MM;
  int32_t r2 = 2 * (int32_t)radius_mm;

  if (radius_mm <= CAR_RADIUS_MIN_MM)
  {
    return 0;                       /* 绕内侧轮打转，内轮停 */
  }
  return (int16_t)((int32_t)outer * (r2 - d) / (r2 + d));
}

/**
  * @brief  保持当前动作一段时间后制动
  * @param  time: 毫秒
  * @note   阻塞式，仅供保留的调试接口使用。
  */
static void Car_RunFor(uint16_t time)
{
  HAL_Delay(time);
  Car_BrakeAll();
}

/**
  * @brief  int32 取绝对值
  * @note   用 int64 中转，避免 INT32_MIN 取反溢出。
  */
static int32_t Car_Abs32(int32_t v)
{
  return (v >= 0) ? v : (int32_t)(-(int64_t)v);
}

/**
  * @brief  算出角度动作的左右轮有效速度
  * @param  speed:     占空比刻度 0~100
  * @param  radius_mm: 0=原地旋转，≤D/2=绕内侧轮，其余为差速圆弧
  * @param  direction: 1=左转，-1=右转
  * @param  left:      输出左侧有效速度
  * @param  right:     输出右侧有效速度
  * @retval 1=参数可用，0=速度落在死区内，调用方应制动并放弃动作
  *
  * @note   从原 Car_RunAngle 抽出，阻塞与非阻塞两条路径共用，避免运动学
  *         算式出现两份实现。
  */
static uint8_t Car_AngleWheelSpeeds(uint8_t speed, uint16_t radius_mm,
                                    int8_t direction,
                                    int16_t *left, int16_t *right)
{
  int16_t outer = (int16_t)Car_ToEff(speed);
  int16_t inner;

  if (outer == 0)
  {
    return 0U;
  }

  if (radius_mm == 0U)
  {
    /* 原地旋转：左右轮等速反向。 */
    inner = (int16_t)(-outer);
  }
  else if (radius_mm <= CAR_RADIUS_MIN_MM)
  {
    /* 绕内侧轮旋转，内侧轮停止。 */
    inner = 0;
  }
  else
  {
    inner = Car_InnerSpeed((uint8_t)outer, radius_mm);
  }

  if (direction > 0)
  {
    *left  = inner;
    *right = outer;
  }
  else
  {
    *left  = outer;
    *right = inner;
  }
  return 1U;
}

/**
  * @brief  结束当前动作：制动、清状态、返回终态
  */
static Car_ActionStatus Car_ActionFinish(Car_ActionStatus status)
{
  Car_BrakeAll();
  car_action.kind = CAR_ACT_NONE;
  return status;
}

/**
  * @brief  循环推进当前动作直到终态
  * @note   阻塞式，仅供保留的标定接口使用。正式主循环应自己调 Car_ActionStep()。
  *         这里保留 1ms 让步，与原实现的节奏一致。
  */
static Car_ActionStatus Car_RunToCompletion(Car_ActionStatus start_status)
{
  Car_ActionStatus status = start_status;

  /* 发起失败时直接返回。不能盲目 Step —— 若失败原因是"已有动作在跑"，
     继续 Step 会把别人的动作推到完成。 */
  while (status == CAR_ACTION_BUSY)
  {
    HAL_Delay(1U);
    status = Car_ActionStep();
  }
  return status;
}

/**
  * @brief  按编码器里程直线运行
  * @param  speed:       占空比刻度 0~100
  * @param  distance_mm: 目标距离 mm
  * @param  direction:   1=前进，-1=后退
  * @note   此函数为阻塞式；Encoder_Update() 由 SysTick 持续执行，所以等待
  *         期间编码器仍会正常累计。到达目标或超时后使用能耗制动停车。
  */
static void Car_RunDistance(uint8_t speed, uint32_t distance_mm, int8_t direction)
{
  Car_ActionStatus started = (direction > 0)
                               ? Car_StartForwardDistance(speed, distance_mm)
                               : Car_StartBackwardDistance(speed, distance_mm);

  (void)Car_RunToCompletion(started);
}

/**
  * @brief  按编码器转角运行（阻塞式，标定入口）
  * @param  direction: 1=左转，-1=右转
  */
static void Car_RunAngle(uint8_t speed, uint16_t radius_mm,
                         uint32_t angle_deg10, int8_t direction)
{
  int32_t signed_angle = (direction > 0)
                           ? (int32_t)angle_deg10
                           : (int32_t)(-(int64_t)angle_deg10);

  (void)Car_RunToCompletion(Car_StartTurnAngle(speed, radius_mm, signed_angle));
}

/* ==== 非阻塞动作状态机 =====================================================*/

/**
  * @brief  登记一个新动作的公共字段
  * @retval 1=可以继续，0=已有动作在跑
  */
static uint8_t Car_ActionBegin(Car_ActionKind kind, int8_t direction)
{
  uint32_t now;

  if (car_action.kind != CAR_ACT_NONE)
  {
    return 0U;
  }

  now = HAL_GetTick();
  car_action.kind              = kind;
  car_action.direction         = direction;
  car_action.start_tick        = now;
  car_action.last_report_tick  = now;
  car_action.timeout_ms        = CAR_DISTANCE_TIMEOUT_MS;
  /* 起始快照：不清零全局计数，用增量判断达标。 */
  car_action.start_distance_mm = Encoder_GetDistanceAvg();
  car_action.start_angle_deg10 = Encoder_GetRotationDeg10(CAR_WHEEL_TRACK_MM);
  return 1U;
}

static Car_ActionStatus Car_StartDistance(uint8_t speed, uint32_t distance_mm,
                                         int8_t direction)
{
  int16_t v = (int16_t)Car_ToEff(speed);

  if (Car_ActionBegin(CAR_ACT_DISTANCE, direction) == 0U)
  {
    printf("[DIST] rejected: action %u already running\r\n",
           (unsigned)car_action.kind);
    return CAR_ACTION_FAILED;
  }

  if ((v == 0) || (distance_mm == 0U))
  {
    return Car_ActionFinish(CAR_ACTION_FAILED);
  }

  car_action.target_distance_mm = distance_mm;

  printf("[DIST] start dir=%s target=%lumm speed=%u\r\n",
         (direction > 0) ? "forward" : "backward",
         (unsigned long)distance_mm,
         (unsigned)speed);
  Car_Drive((int16_t)(direction * v), (int16_t)(direction * v));
  return CAR_ACTION_BUSY;
}

Car_ActionStatus Car_StartForwardDistance(uint8_t speed, uint32_t distance_mm)
{
  return Car_StartDistance(speed, distance_mm, 1);
}

Car_ActionStatus Car_StartBackwardDistance(uint8_t speed, uint32_t distance_mm)
{
  return Car_StartDistance(speed, distance_mm, -1);
}

Car_ActionStatus Car_StartTurnAngle(uint8_t speed, uint16_t radius_mm,
                                    int32_t angle_deg10)
{
  int8_t   direction = (angle_deg10 >= 0) ? 1 : -1;
  uint32_t magnitude = (uint32_t)Car_Abs32(angle_deg10);
  int16_t  left;
  int16_t  right;
  uint16_t comp_x1000;

  if (Car_ActionBegin(CAR_ACT_ANGLE, direction) == 0U)
  {
    printf("[ANGLE] rejected: action %u already running\r\n",
           (unsigned)car_action.kind);
    return CAR_ACTION_FAILED;
  }

  if (magnitude == 0U)
  {
    return Car_ActionFinish(CAR_ACTION_FAILED);
  }

#if CAR_ANGLE_COMP_ENABLE
  /* 滑移补偿表只在 CAR_ANGLE_MIN_SPEED 以上标定过，低速原地旋转不接受。 */
  if ((radius_mm == 0U) && (speed < CAR_ANGLE_MIN_SPEED))
  {
    printf("[ANGLE] rejected: speed %u < calibrated minimum %u\r\n",
           (unsigned)speed, (unsigned)CAR_ANGLE_MIN_SPEED);
    return Car_ActionFinish(CAR_ACTION_FAILED);
  }
#endif

  if (Car_AngleWheelSpeeds(speed, radius_mm, direction, &left, &right) == 0U)
  {
    return Car_ActionFinish(CAR_ACTION_FAILED);
  }

  comp_x1000 = Car_AngleCompX1000(speed, radius_mm, direction);
  car_action.target_angle_deg10 =
      (uint32_t)(((uint64_t)magnitude * comp_x1000 + 500ULL) / 1000ULL);

  printf("[ANGLE] start dir=%s real=%lu.%01lu deg encoder=%lu.%01lu deg "
         "comp=%u.%03u radius=%umm speed=%u\r\n",
         (direction > 0) ? "left" : "right",
         (unsigned long)(magnitude / 10U),
         (unsigned long)(magnitude % 10U),
         (unsigned long)(car_action.target_angle_deg10 / 10U),
         (unsigned long)(car_action.target_angle_deg10 % 10U),
         (unsigned)(comp_x1000 / 1000U),
         (unsigned)(comp_x1000 % 1000U),
         (unsigned)radius_mm,
         (unsigned)speed);
  Car_Drive(left, right);
  return CAR_ACTION_BUSY;
}

Car_ActionStatus Car_StartRotateAngle(uint8_t speed, int32_t angle_deg10)
{
  return Car_StartTurnAngle(speed, 0U, angle_deg10);
}

Car_ActionStatus Car_StartTimed(int16_t left, int16_t right, uint16_t time)
{
  if (Car_ActionBegin(CAR_ACT_TIMED, 1) == 0U)
  {
    return CAR_ACTION_FAILED;
  }

  car_action.duration_ms = time;
  /* 纯计时动作不依赖编码器，duration 到点必然完成，超时上限留一点余量即可。 */
  car_action.timeout_ms  = (uint32_t)time + CAR_ACTION_TIMEOUT_MARGIN_MS;

  if (time == 0U)
  {
    return Car_ActionFinish(CAR_ACTION_DONE);
  }

  Car_DriveSpeed(left, right);
  return CAR_ACTION_BUSY;
}

Car_ActionStatus Car_StartForwardVary(uint8_t speed_from, uint8_t speed_to,
                                      uint16_t time)
{
  if (Car_ActionBegin(CAR_ACT_RAMP, 1) == 0U)
  {
    return CAR_ACTION_FAILED;
  }

  car_action.ramp_from_eff = (int16_t)Car_ToEff(speed_from);
  car_action.ramp_to_eff   = (int16_t)Car_ToEff(speed_to);
  car_action.duration_ms   = time;
  car_action.timeout_ms    = (uint32_t)time + CAR_ACTION_TIMEOUT_MARGIN_MS;

  if (time == 0U)
  {
    return Car_ActionFinish(CAR_ACTION_DONE);
  }

  Car_Drive(car_action.ramp_from_eff, car_action.ramp_from_eff);
  return CAR_ACTION_BUSY;
}

Car_ActionStatus Car_StartBrake(uint16_t time)
{
  if (Car_ActionBegin(CAR_ACT_BRAKE, 1) == 0U)
  {
    return CAR_ACTION_FAILED;
  }

  Car_BrakeAll();
  car_action.duration_ms = time;
  car_action.timeout_ms  = (uint32_t)time + CAR_ACTION_TIMEOUT_MARGIN_MS;

  if (time == 0U)
  {
    return Car_ActionFinish(CAR_ACTION_DONE);
  }
  return CAR_ACTION_BUSY;
}

/**
  * @brief  推进 DISTANCE 动作一步
  */
static Car_ActionStatus Car_StepDistance(uint32_t now)
{
  int32_t  distance  = Encoder_GetDistanceAvg() - car_action.start_distance_mm;
  uint32_t travelled = (uint32_t)Car_Abs32(distance);
  int32_t  speed_mmps = Encoder_GetSpeedMmps();
  uint32_t lead_mm;
  uint32_t lead_cap;
  uint32_t stop_at;

  /* ==== 制动提前量 ====
     提前量 = 车速 × 等效制动时间。走够了才刹车的话，刹车过程走过的距离
     全是净超调；提前这一段开始刹，停下时正好在目标位置附近。
     取绝对值：后退时车速为负，提前量与方向无关。 */
  lead_mm = (uint32_t)((uint64_t)Car_Abs32(speed_mmps) * CAR_BRAKE_LEAD_MS
                       / 1000ULL);

  /* 短距离动作要防提前量吃掉整个目标 —— 否则目标 100mm 而提前量 120mm 时
     一进来就判停，车根本没动。按比例截断。 */
  lead_cap = car_action.target_distance_mm * CAR_BRAKE_LEAD_MAX_PCT / 100U;
  if (lead_mm > lead_cap)
  {
    lead_mm = lead_cap;
  }

  stop_at = car_action.target_distance_mm - lead_mm;

  if (travelled >= stop_at)
  {
    printf("[DIST] stop travelled=%lumm target=%lumm lead=%lumm speed=%ldmm/s\r\n",
           (unsigned long)travelled,
           (unsigned long)car_action.target_distance_mm,
           (unsigned long)lead_mm,
           (long)speed_mmps);
    return Car_ActionFinish(CAR_ACTION_DONE);
  }

  if ((uint32_t)(now - car_action.last_report_tick) >= CAR_ACTION_REPORT_MS)
  {
    car_action.last_report_tick = now;
    printf("[DIST %lums] count=%ld,%ld,%ld,%ld travelled=%ldmm rpm=%d,%d,%d,%d\r\n",
           (unsigned long)(now - car_action.start_tick),
           (long)Encoder_GetCount(MOTOR_1),
           (long)Encoder_GetCount(MOTOR_2),
           (long)Encoder_GetCount(MOTOR_3),
           (long)Encoder_GetCount(MOTOR_4),
           (long)distance,
           (int)Encoder_GetSpeedRpm(MOTOR_1),
           (int)Encoder_GetSpeedRpm(MOTOR_2),
           (int)Encoder_GetSpeedRpm(MOTOR_3),
           (int)Encoder_GetSpeedRpm(MOTOR_4));
  }
  return CAR_ACTION_BUSY;
}

/**
  * @brief  推进 ANGLE 动作一步
  */
static Car_ActionStatus Car_StepAngle(uint32_t now)
{
  int32_t  angle  = Encoder_GetRotationDeg10(CAR_WHEEL_TRACK_MM)
                    - car_action.start_angle_deg10;
  uint32_t turned = (uint32_t)Car_Abs32(angle);

  if (turned >= car_action.target_angle_deg10)
  {
    printf("[ANGLE] stop turned=%ld.%01lddeg\r\n",
           (long)(angle / 10), (long)(Car_Abs32(angle) % 10));
    return Car_ActionFinish(CAR_ACTION_DONE);
  }

  if ((uint32_t)(now - car_action.last_report_tick) >= CAR_ACTION_REPORT_MS)
  {
    car_action.last_report_tick = now;
    printf("[ANGLE %lums] left=%ldmm right=%ldmm angle=%ld.%01lddeg\r\n",
           (unsigned long)(now - car_action.start_tick),
           (long)Encoder_GetDistanceLeft(),
           (long)Encoder_GetDistanceRight(),
           (long)(angle / 10),
           (long)(Car_Abs32(angle) % 10));
  }
  return CAR_ACTION_BUSY;
}

/**
  * @brief  推进 RAMP 动作一步
  * @note   按已用时间比例插值，取代原先每 CAR_RAMP_STEP_MS 一次 HAL_Delay
  *         的写法。插值仍在有效速度刻度上做，加速度保持恒定。
  */
static Car_ActionStatus Car_StepRamp(uint32_t elapsed)
{
  int16_t v0 = car_action.ramp_from_eff;
  int16_t v1 = car_action.ramp_to_eff;
  int16_t v;

  if (elapsed >= (uint32_t)car_action.duration_ms)
  {
    return Car_ActionFinish(CAR_ACTION_DONE);
  }

  v = (int16_t)(v0 + (int32_t)(v1 - v0) * (int32_t)elapsed
                     / (int32_t)car_action.duration_ms);
  Car_Drive(v, v);
  return CAR_ACTION_BUSY;
}

Car_ActionStatus Car_ActionStep(void)
{
  uint32_t now;
  uint32_t elapsed;

  if (car_action.kind == CAR_ACT_NONE)
  {
    return CAR_ACTION_IDLE;
  }

  now     = HAL_GetTick();
  elapsed = (uint32_t)(now - car_action.start_tick);

  /* 超时保护先判：编码器没有脉冲时到期制动，防止一直运行。 */
  if (elapsed >= car_action.timeout_ms)
  {
    printf("[CAR] action %u timeout after %lums\r\n",
           (unsigned)car_action.kind, (unsigned long)elapsed);
    return Car_ActionFinish(CAR_ACTION_TIMEOUT);
  }

  switch (car_action.kind)
  {
    case CAR_ACT_DISTANCE:
      return Car_StepDistance(now);

    case CAR_ACT_ANGLE:
      return Car_StepAngle(now);

    case CAR_ACT_RAMP:
      return Car_StepRamp(elapsed);

    case CAR_ACT_TIMED:
    case CAR_ACT_BRAKE:
      if (elapsed >= (uint32_t)car_action.duration_ms)
      {
        return Car_ActionFinish(CAR_ACTION_DONE);
      }
      return CAR_ACTION_BUSY;

    default:
      return Car_ActionFinish(CAR_ACTION_FAILED);
  }
}

void Car_ActionAbort(void)
{
  if (car_action.kind != CAR_ACT_NONE)
  {
    printf("[CAR] action %u aborted\r\n", (unsigned)car_action.kind);
    (void)Car_ActionFinish(CAR_ACTION_IDLE);
  }
}

Car_ActionStatus Car_GetActionStatus(void)
{
  return (car_action.kind == CAR_ACT_NONE) ? CAR_ACTION_IDLE : CAR_ACTION_BUSY;
}

uint8_t Car_IsActionBusy(void)
{
  return (car_action.kind != CAR_ACT_NONE) ? 1U : 0U;
}

/**
  * @brief  直接指定左右两侧速度
  * @param  left/right: 占空比刻度 -100~100，正数向前
  * @note   供循迹差速修正和遥控手动转向使用。入参是占空比刻度而非有效速度，
  *         内部负责死区换算，调用方不必了解 CAR_SPEED_BASE。
  */
void Car_DriveSpeed(int16_t left, int16_t right)
{
  /* 先夹到 ±MOTOR_SPEED_MAX，再交给 Car_ToEff 做死区换算。不夹的话
     超范围的入参转 uint8_t 会截断，符号还在、数值却绕回去。 */
  int16_t lc = (left  >  (int16_t)MOTOR_SPEED_MAX) ?  (int16_t)MOTOR_SPEED_MAX
             : (left  < -(int16_t)MOTOR_SPEED_MAX) ? -(int16_t)MOTOR_SPEED_MAX
                                                   : left;
  int16_t rc = (right >  (int16_t)MOTOR_SPEED_MAX) ?  (int16_t)MOTOR_SPEED_MAX
             : (right < -(int16_t)MOTOR_SPEED_MAX) ? -(int16_t)MOTOR_SPEED_MAX
                                                   : right;

  int16_t le = (lc >= 0) ? (int16_t)Car_ToEff((uint8_t)lc)
                         : (int16_t)(-(int16_t)Car_ToEff((uint8_t)(-lc)));
  int16_t re = (rc >= 0) ? (int16_t)Car_ToEff((uint8_t)rc)
                         : (int16_t)(-(int16_t)Car_ToEff((uint8_t)(-rc)));

  Car_Drive(le, re);
}

/**
  * @brief  小车运动控制初始化
  * @note   内部调用 Motor_Init() 启动 8 路 PWM，调用前需先完成
  *         MX_TIM1_Init() 与 MX_TIM8_Init()。
  */
void Car_Init(void)
{
  Motor_Init();
  car_action.kind = CAR_ACT_NONE;

#if SPEEDCTRL_ENABLE
  /* 把极性表交给闭环。CAR_*_POLARITY 仍只在本文件定义，speed_ctrl 不重复
     一份 —— 以后改接线只改这里。 */
  SpeedCtrl_Init(car_out_polarity);
#endif
}

/**
  * @brief  前进
  * @param  speed: 速度百分比 0~100
  * @param  time:  持续时间（毫秒），结束后自动制动
  */
void Car_Forward(uint8_t speed, uint16_t time)
{
  int16_t v = (int16_t)Car_ToEff(speed);
  Car_Drive(v, v);
  Car_RunFor(time);
}

/**
  * @brief  后退
  * @param  speed: 速度百分比 0~100
  * @param  time:  持续时间（毫秒），结束后自动制动
  */
void Car_Backward(uint8_t speed, uint16_t time)
{
  int16_t v = (int16_t)Car_ToEff(speed);
  Car_Drive((int16_t)(-v), (int16_t)(-v));
  Car_RunFor(time);
}

/**
  * @brief  非阻塞式前进，供循迹等周期控制使用
  */
void Car_ForwardRun(uint8_t speed)
{
  int16_t v = (int16_t)Car_ToEff(speed);
  Car_Drive(v, v);
}

/**
  * @brief  立即制动
  */
void Car_Stop(void)
{
  Car_BrakeAll();
}

/**
  * @brief  前进指定距离后制动
  * @param  speed:       速度百分比 0~100，必须大于 CAR_SPEED_BASE
  * @param  distance_mm: 目标距离 mm
  */
void Car_ForwardDistance(uint8_t speed, uint32_t distance_mm)
{
  Car_RunDistance(speed, distance_mm, 1);
}

/**
  * @brief  后退指定距离后制动
  * @param  speed:       速度百分比 0~100，必须大于 CAR_SPEED_BASE
  * @param  distance_mm: 目标距离 mm
  */
void Car_BackwardDistance(uint8_t speed, uint32_t distance_mm)
{
  Car_RunDistance(speed, distance_mm, -1);
}

/**
  * @brief  变速直线：速度在 time 内从 speed_from 线性变到 speed_to
  * @param  speed_from: 起始速度百分比 0~100
  * @param  speed_to:   结束速度百分比 0~100，比起始小就是减速
  * @param  time:       全程时间（毫秒），结束后自动制动
  *
  * @note   插值在**有效速度**刻度上做。若按占空比插值，50% 的死区偏置也
  *         被算进插值区间，速度-时间曲线在低速段会被压扁，加速度不恒定。
  *
  * @note   每 CAR_RAMP_STEP_MS 更新一次占空比。time 不是步长整数倍时，
  *         余下的零头并在最后一步，保证总时间准确、末速正好是 speed_to。
  */
void Car_ForwardVary(uint8_t speed_from, uint8_t speed_to, uint16_t time)
{
  (void)Car_RunToCompletion(Car_StartForwardVary(speed_from, speed_to, time));
}

/**
  * @brief  左转，转弯半径可调
  * @param  speed:     外侧（右侧）车轮速度百分比 0~100
  * @param  radius_mm: 转弯半径（车体中心线到圆心），单位 mm
  *                    ≤ D/2 时内侧轮停转，绕内侧轮打转；越大弯道越缓
  * @param  time:      持续时间（毫秒），结束后自动制动
  */
void Car_TurnLeft(uint8_t speed, uint16_t radius_mm, uint16_t time)
{
  uint8_t outer = Car_ToEff(speed);
  Car_Drive(Car_InnerSpeed(outer, radius_mm), (int16_t)outer);
  Car_RunFor(time);
}

/**
  * @brief  右转，转弯半径可调
  * @param  speed:     外侧（左侧）车轮速度百分比 0~100
  * @param  radius_mm: 转弯半径（车体中心线到圆心），单位 mm
  *                    ≤ D/2 时内侧轮停转，绕内侧轮打转；越大弯道越缓
  * @param  time:      持续时间（毫秒），结束后自动制动
  */
void Car_TurnRight(uint8_t speed, uint16_t radius_mm, uint16_t time)
{
  uint8_t outer = Car_ToEff(speed);
  Car_Drive((int16_t)outer, Car_InnerSpeed(outer, radius_mm));
  Car_RunFor(time);
}

/**
  * @brief  左旋转（原地左转，左轮后退右轮前进）
  * @param  speed: 速度百分比 0~100
  * @param  time:  持续时间（毫秒），结束后自动制动
  */
void Car_RotateLeft(uint8_t speed, uint16_t time)
{
  int16_t v = (int16_t)Car_ToEff(speed);
  Car_Drive((int16_t)(-v), v);
  Car_RunFor(time);
}

/**
  * @brief  右旋转（原地右转，左轮前进右轮后退）
  * @param  speed: 速度百分比 0~100
  * @param  time:  持续时间（毫秒），结束后自动制动
  */
void Car_RotateRight(uint8_t speed, uint16_t time)
{
  int16_t v = (int16_t)Car_ToEff(speed);
  Car_Drive(v, (int16_t)(-v));
  Car_RunFor(time);
}

/**
  * @brief  按编码器角度原地左转
  * @param  speed:      速度百分比 0~100
  * @param  angle_deg10:目标角度，单位 0.1 度，例如 900=90.0 度
  */
void Car_RotateLeftAngle(uint8_t speed, uint32_t angle_deg10)
{
  Car_RunAngle(speed, 0U, angle_deg10, 1);
}

/**
  * @brief  按编码器角度原地右转
  */
void Car_RotateRightAngle(uint8_t speed, uint32_t angle_deg10)
{
  Car_RunAngle(speed, 0U, angle_deg10, -1);
}

/**
  * @brief  按编码器角度左转指定半径圆弧
  */
void Car_TurnLeftAngle(uint8_t speed, uint16_t radius_mm, uint32_t angle_deg10)
{
  Car_RunAngle(speed, radius_mm, angle_deg10, 1);
}

/**
  * @brief  按编码器角度右转指定半径圆弧
  */
void Car_TurnRightAngle(uint8_t speed, uint16_t radius_mm, uint32_t angle_deg10)
{
  Car_RunAngle(speed, radius_mm, angle_deg10, -1);
}

/**
  * @brief  制动
  * @param  time: 制动状态保持的时间（毫秒），传 0 则立即返回
  */
void Car_Brake(uint16_t time)
{
  Car_BrakeAll();
  if (time > 0U)
  {
    HAL_Delay(time);
  }
}

/* 说明：Car_Brake() 保持原样（直接制动 + 阻塞等待），因为它常被用作"立即停车"
   的同义词，改成状态机会让 Car_Brake(0) 之类的调用多绕一圈。需要非阻塞制动
   保持的场合用 Car_StartBrake()。 */

/**
  * @brief  走一个三叶草轨迹
  * @param  speed:     叶片行驶速度 0~100
  * @param  radius_mm: 叶片转弯半径 mm，越小整个图形占地越小
  * @param  leaf_time: 走完一整片叶子（转一整圈）的时间（毫秒）
  *
  * @note   每片叶子是一整圈 360° 定半径圆弧，走完车回到出发点、朝向复原；
  *         再原地左旋 120° 换下一片叶子的方向。三片叶子的圆心按 120°
  *         均匀分布，合起来就是三叶草：
  *
  *                    ___
  *                   /   \        ← 叶 1
  *              ___  \___/  ___
  *             /   \  /|\  /   \
  *             \___/   |   \___/  ← 叶 2、叶 3
  *
  *         三次旋转累计 360°，一轮结束后车头方向与开始时一致，可无缝重复。
  *         整个图形的外径约 4R，R = 150mm 时占地约 60cm 见方。
  *
  * @note   目前圆弧和旋转角度靠时间估算，leaf_time 和 CLOVER_TURN_MS
  *         必须在实际场地上标定：
  *           1. 先只跑一片叶子，调 leaf_time 到车刚好回到出发点；
  *           2. 再调 CLOVER_TURN_MS 到原地转过的角度接近 120°。
  *         电池电压下降会让两个时间都变长。接上编码器后可改成按里程闭环。
  */
void Car_Clover(uint8_t speed, uint16_t radius_mm, uint16_t leaf_time)
{
  for (uint8_t leaf = 0U; leaf < 3U; leaf++)
  {
    /* 一整圈定半径圆弧，画出一片叶子 */
    Car_TurnLeft(speed, radius_mm, leaf_time);
    Car_Brake(150U);                 /* 短暂停顿，让车稳住再换向 */

    /* 原地左旋 120°，对准下一片叶子 */
    Car_RotateLeft(CLOVER_TURN_SPEED, CLOVER_TURN_MS);
    Car_Brake(150U);
  }
}
