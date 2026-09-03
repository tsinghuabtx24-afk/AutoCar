/**
  ******************************************************************************
  * @file    car.c
  * @brief   智能小车运动控制实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "car.h"
#include "encoder.h"

#include <stdio.h>

/* Private define ------------------------------------------------------------*/
/* 左右两侧电机镜像安装，因此两侧底层极性相反。 */
#define CAR_LEFT_POLARITY          (1)
#define CAR_RIGHT_POLARITY         (-1)

/* 整车前进方向补偿。根据最新实车测试，Car_Drive() 的正输入需要整体取反
   才对应物理车头方向。以后只调整这一项，不再同时改多个方向参数。 */
#define CAR_FORWARD_POLARITY       (-1)

/* Private function prototypes -----------------------------------------------*/
static uint8_t Car_ToEff(uint8_t speed);
static int16_t Car_EffToDuty(int16_t eff);
static uint16_t Car_AngleCompX1000(uint8_t speed, uint16_t radius_mm,
                                   int8_t direction);
static void Car_RunDistance(uint8_t speed, uint32_t distance_mm, int8_t direction);
static void Car_RunAngle(uint8_t speed, uint16_t radius_mm,
                         uint32_t angle_deg10, int8_t direction);

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
  int16_t dl = Car_EffToDuty(left);
  int16_t dr = Car_EffToDuty(right);

    Motor_SetSpeedRaw(MOTOR_1,
                    (int8_t)(CAR_FORWARD_POLARITY * CAR_LEFT_POLARITY * dl));
  Motor_SetSpeedRaw(MOTOR_2,
                    (int8_t)(CAR_FORWARD_POLARITY * CAR_LEFT_POLARITY * dl));
  Motor_SetSpeedRaw(MOTOR_3,
                    (int8_t)(CAR_FORWARD_POLARITY * CAR_RIGHT_POLARITY * dr));
  Motor_SetSpeedRaw(MOTOR_4,
                    (int8_t)(CAR_FORWARD_POLARITY * CAR_RIGHT_POLARITY * dr));
}

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

/**
  * @brief  有效速度 → 占空比刻度
  * @param  eff: 有效速度，-CAR_SPEED_SPAN~CAR_SPEED_SPAN，正数向前
  * @retval 占空比刻度，符号保留
  *
  * @note   加回死区。有效速度 0 输出 0 而不是 BASE：输出 BASE 电机
  *         照样不转，却白白通电发热，还让被拖动的轮子多了阻力。
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
  */
static void Car_RunFor(uint16_t time)
{
  HAL_Delay(time);
  Motor_BrakeAll();
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
  int16_t v = (int16_t)Car_ToEff(speed);
  uint32_t start_tick;
  uint32_t last_report_tick;

  if ((v == 0) || (distance_mm == 0U))
  {
    Motor_BrakeAll();
    return;
  }

  Encoder_ResetAll();
  start_tick = HAL_GetTick();
  last_report_tick = start_tick;
  printf("[DIST] start dir=%s target=%lumm speed=%u\r\n",
         (direction > 0) ? "forward" : "backward",
         (unsigned long)distance_mm,
         (unsigned)speed);
  Car_Drive((int16_t)(direction * v), (int16_t)(direction * v));

  while (1)
  {
    uint32_t now = HAL_GetTick();
    int32_t distance = Encoder_GetDistanceAvg();
    uint32_t travelled = (distance >= 0)
                           ? (uint32_t)distance
                           : (uint32_t)(-(int64_t)distance);

    if (travelled >= distance_mm)
    {
      break;
    }
    if ((uint32_t)(now - start_tick) >= CAR_DISTANCE_TIMEOUT_MS)
    {
      printf("[DIST] timeout\r\n");
      break;
    }
    if ((uint32_t)(now - last_report_tick) >= 500U)
    {
      last_report_tick = now;
      printf("[DIST %lums] count=%ld,%ld,%ld,%ld avg=%ldmm rpm=%d,%d,%d,%d\r\n",
             (unsigned long)(now - start_tick),
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
    HAL_Delay(1U);
  }

  Motor_BrakeAll();
  printf("[DIST] stop count=%ld,%ld,%ld,%ld avg=%ldmm\r\n",
         (long)Encoder_GetCount(MOTOR_1),
         (long)Encoder_GetCount(MOTOR_2),
         (long)Encoder_GetCount(MOTOR_3),
         (long)Encoder_GetCount(MOTOR_4),
         (long)Encoder_GetDistanceAvg());
}

static void Car_RunAngle(uint8_t speed, uint16_t radius_mm,
                         uint32_t angle_deg10, int8_t direction)
{
  int16_t outer;
  int16_t inner;
    int16_t left;
  int16_t right;
  uint32_t start_tick;
  uint32_t last_report_tick;
  uint32_t encoder_target_deg10;
  uint16_t comp_x1000;

  if (angle_deg10 == 0U)
  {
    Motor_BrakeAll();
    return;
  }

#if CAR_ANGLE_COMP_ENABLE
    if ((radius_mm == 0U) && (speed < CAR_ANGLE_MIN_SPEED))
  {
    printf("[ANGLE] rejected: speed %u < calibrated minimum %u\r\n",
           (unsigned)speed, (unsigned)CAR_ANGLE_MIN_SPEED);
    Motor_BrakeAll();
    return;
  }
#endif

  outer = (int16_t)Car_ToEff(speed);
  if (outer == 0)
  {
    Motor_BrakeAll();
    return;
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
    left  = inner;
    right = outer;
  }
  else
  {
    left  = outer;
    right = inner;
  }

  comp_x1000 = Car_AngleCompX1000(speed, radius_mm, direction);
  encoder_target_deg10 = (uint32_t)(((uint64_t)angle_deg10 * comp_x1000
                                      + 500ULL) / 1000ULL);

  Encoder_ResetAll();
  start_tick = HAL_GetTick();
  last_report_tick = start_tick;
  printf("[ANGLE] start dir=%s real=%lu.%01lu deg encoder=%lu.%01lu deg "
         "comp=%u.%03u radius=%umm speed=%u\r\n",
         (direction > 0) ? "left" : "right",
         (unsigned long)(angle_deg10 / 10U),
         (unsigned long)(angle_deg10 % 10U),
         (unsigned long)(encoder_target_deg10 / 10U),
         (unsigned long)(encoder_target_deg10 % 10U),
         (unsigned)(comp_x1000 / 1000U),
         (unsigned)(comp_x1000 % 1000U),
         (unsigned)radius_mm,
         (unsigned)speed);
  Car_Drive(left, right);

  while (1)
  {
    uint32_t now = HAL_GetTick();
    int32_t angle = Encoder_GetRotationDeg10(CAR_WHEEL_TRACK_MM);
    uint32_t turned = (angle >= 0)
                        ? (uint32_t)angle
                        : (uint32_t)(-(int64_t)angle);

    if (turned >= encoder_target_deg10)
    {
      break;
    }
    if ((uint32_t)(now - start_tick) >= CAR_DISTANCE_TIMEOUT_MS)
    {
      printf("[ANGLE] timeout\r\n");
      break;
    }
    if ((uint32_t)(now - last_report_tick) >= 500U)
    {
      last_report_tick = now;
      printf("[ANGLE %lums] left=%ldmm right=%ldmm angle=%ld.%01lddeg\r\n",
             (unsigned long)(now - start_tick),
             (long)Encoder_GetDistanceLeft(),
             (long)Encoder_GetDistanceRight(),
             (long)(angle / 10),
             (long)(angle >= 0 ? angle % 10 : -angle % 10));
    }
    HAL_Delay(1U);
  }

  Motor_BrakeAll();
  printf("[ANGLE] stop left=%ldmm right=%ldmm angle=%ld.%01lddeg\r\n",
         (long)Encoder_GetDistanceLeft(),
         (long)Encoder_GetDistanceRight(),
         (long)(Encoder_GetRotationDeg10(CAR_WHEEL_TRACK_MM) / 10),
         (long)(Encoder_GetRotationDeg10(CAR_WHEEL_TRACK_MM) >= 0
                  ? Encoder_GetRotationDeg10(CAR_WHEEL_TRACK_MM) % 10
                  : -Encoder_GetRotationDeg10(CAR_WHEEL_TRACK_MM) % 10));
}

/**
  * @brief  小车运动控制初始化
  * @note   内部调用 Motor_Init() 启动 8 路 PWM，调用前需先完成
  *         MX_TIM1_Init() 与 MX_TIM8_Init()。
  */
void Car_Init(void)
{
  Motor_Init();
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
  Motor_BrakeAll();
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
  int16_t  v0    = (int16_t)Car_ToEff(speed_from);
  int16_t  v1    = (int16_t)Car_ToEff(speed_to);
  uint16_t steps = (uint16_t)(time / CAR_RAMP_STEP_MS);

  for (uint16_t i = 1U; i <= steps; i++)
  {
    /* v0 + (v1-v0)·i/steps，整数运算先乘后除保精度 */
    int16_t v = (int16_t)(v0 + (int32_t)(v1 - v0) * (int32_t)i / (int32_t)steps);

    Car_Drive(v, v);
    HAL_Delay(CAR_RAMP_STEP_MS);
  }

  /* 不足一个步长的零头（含 time < CAR_RAMP_STEP_MS 的情况）按末速补齐 */
  if ((uint16_t)(time % CAR_RAMP_STEP_MS) > 0U)
  {
    Car_Drive(v1, v1);
    HAL_Delay((uint16_t)(time % CAR_RAMP_STEP_MS));
  }

  Motor_BrakeAll();
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
  Motor_BrakeAll();
  if (time > 0U)
  {
    HAL_Delay(time);
  }
}

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
