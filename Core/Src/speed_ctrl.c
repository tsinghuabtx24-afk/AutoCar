/**
  ******************************************************************************
  * @file    speed_ctrl.c
  * @brief   四路电机转速闭环（PI + 前馈）实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "speed_ctrl.h"
#include "encoder.h"

/* Private types -------------------------------------------------------------*/
typedef struct
{
  int16_t target_rpm;    /* 车体系目标转速，前进为正 */
  int16_t ramp_rpm;      /* 经斜坡限速后实际交给 PI 的目标，见 SpeedCtrl_Slew() */
  int32_t integral;      /* 误差积分，单位 rpm·周期 */
  int16_t duty;          /* 最近一次输出的占空比（车体系，带符号） */
} SpeedCtrl_Motor;

/* Private variables ---------------------------------------------------------*/

/* volatile：SysTick 里的 Step() 写，主循环通过 Get* 读。 */
static volatile SpeedCtrl_Motor sc[MOTOR_NUM];

/* 闭环是否接管电机。0 时 Step() 直接返回，不触碰 PWM，
   这样 Motor_BrakeAll() 的制动状态不会被下一次 SysTick 覆盖。 */
static volatile uint8_t sc_enabled;

/* 车体前进 → 各电机 Motor_SetSpeedRaw 的符号，由 Car_Init() 传入。
   NULL 表示尚未初始化，Step() 一律不动作。 */
static const int8_t *sc_polarity;

static uint32_t sc_last_tick;

/* Private functions ---------------------------------------------------------*/
#if SPEEDCTRL_ENABLE

/**
  * @brief  目标转速 → 前馈占空比
  * @param  rpm: 车体系目标转速（可负）
  * @retval 占空比 -100~100
  *
  * @note   前馈让 PI 只需要修正偏差，而不是从零把整个占空比积出来。
  *         没有前馈时，起步瞬间误差最大、积分还没涨起来，车会先顿一下；
  *         而且稳态完全靠积分维持，KI 稍小就压不住。
  *
  * @note   映射用"死区 + 线性"：占空比 = BASE + (MAX-BASE) * rpm / MAX_RPM。
  *         这正是原先开环那套 CAR_SPEED_BASE 补偿的含义 —— 死区那一段占空比
  *         对转速无贡献，所以线性区间要从 BASE 起算，不是从 0 起算。
  */
static int16_t SpeedCtrl_FeedForward(int16_t rpm)
{
  int32_t mag = (rpm >= 0) ? rpm : -(int32_t)rpm;
  int32_t duty;

  if (mag == 0)
  {
    return 0;
  }

  duty = (int32_t)MOTOR_SPEED_MIN
         + (int32_t)(MOTOR_SPEED_MAX - MOTOR_SPEED_MIN) * mag
           / (int32_t)ENC_MAX_RPM;

  if (duty > (int32_t)MOTOR_SPEED_MAX)
  {
    duty = (int32_t)MOTOR_SPEED_MAX;
  }

  return (rpm >= 0) ? (int16_t)duty : (int16_t)(-duty);
}

/**
  * @brief  把占空比夹到合法区间并保证能克服死区
  * @param  duty:   PI 算出的占空比（带符号）
  * @param  moving: 目标是否非零。为 0 时不施加下限，允许输出 0。
  */
static int16_t SpeedCtrl_ClampDuty(int32_t duty, uint8_t moving)
{
  int32_t mag = (duty >= 0) ? duty : -duty;

  if (mag > (int32_t)MOTOR_SPEED_MAX)
  {
    mag = (int32_t)MOTOR_SPEED_MAX;
  }

  /* 目标非零时抬到下限：低于这个值电机只嗡嗡不转，编码器没有反馈，
     PI 就成了开环，积分会一路涨到限幅。 */
  if ((moving != 0U) && (mag < (int32_t)SPEEDCTRL_OUT_MIN))
  {
    mag = (int32_t)SPEEDCTRL_OUT_MIN;
  }

  return (duty >= 0) ? (int16_t)mag : (int16_t)(-mag);
}

/**
  * @brief  对单只电机跑一次 PI
  */
/**
  * @brief  目标转速斜坡限速
  * @param  current: 上一周期的斜坡输出
  * @param  target:  调用方设定的目标
  * @retval 本周期允许的目标转速
  *
  * @note   只限制"增大"。减速立即生效 —— 制动、循迹急转、避障退让都要求马上
  *         降速，为了平顺去拖慢它们是拿安全换手感。
  *
  * @note   换向先经过 0：从 +60 直接要 -60 时本周期输出 0，下周期再从 0 往负
  *         方向爬。不这么做的话反向的那一瞬间等于全压反接，浪涌比启动还大，
  *         恰好是斜坡要避免的事。原地旋转就是这种场合（左右目标反号）。
  */
static int16_t SpeedCtrl_Slew(int16_t current, int16_t target)
{
  int32_t c = current;
  int32_t t = target;
  int32_t mag_c;
  int32_t mag_t;

  if (SPEEDCTRL_RAMP_STEP <= 0)
  {
    return target;                          /* 斜坡关闭 */
  }

  /* 换向：先归零。c != 0 的判断不能省，否则静止时目标为负会被永久卡在 0。 */
  if ((c != 0) && (t != 0) && (((c > 0) && (t < 0)) || ((c < 0) && (t > 0))))
  {
    return 0;
  }

  mag_c = (c >= 0) ? c : -c;
  mag_t = (t >= 0) ? t : -t;

  if (mag_t <= mag_c)
  {
    return target;                          /* 减速或不变：直接给 */
  }

  if ((mag_t - mag_c) <= SPEEDCTRL_RAMP_STEP)
  {
    return target;                          /* 差额不足一步，一次到位 */
  }

  mag_c += SPEEDCTRL_RAMP_STEP;
  return (int16_t)((t >= 0) ? mag_c : -mag_c);
}

static void SpeedCtrl_StepMotor(Motor_ID m)
{
  int16_t target = sc[m].target_rpm;
  int16_t actual;
  int32_t error;
  int32_t out;
  int16_t duty;
  int32_t mag = (target >= 0) ? target : -(int32_t)target;

  /* 死区判断用的是**调用方设定的**目标，不是斜坡后的值：目标一旦给 0 就要
     立刻停，不能等斜坡降完。 */
  if (mag < (int32_t)SPEEDCTRL_TARGET_DEAD)
  {
    sc[m].ramp_rpm = 0;                     /* 清掉，否则下次起步从残留值开始 */
    sc[m].integral = 0;
    sc[m].duty     = 0;
    Motor_Coast(m);
    return;
  }

  /* 斜坡：PI 跟踪的是 ramp_rpm，不是 target_rpm。前馈也用 ramp_rpm —— 用
     target_rpm 会让前馈一步顶到终值，斜坡就形同虚设。 */
  sc[m].ramp_rpm = SpeedCtrl_Slew(sc[m].ramp_rpm, target);
  target         = sc[m].ramp_rpm;
  mag            = (target >= 0) ? target : -(int32_t)target;

  /* 斜坡正好过零（换向途中）：这一周期滑行。积分不清 —— 命令还在生效，
     清了等于每次换向都丢掉已经积累的工作点。 */
  if (mag < (int32_t)SPEEDCTRL_TARGET_DEAD)
  {
    sc[m].duty = 0;
    Motor_Coast(m);
    return;
  }

  actual = Encoder_GetSpeedRpm(m);          /* 已含极性补偿，前进为正 */
  error  = (int32_t)target - (int32_t)actual;

  /* 积分累加并限幅，防止堵转或起步时饱和 */
  sc[m].integral += error;
  if (sc[m].integral > SPEEDCTRL_I_LIMIT)
  {
    sc[m].integral = SPEEDCTRL_I_LIMIT;
  }
  else if (sc[m].integral < -SPEEDCTRL_I_LIMIT)
  {
    sc[m].integral = -SPEEDCTRL_I_LIMIT;
  }

  out = (int32_t)SpeedCtrl_FeedForward(target)
        + error * SPEEDCTRL_KP_NUM / SPEEDCTRL_KP_DEN
        + sc[m].integral * SPEEDCTRL_KI_NUM / SPEEDCTRL_KI_DEN;

  duty       = SpeedCtrl_ClampDuty(out, 1U);
  sc[m].duty = duty;

  /* 车体系 → 电机系。极性表是 car.c 的 CAR_*_POLARITY 组合结果。 */
  Motor_SetSpeedRaw(m, (int8_t)(duty * sc_polarity[m]));
}

#endif /* SPEEDCTRL_ENABLE */

/* Exported functions --------------------------------------------------------*/

void SpeedCtrl_Init(const int8_t *out_polarity)
{
  sc_polarity = out_polarity;
  sc_enabled  = 0U;
  sc_last_tick = HAL_GetTick();

  for (Motor_ID m = MOTOR_1; m < MOTOR_NUM; m++)
  {
    sc[m].target_rpm = 0;
    sc[m].ramp_rpm   = 0;
    sc[m].integral   = 0;
    sc[m].duty       = 0;
  }
}

void SpeedCtrl_SetTargetRpm(int16_t left_rpm, int16_t right_rpm)
{
  sc[MOTOR_1].target_rpm = left_rpm;
  sc[MOTOR_2].target_rpm = left_rpm;
  sc[MOTOR_3].target_rpm = right_rpm;
  sc[MOTOR_4].target_rpm = right_rpm;

  /* 目标一旦设定就接管，哪怕是 0（0 表示滑行，与原开环 Car_Drive(0,0) 一致）。
     真要制动必须显式 Disable + Motor_BrakeAll()。 */
  sc_enabled = 1U;
}

void SpeedCtrl_Disable(void)
{
  /* 先置 0 再动电机：这个顺序保证之后 SysTick 里的 Step() 立刻返回，
     不会在调用方制动之后又写一次 PWM 把制动状态冲掉。 */
  sc_enabled = 0U;

  for (Motor_ID m = MOTOR_1; m < MOTOR_NUM; m++)
  {
    sc[m].target_rpm = 0;
    sc[m].ramp_rpm   = 0;                   /* 制动后必须清：下次起步要从 0 爬 */
    sc[m].integral   = 0;
    sc[m].duty       = 0;
  }
}

void SpeedCtrl_Step(void)
{
#if SPEEDCTRL_ENABLE
  uint32_t now;

  if ((sc_enabled == 0U) || (sc_polarity == NULL))
  {
    return;
  }

  now = HAL_GetTick();
  if ((uint32_t)(now - sc_last_tick) < SPEEDCTRL_PERIOD_MS)
  {
    return;                    /* 转速值还没更新，跑了也是白积分 */
  }
  sc_last_tick = now;

  for (Motor_ID m = MOTOR_1; m < MOTOR_NUM; m++)
  {
    SpeedCtrl_StepMotor(m);
  }
#endif
}

uint8_t SpeedCtrl_IsEnabled(void)
{
  return sc_enabled;
}

int16_t SpeedCtrl_GetTargetRpm(Motor_ID motor)
{
  return (motor < MOTOR_NUM) ? sc[motor].target_rpm : 0;
}

int16_t SpeedCtrl_GetDuty(Motor_ID motor)
{
  return (motor < MOTOR_NUM) ? sc[motor].duty : 0;
}
