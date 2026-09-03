/**
  ******************************************************************************
  * @file    encoder.c
  * @brief   四路电机霍尔编码器读取实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "encoder.h"

/* Private types -------------------------------------------------------------*/
typedef struct
{
  int32_t  count;        /* 累计计数，已施加方向补偿 */
  uint16_t last_cnt;     /* 上次读到的 CNT 原始值 */
  int32_t  count_mark;   /* 上次算转速时的 count 快照 */
  int16_t  rpm;          /* 最近一次算出的转速 */
} Encoder_State;

/* Private variables ---------------------------------------------------------*/

/* volatile：Encoder_Update() 在 SysTick 中断里写，主循环通过 Get* 接口读。
   不加的话 -O2 会把主循环里的读缓存进寄存器，里程看起来永远不变。 */
static volatile Encoder_State enc[MOTOR_NUM];
static uint32_t               enc_last_tick;   /* 只在中断内使用 */

/* SysTick 在 HAL_Init() 里就开始跳了，比 Encoder_Init() 早得多。那时
   htim4.Instance 还是 NULL，读 CNT 等于解引用空指针。用这个标志挡住，
   Encoder_Init() 完成后才真正开始采样。 */
static volatile uint8_t       enc_ready;

/* 电机编号 → 编码器定时器。顺序与 Motor_ID 一致。 */
static TIM_HandleTypeDef *const enc_tim[MOTOR_NUM] =
{
  &htim4,   /* M1 左前 */
  &htim2,   /* M2 左后 */
  &htim5,   /* M3 右前 */
  &htim3,   /* M4 右后 */
};

static const int8_t enc_polarity[MOTOR_NUM] =
{
  ENC_POLARITY_M1,
  ENC_POLARITY_M2,
  ENC_POLARITY_M3,
  ENC_POLARITY_M4,
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  读取一路编码器自上次调用以来的 CNT 增量
  * @param  motor: 电机编号
  * @retval 增量计数，正负表示转向（未施加方向补偿）
  *
  * @note   CNT 是 16 位且会自然绕回，直接相减在绕回处会得到一个巨大的
  *         错误值。这里按**模运算取最短路径**还原真实增量：把差值折算到
  *         (-period/2, +period/2] 区间内，即假定两次采样之间轮子转过的
  *         计数不超过半个量程。ARR=65535 时半量程 32768 计数，即使 x4
  *         倍频、1040 计数每圈也有 31 圈的余量，20ms 采样周期下绝对够。
  *
  * @note   period 从 ARR 实时读取而不是写死 65536，这样 CubeMX 里
  *         Counter Period 填多少都能算对（当前工程里是 3600-1）。
  */
static int32_t Encoder_ReadDelta(Motor_ID motor)
{
  uint16_t now    = (uint16_t)__HAL_TIM_GET_COUNTER(enc_tim[motor]);
  int32_t  period = (int32_t)__HAL_TIM_GET_AUTORELOAD(enc_tim[motor]) + 1;
  int32_t  delta  = (int32_t)now - (int32_t)enc[motor].last_cnt;

  enc[motor].last_cnt = now;

  /* 折算到 (-period/2, +period/2]，还原绕回 */
  if (delta > (period / 2))
  {
    delta -= period;
  }
  else if (delta < -(period / 2))
  {
    delta += period;
  }

  return delta;
}

/**
  * @brief  计数 → 毫米
  * @note   mm = counts × π·D / COUNTS_PER_REV × CAL_NUM / CAL_DEN
  *         π 用 355/113 近似（误差 8.5e-8，远小于标定误差）。
  *         全程 int64 中间量，避免长距离行驶时溢出：counts 到 1e6 量级时
  *         乘 355 再乘 D 已经超出 int32。
  */
static int32_t Encoder_CountToMm(int32_t counts)
{
  int64_t num = (int64_t)counts * 355LL * (int64_t)ENC_WHEEL_DIA_MM * ENC_CAL_NUM;
  int64_t den = 113LL * (int64_t)ENC_COUNTS_PER_REV * ENC_CAL_DEN;

  return (int32_t)(num / den);
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  启动四路编码器
  * @note   必须在 MX_TIM2/3/4/5_Init() 之后调用。
  *         每路都启动 CH1+CH2：正交解码要两相同时参与，只启动一个通道时
  *         x4 模式下会漏掉一半边沿，x2 模式下方向判断也会失效。
  */
void Encoder_Init(void)
{
  for (Motor_ID m = MOTOR_1; m < MOTOR_NUM; m++)
  {
    HAL_TIM_Encoder_Start(enc_tim[m], TIM_CHANNEL_ALL);
  }
  Encoder_ResetAll();
  enc_last_tick = HAL_GetTick();
  enc_ready     = 1U;                      /* 放最后：之前中断里不采样 */
}

/**
  * @brief  采样四路编码器，累加里程并按需更新转速
  * @note   需要周期调用。调用越勤越不容易漏掉 CNT 绕回，代价只是几十条
  *         指令。转速受 ENC_SPEED_PERIOD_MS 限流，调得再勤也不会因为
  *         采样间隔太短而抖动。
  */
void Encoder_Update(void)
{
  uint32_t now;
  uint32_t elapsed;

  if (enc_ready == 0U)
  {
    return;                                 /* 定时器还没初始化 */
  }

  now     = HAL_GetTick();
  elapsed = now - enc_last_tick;            /* 无符号相减，tick 溢出也对 */

  for (Motor_ID m = MOTOR_1; m < MOTOR_NUM; m++)
  {
    enc[m].count += Encoder_ReadDelta(m) * (int32_t)enc_polarity[m];
  }

  if (elapsed < ENC_SPEED_PERIOD_MS)
  {
    return;                                 /* 里程已累加，转速这次不更新 */
  }

  for (Motor_ID m = MOTOR_1; m < MOTOR_NUM; m++)
  {
    int32_t d = enc[m].count - enc[m].count_mark;

    /* rpm = 计数增量 / 每圈计数 × (60000ms / 间隔ms)，先乘后除保精度 */
    enc[m].rpm        = (int16_t)(d * 60000L
                                  / (int32_t)ENC_COUNTS_PER_REV
                                  / (int32_t)elapsed);
    enc[m].count_mark = enc[m].count;
  }

  enc_last_tick = now;
}

/**
  * @brief  取累计计数（已施加方向补偿，前进为正）
  */
int32_t Encoder_GetCount(Motor_ID motor)
{
  return (motor < MOTOR_NUM) ? enc[motor].count : 0;
}

/**
  * @brief  取累计里程 mm（前进为正）
  */
int32_t Encoder_GetDistance(Motor_ID motor)
{
  return (motor < MOTOR_NUM) ? Encoder_CountToMm(enc[motor].count) : 0;
}

/**
  * @brief  取转速 rpm（轮子转速，非电机轴转速；前进为正）
  * @note   由 Encoder_Update() 每 ENC_SPEED_PERIOD_MS 更新一次。
  */
int16_t Encoder_GetSpeedRpm(Motor_ID motor)
{
  return (motor < MOTOR_NUM) ? enc[motor].rpm : 0;
}

/**
  * @brief  四轮平均里程 mm，作为车体前进距离
  * @note   直线行驶时四轮里程接近，取平均可以抵消单轮打滑的影响。
  *         原地旋转时左右轮反向，平均值接近 0 —— 那种场合应该看
  *         Encoder_GetDistanceLeft/Right 的差值而不是平均值。
  */
int32_t Encoder_GetDistanceAvg(void)
{
  int32_t sum = 0;

  for (Motor_ID m = MOTOR_1; m < MOTOR_NUM; m++)
  {
    sum += enc[m].count;
  }

  return Encoder_CountToMm(sum / (int32_t)MOTOR_NUM);
}

/**
  * @brief  左侧两轮平均里程 mm
  */
int32_t Encoder_GetDistanceLeft(void)
{
  return Encoder_CountToMm((enc[MOTOR_1].count + enc[MOTOR_2].count) / 2);
}

/**
  * @brief  右侧两轮平均里程 mm
  */
int32_t Encoder_GetDistanceRight(void)
{
  return Encoder_CountToMm((enc[MOTOR_3].count + enc[MOTOR_4].count) / 2);
}

int32_t Encoder_GetRotationDeg10(uint16_t wheel_track_mm)
{
  int32_t left_mm;
  int32_t right_mm;
  int64_t delta_mm;

  if (wheel_track_mm == 0U)
  {
    return 0;
  }

  left_mm  = Encoder_GetDistanceLeft();
  right_mm = Encoder_GetDistanceRight();
  delta_mm = (int64_t)right_mm - (int64_t)left_mm;

  /* deg10 = (right-left) / track * 180/pi * 10
     572.96 用 57296/100 近似，避免使用浮点数。 */
  return (int32_t)(delta_mm * 57296LL / (int32_t)wheel_track_mm / 100LL);
}

/**
  * @brief  清零一路编码器的累计量
  * @note   同步刷新 last_cnt，否则下次 Update 会把清零期间的 CNT 变化
  *         当成增量补回来，等于没清干净。
  */
void Encoder_Reset(Motor_ID motor)
{
  if (motor >= MOTOR_NUM)
  {
    return;
  }
  enc[motor].count      = 0;
  enc[motor].count_mark = 0;
  enc[motor].rpm        = 0;
  enc[motor].last_cnt   = (uint16_t)__HAL_TIM_GET_COUNTER(enc_tim[motor]);
}

/**
  * @brief  清零四路编码器
  */
void Encoder_ResetAll(void)
{
  for (Motor_ID m = MOTOR_1; m < MOTOR_NUM; m++)
  {
    Encoder_Reset(m);
  }
}
