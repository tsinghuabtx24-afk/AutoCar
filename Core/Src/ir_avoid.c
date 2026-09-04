#include "ir_avoid.h"
#include "adc.h"
#include "car.h"
#include "retarget.h"

#include <stdio.h>

/* 独立于 main 的红外避障执行参数。 */
#define IR_AVOID_FORWARD_SPEED       70U
#define IR_AVOID_REVERSE_SPEED       70U
#define IR_AVOID_ROTATE_SPEED        80U
#define IR_AVOID_REVERSE_DISTANCE_MM 50U
#define IR_AVOID_ROTATE_ANGLE_DEG10  300U
#define IR_AVOID_RGB_PERIOD_MS       200U

static IrAvoid_State ir_state;
static uint16_t ir_left_raw;
static uint16_t ir_right_raw;
static uint32_t ir_last_update;
static uint32_t ir_last_report;
static uint32_t ir_random_state = 0xA5C31F27UL;

static uint16_t IrAvoid_ReadChannel(uint32_t channel)
{
  ADC_ChannelConfTypeDef config = {0};
  uint32_t sum = 0U;

  config.Channel = channel;
  config.Rank = ADC_REGULAR_RANK_1;
  config.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  HAL_ADC_ConfigChannel(&hadc3, &config);

  for (uint8_t i = 0U; i < IR_AVOID_SAMPLE_COUNT; i++)
  {
    HAL_ADC_Start(&hadc3);
    if (HAL_ADC_PollForConversion(&hadc3, 10U) == HAL_OK)
    {
      sum += HAL_ADC_GetValue(&hadc3);
    }
    HAL_ADC_Stop(&hadc3);
  }

  return (uint16_t)(sum / IR_AVOID_SAMPLE_COUNT);
}

static uint8_t IrAvoid_Level(uint16_t value, uint16_t threshold,
                             uint8_t previous)
{
#if IR_AVOID_OBSTACLE_WHEN_HIGH
  if (previous != 0U)
  {
    return (value > (uint16_t)(threshold - IR_AVOID_HYSTERESIS)) ? 1U : 0U;
  }
  return (value >= threshold) ? 1U : 0U;
#else
  if (previous != 0U)
  {
    return (value < (uint16_t)(threshold + IR_AVOID_HYSTERESIS)) ? 1U : 0U;
  }
  return (value <= threshold) ? 1U : 0U;
#endif
}

void IrAvoid_Init(void)
{
  HAL_GPIO_WritePin(IR_AVOID_LEFT_ENABLE_PORT, IR_AVOID_LEFT_ENABLE_PIN,
                    IR_AVOID_DISABLE_LEVEL);
  HAL_GPIO_WritePin(IR_AVOID_RIGHT_ENABLE_PORT, IR_AVOID_RIGHT_ENABLE_PIN,
                    IR_AVOID_DISABLE_LEVEL);
  ir_state = IR_AVOID_CLEAR;
  ir_left_raw = 0U;
  ir_right_raw = 0U;
  ir_last_update = 0U;
  ir_last_report = 0U;
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
  RGB_SetColor(0U, 0U, 0U);
  printf("[IR] init: PF9=left PF10=right threshold=%u\r\n",
         (unsigned)IR_AVOID_LEFT_THRESHOLD);
}

void IrAvoid_Update(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t left_blocked;
  uint8_t right_blocked;

  if ((uint32_t)(now - ir_last_update) < IR_AVOID_UPDATE_PERIOD_MS)
  {
    return;
  }
  ir_last_update = now;

  HAL_GPIO_WritePin(IR_AVOID_LEFT_ENABLE_PORT, IR_AVOID_LEFT_ENABLE_PIN,
                    IR_AVOID_ENABLE_LEVEL);
  HAL_Delay(IR_AVOID_EMITTER_SETTLE_MS);
  ir_left_raw = IrAvoid_ReadChannel(IR_AVOID_LEFT_ADC_CHANNEL);
  HAL_GPIO_WritePin(IR_AVOID_LEFT_ENABLE_PORT, IR_AVOID_LEFT_ENABLE_PIN,
                    IR_AVOID_DISABLE_LEVEL);

  HAL_GPIO_WritePin(IR_AVOID_RIGHT_ENABLE_PORT, IR_AVOID_RIGHT_ENABLE_PIN,
                    IR_AVOID_ENABLE_LEVEL);
  HAL_Delay(IR_AVOID_EMITTER_SETTLE_MS);
  ir_right_raw = IrAvoid_ReadChannel(IR_AVOID_RIGHT_ADC_CHANNEL);
  HAL_GPIO_WritePin(IR_AVOID_RIGHT_ENABLE_PORT, IR_AVOID_RIGHT_ENABLE_PIN,
                    IR_AVOID_DISABLE_LEVEL);

  left_blocked = IrAvoid_Level(ir_left_raw, IR_AVOID_LEFT_THRESHOLD,
                               (ir_state == IR_AVOID_LEFT || ir_state == IR_AVOID_BOTH));
  right_blocked = IrAvoid_Level(ir_right_raw, IR_AVOID_RIGHT_THRESHOLD,
                                (ir_state == IR_AVOID_RIGHT || ir_state == IR_AVOID_BOTH));

  ir_state = (left_blocked != 0U)
               ? ((right_blocked != 0U) ? IR_AVOID_BOTH : IR_AVOID_LEFT)
               : ((right_blocked != 0U) ? IR_AVOID_RIGHT : IR_AVOID_CLEAR);

  if ((uint32_t)(now - ir_last_report) >= IR_AVOID_REPORT_PERIOD_MS)
  {
    ir_last_report = now;
    printf("[IR] state=%s left=%u right=%u\r\n",
           IrAvoid_StateName(ir_state),
           (unsigned)ir_left_raw, (unsigned)ir_right_raw);
  }
}

IrAvoid_State IrAvoid_GetState(void) { return ir_state; }
uint8_t IrAvoid_IsLeftBlocked(void) { return (ir_state == IR_AVOID_LEFT || ir_state == IR_AVOID_BOTH); }
uint8_t IrAvoid_IsRightBlocked(void) { return (ir_state == IR_AVOID_RIGHT || ir_state == IR_AVOID_BOTH); }
uint8_t IrAvoid_IsAnyBlocked(void) { return (ir_state != IR_AVOID_CLEAR) ? 1U : 0U; }
uint16_t IrAvoid_GetLeftRaw(void) { return ir_left_raw; }
uint16_t IrAvoid_GetRightRaw(void) { return ir_right_raw; }

const char *IrAvoid_StateName(IrAvoid_State state)
{
  switch (state)
  {
    case IR_AVOID_LEFT: return "LEFT";
    case IR_AVOID_RIGHT: return "RIGHT";
    case IR_AVOID_BOTH: return "BOTH";
    default: return "CLEAR";
  }
}

static void IrAvoid_UpdateAlert(IrAvoid_State state)
{
  static uint32_t last_blink_tick;
  static uint8_t blink_on;
  static IrAvoid_State last_alert_state = IR_AVOID_CLEAR;
  uint32_t now = HAL_GetTick();

  if (state == IR_AVOID_CLEAR)
  {
    blink_on = 0U;
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    RGB_SetColor(0U, 0U, 0U);
  }
  else
  {
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);

    if (state != last_alert_state)
    {
      blink_on = 1U;
      last_blink_tick = now;
    }
    else if ((uint32_t)(now - last_blink_tick) >= IR_AVOID_RGB_PERIOD_MS)
    {
      blink_on = (blink_on == 0U) ? 1U : 0U;
      last_blink_tick = now;
    }

    /* RGB_SetColor 的实际映射：g 控制左侧红色，r 控制右侧红色。 */
    RGB_SetColor(
        (uint8_t)(blink_on != 0U &&
                  (state == IR_AVOID_RIGHT || state == IR_AVOID_BOTH)),
        (uint8_t)(blink_on != 0U &&
                  (state == IR_AVOID_LEFT || state == IR_AVOID_BOTH)),
        0U);
  }

  last_alert_state = state;
}

static uint8_t IrAvoid_RandomBit(void)
{
  /* 软件伪随机：加入时间和 ADC 采样值，避免每次上电方向完全固定。 */
  ir_random_state ^= HAL_GetTick();
  ir_random_state ^= ((uint32_t)ir_left_raw << 16) | ir_right_raw;
  ir_random_state ^= ir_random_state << 13;
  ir_random_state ^= ir_random_state >> 17;
  ir_random_state ^= ir_random_state << 5;
  return (uint8_t)(ir_random_state & 1U);
}

static void IrAvoid_RotateAway(IrAvoid_State state)
{
  if (state == IR_AVOID_LEFT)
  {
    /* 左侧有障碍，向右旋转。 */
    Car_RotateRightAngle(IR_AVOID_ROTATE_SPEED, IR_AVOID_ROTATE_ANGLE_DEG10);
  }
  else if (state == IR_AVOID_RIGHT)
  {
    /* 右侧有障碍，向左旋转。 */
    Car_RotateLeftAngle(IR_AVOID_ROTATE_SPEED, IR_AVOID_ROTATE_ANGLE_DEG10);
  }
}

uint8_t IrAvoid_Handle(void)
{
  IrAvoid_State state;

  IrAvoid_Update();
  state = IrAvoid_GetState();
  IrAvoid_UpdateAlert(state);

  if (state == IR_AVOID_CLEAR)
  {
    IrAvoid_UpdateAlert(IR_AVOID_CLEAR);
    return 0U;
  }

  Car_Stop();
  /* 告警只覆盖本次避障动作，不因障碍仍在视野内而持续占用告警。 */
  IrAvoid_UpdateAlert(state);

  if (state == IR_AVOID_BOTH)
  {
    Car_BackwardDistance(IR_AVOID_REVERSE_SPEED,
                         IR_AVOID_REVERSE_DISTANCE_MM);
    /* 后退动作可能恰好未跨过采样周期，强制重新采样左右 ADC。 */
    ir_last_update = 0U;
    IrAvoid_Update();
    state = IrAvoid_GetState();
    IrAvoid_UpdateAlert(state);

    /* 后退后必须先随机转向 30°，不根据此刻的左右状态改变这一次转向。 */
    IrAvoid_UpdateAlert(IR_AVOID_BOTH);
    if (IrAvoid_RandomBit() != 0U)
    {
      Car_RotateLeftAngle(IR_AVOID_ROTATE_SPEED, IR_AVOID_ROTATE_ANGLE_DEG10);
    }
    else
    {
      Car_RotateRightAngle(IR_AVOID_ROTATE_SPEED, IR_AVOID_ROTATE_ANGLE_DEG10);
    }
    IrAvoid_UpdateAlert(IR_AVOID_CLEAR);
    /* 转向结束后下一轮再重新检测。 */
    ir_last_update = 0U;
    return 1U;
  }

  IrAvoid_RotateAway(state);
  IrAvoid_UpdateAlert(IR_AVOID_CLEAR);
  /* 下一次 Handle 必须重新采样，判断旋转后障碍是否仍存在。 */
  ir_last_update = 0U;
  return 1U;
}
