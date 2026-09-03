#include "ir_avoid.h"
#include "adc.h"
#include "retarget.h"

#include <stdio.h>

static IrAvoid_State ir_state;
static uint16_t ir_left_raw;
static uint16_t ir_right_raw;
static uint32_t ir_last_update;
static uint32_t ir_last_report;

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
