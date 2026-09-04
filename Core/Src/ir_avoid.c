/**
  ******************************************************************************
  * @file    ir_avoid.c
  * @brief   双路红外避障传感器实现
  ******************************************************************************
  * @note    只采样、只判状态、只发事件。避障动作见 avoid_task.c。
  ******************************************************************************
  */

#include "ir_avoid.h"
#include "adc.h"
#include "event.h"
#include "retarget.h"

#include <stdio.h>

/* 采样相位。发射管使能后需要 IR_AVOID_EMITTER_SETTLE_MS 稳定，
   原来用 HAL_Delay 等，现在拆成相位由主循环推进。 */
typedef enum
{
  IR_PHASE_IDLE = 0,      /* 等下一轮采样周期 */
  IR_PHASE_LEFT_SETTLE,   /* 左发射管已开，等稳定 */
  IR_PHASE_RIGHT_SETTLE   /* 右发射管已开，等稳定 */
} IrAvoid_Phase;

static IrAvoid_State ir_state;
static IrAvoid_Phase ir_phase;
static uint16_t ir_left_raw;
static uint16_t ir_right_raw;
static uint32_t ir_last_update;
static uint32_t ir_phase_tick;
static uint32_t ir_last_report;
static uint8_t  ir_has_sample;

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

static void IrAvoid_SetEmitter(uint8_t left_on, uint8_t right_on)
{
  HAL_GPIO_WritePin(IR_AVOID_LEFT_ENABLE_PORT, IR_AVOID_LEFT_ENABLE_PIN,
                    (left_on != 0U) ? IR_AVOID_ENABLE_LEVEL
                                    : IR_AVOID_DISABLE_LEVEL);
  HAL_GPIO_WritePin(IR_AVOID_RIGHT_ENABLE_PORT, IR_AVOID_RIGHT_ENABLE_PIN,
                    (right_on != 0U) ? IR_AVOID_ENABLE_LEVEL
                                     : IR_AVOID_DISABLE_LEVEL);
}

void IrAvoid_Init(void)
{
  ir_state       = IR_AVOID_CLEAR;
  ir_phase       = IR_PHASE_IDLE;
  ir_left_raw    = 0U;
  ir_right_raw   = 0U;
  ir_last_update = 0U;
  ir_phase_tick  = 0U;
  ir_last_report = 0U;
  ir_has_sample  = 0U;

  IrAvoid_SetEmitter(0U, 0U);
  printf("[IR] init: PF9=left PF10=right threshold=%u\r\n",
         (unsigned)IR_AVOID_LEFT_THRESHOLD);
}

void IrAvoid_Restart(void)
{
  IrAvoid_SetEmitter(0U, 0U);
  ir_phase       = IR_PHASE_IDLE;
  ir_last_update = 0U;   /* 立刻允许下一轮采样 */
  ir_has_sample  = 0U;
}

/**
  * @brief  两路原始值都拿到后，更新状态并在变化时发事件
  */
static void IrAvoid_Commit(uint32_t now)
{
  IrAvoid_State previous = ir_state;
  uint8_t left_blocked;
  uint8_t right_blocked;

  left_blocked = IrAvoid_Level(ir_left_raw, IR_AVOID_LEFT_THRESHOLD,
                               (previous == IR_AVOID_LEFT ||
                                previous == IR_AVOID_BOTH));
  right_blocked = IrAvoid_Level(ir_right_raw, IR_AVOID_RIGHT_THRESHOLD,
                                (previous == IR_AVOID_RIGHT ||
                                 previous == IR_AVOID_BOTH));

  ir_state = (left_blocked != 0U)
               ? ((right_blocked != 0U) ? IR_AVOID_BOTH : IR_AVOID_LEFT)
               : ((right_blocked != 0U) ? IR_AVOID_RIGHT : IR_AVOID_CLEAR);
  ir_has_sample = 1U;

  /* 只在状态变化时发事件，避免每 20ms 灌满队列。 */
  if (ir_state != previous)
  {
    switch (ir_state)
    {
      case IR_AVOID_LEFT:  (void)Event_Post(EVENT_OBSTACLE_LEFT,  0U); break;
      case IR_AVOID_RIGHT: (void)Event_Post(EVENT_OBSTACLE_RIGHT, 0U); break;
      case IR_AVOID_BOTH:  (void)Event_Post(EVENT_OBSTACLE_BOTH,  0U); break;
      default:             (void)Event_Post(EVENT_OBSTACLE_CLEAR, 0U); break;
    }
  }

  if ((uint32_t)(now - ir_last_report) >= IR_AVOID_REPORT_PERIOD_MS)
  {
    ir_last_report = now;
    printf("[IR] state=%s left=%u right=%u\r\n",
           IrAvoid_StateName(ir_state),
           (unsigned)ir_left_raw, (unsigned)ir_right_raw);
  }
}

void IrAvoid_Sense(void)
{
  uint32_t now = HAL_GetTick();

  switch (ir_phase)
  {
    case IR_PHASE_IDLE:
      if ((uint32_t)(now - ir_last_update) < IR_AVOID_UPDATE_PERIOD_MS)
      {
        return;
      }
      ir_last_update = now;
      /* 开左发射管，等稳定。 */
      IrAvoid_SetEmitter(1U, 0U);
      ir_phase_tick = now;
      ir_phase      = IR_PHASE_LEFT_SETTLE;
      break;

    case IR_PHASE_LEFT_SETTLE:
      if ((uint32_t)(now - ir_phase_tick) < IR_AVOID_EMITTER_SETTLE_MS)
      {
        return;
      }
      ir_left_raw = IrAvoid_ReadChannel(IR_AVOID_LEFT_ADC_CHANNEL);
      /* 关左、开右，等稳定。 */
      IrAvoid_SetEmitter(0U, 1U);
      ir_phase_tick = now;
      ir_phase      = IR_PHASE_RIGHT_SETTLE;
      break;

    case IR_PHASE_RIGHT_SETTLE:
    default:
      if ((uint32_t)(now - ir_phase_tick) < IR_AVOID_EMITTER_SETTLE_MS)
      {
        return;
      }
      ir_right_raw = IrAvoid_ReadChannel(IR_AVOID_RIGHT_ADC_CHANNEL);
      IrAvoid_SetEmitter(0U, 0U);
      ir_phase = IR_PHASE_IDLE;
      IrAvoid_Commit(now);
      break;
  }
}

/**
  * @brief  阻塞式单次采样，仅供独立标定使用
  */
void IrAvoid_UpdateBlocking(void)
{
  uint32_t now;

  IrAvoid_SetEmitter(1U, 0U);
  HAL_Delay(IR_AVOID_EMITTER_SETTLE_MS);
  ir_left_raw = IrAvoid_ReadChannel(IR_AVOID_LEFT_ADC_CHANNEL);

  IrAvoid_SetEmitter(0U, 1U);
  HAL_Delay(IR_AVOID_EMITTER_SETTLE_MS);
  ir_right_raw = IrAvoid_ReadChannel(IR_AVOID_RIGHT_ADC_CHANNEL);

  IrAvoid_SetEmitter(0U, 0U);
  now = HAL_GetTick();
  IrAvoid_Commit(now);
}

IrAvoid_State IrAvoid_GetState(void) { return ir_state; }
uint8_t IrAvoid_IsLeftBlocked(void)  { return (ir_state == IR_AVOID_LEFT  || ir_state == IR_AVOID_BOTH); }
uint8_t IrAvoid_IsRightBlocked(void) { return (ir_state == IR_AVOID_RIGHT || ir_state == IR_AVOID_BOTH); }
uint8_t IrAvoid_IsAnyBlocked(void)   { return (ir_state != IR_AVOID_CLEAR) ? 1U : 0U; }
uint16_t IrAvoid_GetLeftRaw(void)    { return ir_left_raw; }
uint16_t IrAvoid_GetRightRaw(void)   { return ir_right_raw; }
uint8_t IrAvoid_HasSample(void)      { return ir_has_sample; }

const char *IrAvoid_StateName(IrAvoid_State state)
{
  switch (state)
  {
    case IR_AVOID_LEFT:  return "LEFT";
    case IR_AVOID_RIGHT: return "RIGHT";
    case IR_AVOID_BOTH:  return "BOTH";
    default:             return "CLEAR";
  }
}
