#include "ir_remote.h"

#define IR_REMOTE_START_MIN_US  12000U
#define IR_REMOTE_START_MAX_US  16000U
#define IR_REMOTE_BIT0_MAX_US   1600U
#define IR_REMOTE_BIT1_MIN_US   1600U
#define IR_REMOTE_REPEAT_MIN_US 100000U
#define IR_REMOTE_REPEAT_MAX_US 140000U

static volatile uint8_t ir_remote_bit_count;
static volatile uint32_t ir_remote_last_fall_us;
static volatile uint32_t ir_remote_data;
static volatile uint8_t ir_remote_ready;
static volatile uint8_t ir_remote_repeat;
static uint32_t ir_remote_cycles_per_us;

static uint32_t IrRemote_NowUs(void) { return DWT->CYCCNT / ir_remote_cycles_per_us; }

void IrRemote_Init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0U;
  ir_remote_cycles_per_us = SystemCoreClock / 1000000U;
  if (ir_remote_cycles_per_us == 0U) ir_remote_cycles_per_us = 1U;
  ir_remote_bit_count = 0U;
  ir_remote_last_fall_us = 0U;
  ir_remote_data = 0U;
  ir_remote_ready = 0U;
  ir_remote_repeat = 0U;
}

void IrRemote_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t now;
  uint32_t elapsed;
  if (GPIO_Pin != IR_IN_Pin) return;

  now = IrRemote_NowUs();
  elapsed = (uint32_t)(now - ir_remote_last_fall_us);
  ir_remote_last_fall_us = now;

  if (elapsed >= IR_REMOTE_REPEAT_MIN_US && elapsed <= IR_REMOTE_REPEAT_MAX_US)
  {
    ir_remote_repeat = 1U;
    return;
  }

  if (elapsed >= IR_REMOTE_START_MIN_US && elapsed <= IR_REMOTE_START_MAX_US)
  {
    ir_remote_bit_count = 0U;
    ir_remote_data = 0U;
    ir_remote_repeat = 0U;
    return;
  }

  if (ir_remote_bit_count >= 32U) return;
  if (elapsed < 700U || elapsed > 3000U) return;

  if (elapsed >= IR_REMOTE_BIT1_MIN_US)
  {
    ir_remote_data |= (uint32_t)1U << ir_remote_bit_count;
  }
  ir_remote_bit_count++;
  if (ir_remote_bit_count == 32U)
  {
    ir_remote_ready = 1U;
  }
}

uint8_t IrRemote_Read(IrRemote_Frame *frame)
{
  uint32_t data;
  uint8_t command;
  uint8_t inverse_command;
  if (frame == NULL || ir_remote_ready == 0U) return 0U;
  __disable_irq();
  data = ir_remote_data;
  ir_remote_ready = 0U;
  __enable_irq();
  frame->address = (uint8_t)(data & 0xFFU);
  command = (uint8_t)((data >> 16) & 0xFFU);
  inverse_command = (uint8_t)((data >> 24) & 0xFFU);
  if ((uint8_t)(command ^ inverse_command) != 0xFFU)
  {
    return 0U;
  }
  frame->command = command;
  frame->inverse_command = inverse_command;
  return 1U;
}

uint8_t IrRemote_IsRepeat(void)
{
  uint8_t repeat;
  __disable_irq();
  repeat = ir_remote_repeat;
  ir_remote_repeat = 0U;
  __enable_irq();
  return repeat;
}
