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
static volatile uint8_t ir_remote_repeat;
static uint32_t ir_remote_cycles_per_us;

/* 帧队列。中断只写 head，主循环只写 tail，单生产者单消费者，入队出队都只是
   指针搬移，不需要临界区。 */
static volatile IrRemote_Frame ir_remote_queue[IR_REMOTE_QUEUE_LEN];
static volatile uint8_t  ir_remote_head;
static volatile uint8_t  ir_remote_tail;
static volatile uint32_t ir_remote_dropped;

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
  ir_remote_repeat = 0U;
  ir_remote_head = 0U;
  ir_remote_tail = 0U;
  ir_remote_dropped = 0U;
}

/**
  * @brief  解码完成，校验后入队（中断上下文）
  * @note   校验放在入队前：坏帧根本不该占队列位置，否则连按时坏帧会把好帧
  *         挤掉。原实现把校验放在 Read 里，坏帧会让那一次 Read 直接失败。
  */
static void IrRemote_Enqueue(uint32_t data)
{
  uint8_t command         = (uint8_t)((data >> 16) & 0xFFU);
  uint8_t inverse_command = (uint8_t)((data >> 24) & 0xFFU);
  uint8_t next;

  if ((uint8_t)(command ^ inverse_command) != 0xFFU)
  {
    return;   /* 校验失败，丢弃，不占队列 */
  }

  next = (uint8_t)((ir_remote_head + 1U) % IR_REMOTE_QUEUE_LEN);
  if (next == ir_remote_tail)
  {
    /* 队列满：丢弃最新的一帧并计数。中断里不 printf。 */
    ir_remote_dropped++;
    return;
  }

  ir_remote_queue[ir_remote_head].address         = (uint8_t)(data & 0xFFU);
  ir_remote_queue[ir_remote_head].command         = command;
  ir_remote_queue[ir_remote_head].inverse_command = inverse_command;
  ir_remote_head = next;
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
    /* 入队一次即止。bit_count 停在 32，上面的 `>= 32U` 守卫会挡住后续杂波，
       直到下一个起始脉冲把它清零——沿用原实现的做法，只是把"置 ready 标志"
       换成"入队"。 */
    IrRemote_Enqueue(ir_remote_data);
  }
}

uint8_t IrRemote_Read(IrRemote_Frame *frame)
{
  if (frame == NULL) return 0U;
  if (ir_remote_tail == ir_remote_head) return 0U;   /* 队列空 */

  frame->address         = ir_remote_queue[ir_remote_tail].address;
  frame->command         = ir_remote_queue[ir_remote_tail].command;
  frame->inverse_command = ir_remote_queue[ir_remote_tail].inverse_command;
  ir_remote_tail = (uint8_t)((ir_remote_tail + 1U) % IR_REMOTE_QUEUE_LEN);
  return 1U;
}

uint32_t IrRemote_GetDroppedCount(void) { return ir_remote_dropped; }

uint8_t IrRemote_IsRepeat(void)
{
  uint8_t repeat;
  __disable_irq();
  repeat = ir_remote_repeat;
  ir_remote_repeat = 0U;
  __enable_irq();
  return repeat;
}
