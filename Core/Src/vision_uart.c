#include "vision_uart.h"
#include "usart.h"
#include "event.h"

#include <stdio.h>

static volatile uint8_t vision_rx_byte;
static volatile uint8_t vision_frame_id;
static volatile uint8_t vision_frame_step;
static volatile Vision_Event vision_event;
static volatile uint8_t vision_event_ready;
static volatile uint32_t vision_rx_count;
static volatile uint8_t vision_last_byte;
static volatile uint32_t vision_error_count;
static volatile uint32_t vision_last_error;
static volatile uint32_t vision_last_byte_tick;
static volatile uint32_t vision_resync_count;

/* 同图案静默窗：每个类别上次放行的时刻。只在 ISR 内读写，无需临界区。
   0 表示还没放行过。 */
static volatile uint32_t vision_pass_tick[VISION_TARGET_COUNT];
static volatile uint8_t  vision_pass_valid[VISION_TARGET_COUNT];
static volatile uint32_t vision_silenced_count;

/**
  * @brief  判断某类别此刻是否放行，放行则顺手续窗
  * @param  target 识别到的类别
  * @param  now    当前 tick
  * @retval 1=放行（应投事件），0=在静默窗内，丢弃
  */
static uint8_t VisionUart_TakeSilenceSlot(Vision_Target target, uint32_t now)
{
  uint8_t idx = (uint8_t)target;

  if (idx >= (uint8_t)VISION_TARGET_COUNT)
  {
    return 0U;
  }

  if ((vision_pass_valid[idx] != 0U) &&
      ((uint32_t)(now - vision_pass_tick[idx]) < VISION_REPEAT_SILENCE_MS))
  {
    vision_silenced_count++;
    return 0U;
  }

  /* 窗口从"放行时刻"起算，不是从最后一帧起算——否则牌子一直在视野里，
     窗口被每一帧不断续期，3s 之后也永远放不出第二次。 */
  vision_pass_tick[idx]  = now;
  vision_pass_valid[idx] = 1U;
  return 1U;
}

/**
  * @brief  视觉目标 → 全车事件
  */
static Event_Type VisionUart_TargetToEvent(Vision_Target target)
{
  switch (target)
  {
    case VISION_TARGET_NO_ENTRY:        return EVENT_VISION_NO_ENTRY;
    case VISION_TARGET_SLOW_AHEAD:      return EVENT_VISION_SLOW_AHEAD;
    case VISION_TARGET_FRIENDLY:        return EVENT_VISION_FRIENDLY;
    case VISION_TARGET_TUNNEL:          return EVENT_VISION_TUNNEL;
    case VISION_TARGET_NARROW_STREET:   return EVENT_VISION_NARROW_STREET;
    case VISION_TARGET_WAREHOUSE:       return EVENT_VISION_WAREHOUSE;
    case VISION_TARGET_COLLAPSED_HOUSE: return EVENT_VISION_COLLAPSED_HOUSE;
    default:                            return EVENT_NONE;
  }
}

/**
  * @brief  线上id → 类别
  * @retval 0xFF 表示id不在约定范围内
  */
static Vision_Target VisionUart_IdToTarget(uint8_t id)
{
  /* 下界用 > 0 包一层：VISION_ID_FIRST 现在是 0，直接写 id < FIRST 对无符号数
     恒假，编译器会警告比较无意义。基数改回非 0 时这段仍然正确。 */
#if (VISION_ID_FIRST > 0U)
  if (id < VISION_ID_FIRST)
  {
    return (Vision_Target)0xFFU;
  }
#endif

  if (id >= (uint8_t)(VISION_ID_FIRST + (uint8_t)VISION_TARGET_COUNT))
  {
    return (Vision_Target)0xFFU;
  }
  return (Vision_Target)(uint8_t)(id - VISION_ID_FIRST);
}

void VisionUart_Init(void)
{
  vision_frame_step = 0U;
  vision_frame_id = 0U;
  vision_event_ready = 0U;
  vision_rx_count = 0U;
  vision_last_byte = 0U;
  vision_error_count = 0U;
  vision_last_error = 0U;
  vision_last_byte_tick = HAL_GetTick();
  vision_resync_count = 0U;
  vision_silenced_count = 0U;
  for (uint8_t i = 0U; i < (uint8_t)VISION_TARGET_COUNT; i++)
  {
    vision_pass_tick[i]  = 0U;
    vision_pass_valid[i] = 0U;
  }
  if (HAL_UART_Receive_IT(&huart2, (uint8_t *)&vision_rx_byte, 1U) != HAL_OK)
  {
    printf("[VISION] UART2 receive start failed\r\n");
  }
}

void VisionUart_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t checksum;
  Vision_Target target;
  uint32_t now;

  if (huart != &huart2) return;

  now = HAL_GetTick();
  vision_rx_count++;
  vision_last_byte = vision_rx_byte;

  if (((uint32_t)(now - vision_last_byte_tick) >= VISION_FRAME_TIMEOUT_MS) &&
      (vision_frame_step != 0U))
  {
    vision_frame_step = 0U;
    vision_resync_count++;
  }
  vision_last_byte_tick = now;

  switch (vision_frame_step)
  {
    case 0U:
      if (vision_rx_byte == 0xAAU)
      {
        vision_frame_step = 1U;
      }
      break;

    case 1U:
      vision_frame_id = vision_rx_byte;
      vision_frame_step = 2U;
      break;

    default:
      checksum = (uint8_t)(0xAAU ^ vision_frame_id);
      target = VisionUart_IdToTarget(vision_frame_id);
      if (vision_rx_byte == checksum && target != (Vision_Target)0xFFU)
      {
        /* 诊断槽照常更新：静默窗只拦事件投递，不影响"最近识别到什么"的观测。 */
        vision_event.target = target;
        vision_event.id = vision_frame_id;
        vision_event_ready = 1U;

        if (VisionUart_TakeSilenceSlot(target, now) != 0U)
        {
          (void)Event_Post(VisionUart_TargetToEvent(target), vision_frame_id);
        }
        vision_frame_step = 0U;
      }
      else
      {
        vision_error_count++;
        if (vision_rx_byte == 0xAAU)
        {
          vision_frame_step = 1U;
          vision_resync_count++;
        }
        else
        {
          vision_frame_step = 0U;
        }
      }
      break;
  }

  (void)HAL_UART_Receive_IT(&huart2, (uint8_t *)&vision_rx_byte, 1U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  VisionUart_RxCpltCallback(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == &huart2)
  {
    vision_error_count++;
    vision_last_error = huart->ErrorCode;
    vision_frame_step = 0U;
    (void)HAL_UART_Receive_IT(&huart2, (uint8_t *)&vision_rx_byte, 1U);
  }
}

uint8_t VisionUart_ReadEvent(Vision_Event *event)
{
  if (event == NULL || vision_event_ready == 0U) return 0U;
  __disable_irq();
  *event = vision_event;
  vision_event_ready = 0U;
  __enable_irq();
  return 1U;
}

uint32_t VisionUart_GetRxCount(void) { return vision_rx_count; }
uint8_t VisionUart_GetLastByte(void) { return vision_last_byte; }
uint32_t VisionUart_GetErrorCount(void) { return vision_error_count; }
uint32_t VisionUart_GetResyncCount(void) { return vision_resync_count; }
uint32_t VisionUart_GetSilencedCount(void) { return vision_silenced_count; }
uint32_t VisionUart_GetLastError(void) { return vision_last_error; }

const char *VisionUart_TargetName(Vision_Target target)
{
  switch (target)
  {
    case VISION_TARGET_NO_ENTRY:        return "NO_ENTRY";
    case VISION_TARGET_SLOW_AHEAD:      return "SLOW_AHEAD";
    case VISION_TARGET_FRIENDLY:        return "FRIENDLY";
    case VISION_TARGET_TUNNEL:          return "TUNNEL";
    case VISION_TARGET_NARROW_STREET:   return "NARROW_STREET";
    case VISION_TARGET_WAREHOUSE:       return "WAREHOUSE";
    case VISION_TARGET_COLLAPSED_HOUSE: return "COLLAPSED_HOUSE";
    default:                            return "UNKNOWN";
  }
}