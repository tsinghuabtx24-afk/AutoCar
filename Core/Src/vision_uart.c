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
static volatile uint32_t vision_last_byte_tick;   /* 上一个字节到达的 tick */
static volatile uint32_t vision_resync_count;

/**
  * @brief  视觉目标 → 全车事件
  */
static Event_Type VisionUart_TargetToEvent(Vision_Target target)
{
  switch (target)
  {
    /* 动作未改动的三类，只是换了触发者，映射写在 vision_uart.h。 */
    case VISION_TARGET_WAREHOUSE:           return VISION_EVENT_WAREHOUSE;
    case VISION_TARGET_NO_ENTRY:            return VISION_EVENT_NO_ENTRY;
    case VISION_TARGET_CLEARABLE_OBSTACLE:  return VISION_EVENT_CLEARABLE_OBSTACLE;

    /* 新增动作的三类。 */
    case VISION_TARGET_TUNNEL:              return EVENT_VISION_TUNNEL;
    case VISION_TARGET_ROUGH_ROAD:          return EVENT_VISION_ROUGH_ROAD;
    case VISION_TARGET_FRIENDLY:            return EVENT_VISION_FRIENDLY;

    default:                                return EVENT_NONE;
  }
}

/**
  * @brief  线上 id → 类别
  * @retval 0xFF 表示 id 不在约定范围内，调用方按校验失败处理
  */
static Vision_Target VisionUart_IdToTarget(uint8_t id)
{
  if ((id < VISION_ID_FIRST) ||
      (id >= (uint8_t)(VISION_ID_FIRST + (uint8_t)VISION_TARGET_COUNT)))
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

  /* ---- 帧间静默检测：超时就无条件重新对齐 ----
     一帧的三个字节是连着发的（115200 下字节间隔约 87us）。距上一个字节
     超过 VISION_FRAME_TIMEOUT_MS 说明上一帧已经结束或残缺，当前字节必须
     按"新帧的第一个字节"来解释，否则残帧会把后面的帧一路错位下去。 */
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
        /* 单事件槽保留给旧的诊断读法；正式路径投递到全车事件队列，
           这样任务执行期间到达的第二个事件不会被覆盖。 */
        vision_event.target = target;
        vision_event.id = vision_frame_id;
        vision_event_ready = 1U;
        (void)Event_Post(VisionUart_TargetToEvent(target), vision_frame_id);
        vision_frame_step = 0U;
      }
      else
      {
        /* 中断里不能 printf：阻塞发送会拖垮时基，这里只累计错误计数。 */
        vision_error_count++;

        /* 校验失败时如果这个字节本身是 0xAA，很可能前面丢了字节、而它是
           下一帧的帧头。按帧头解释而不是丢弃，这样只损失一帧就能重新对齐，
           不必等帧间静默超时。 */
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

uint32_t VisionUart_GetLastError(void) { return vision_last_error; }

const char *VisionUart_TargetName(Vision_Target target)
{
  switch (target)
  {
    case VISION_TARGET_WAREHOUSE:          return "WAREHOUSE";
    case VISION_TARGET_NO_ENTRY:           return "NO_ENTRY";
    case VISION_TARGET_TUNNEL:             return "TUNNEL";
    case VISION_TARGET_ROUGH_ROAD:         return "ROUGH_ROAD";
    case VISION_TARGET_FRIENDLY:           return "FRIENDLY";
    case VISION_TARGET_CLEARABLE_OBSTACLE: return "CLEARABLE_OBSTACLE";
    default: return "UNKNOWN";
  }
}

