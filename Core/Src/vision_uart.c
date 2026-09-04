#include "vision_uart.h"
#include "usart.h"

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

static Vision_Target VisionUart_IdToTarget(uint8_t id)
{
  switch (id)
  {
    case 0U: return VISION_TARGET_SPEED_NORMAL;
    case 1U: return VISION_TARGET_SPEED_RELEASE;
    case 2U: return VISION_TARGET_TURN_LEFT;
    case 3U: return VISION_TARGET_TURN_RIGHT;
    case 4U: return VISION_TARGET_HORN;
    case 5U: return VISION_TARGET_PARK_1;
    case 6U: return VISION_TARGET_PARK_2;
    default: return (Vision_Target)0xFFU;
  }
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
  if (HAL_UART_Receive_IT(&huart2, (uint8_t *)&vision_rx_byte, 1U) != HAL_OK)
  {
    printf("[VISION] UART2 receive start failed\r\n");
  }
}

void VisionUart_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uint8_t checksum;
  Vision_Target target;

  if (huart != &huart2) return;

  vision_rx_count++;
  vision_last_byte = vision_rx_byte;

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
        vision_event.target = target;
        vision_event.id = vision_frame_id;
        vision_event_ready = 1U;
      }
      else
      {
        /* 中断里不能 printf：阻塞发送会拖垮时基，这里只累计错误计数。 */
    vision_error_count++;
  }
      vision_frame_step = 0U;
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

uint32_t VisionUart_GetLastError(void) { return vision_last_error; }

const char *VisionUart_TargetName(Vision_Target target)
{
  switch (target)
  {
    case VISION_TARGET_SPEED_NORMAL: return "NORMAL_SPEED";
    case VISION_TARGET_SPEED_RELEASE: return "RELEASE_SPEED";
    case VISION_TARGET_TURN_LEFT: return "TURN_LEFT";
    case VISION_TARGET_TURN_RIGHT: return "TURN_RIGHT";
    case VISION_TARGET_HORN: return "HORN";
    case VISION_TARGET_PARK_1: return "PARK_1";
    case VISION_TARGET_PARK_2: return "PARK_2";
    default: return "UNKNOWN";
  }
}

