#ifndef __VISION_UART_H__
#define __VISION_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum
{
  VISION_TARGET_SPEED_NORMAL = 0U,
  VISION_TARGET_SPEED_RELEASE,
  VISION_TARGET_TURN_LEFT,
  VISION_TARGET_TURN_RIGHT,
  VISION_TARGET_HORN,
  VISION_TARGET_PARK_1,
  VISION_TARGET_PARK_2
} Vision_Target;

typedef struct
{
  Vision_Target target;
  uint8_t id;
} Vision_Event;

void VisionUart_Init(void);
void VisionUart_RxCpltCallback(UART_HandleTypeDef *huart);
uint8_t VisionUart_ReadEvent(Vision_Event *event);
uint32_t VisionUart_GetRxCount(void);
uint8_t VisionUart_GetLastByte(void);
uint32_t VisionUart_GetErrorCount(void);
uint32_t VisionUart_GetLastError(void);
const char *VisionUart_TargetName(Vision_Target target);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_UART_H__ */
