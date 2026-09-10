#ifndef __VISION_UART_H__
#define __VISION_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==== 帧同步超时重整 ========================================================
   3 字节帧 0xAA / id / (0xAA^id)。原实现只按字节顺序推进 frame_step，丢字节后
   若 id 或校验位恰好是 0xAA，要等本帧走完才可能重新对齐，中间的帧全部作废。

   视觉模块连续发送时字节间隔是微秒级，帧间隔远大于此。所以：**距上一个字节
   超过这么久，就认为新的一帧开始了，无条件从 step 0 重新对齐**。
   取 20ms：远大于 115200 下的字节间隔（约 87us），也远小于视觉模块的帧周期。
   ============================================================================ */
#define VISION_FRAME_TIMEOUT_MS   20U

typedef enum
{
  VISION_TARGET_SPEED_NORMAL = 0U,
  VISION_TARGET_SPEED_RELEASE,
  VISION_TARGET_TURN_LEFT,
  VISION_TARGET_TURN_RIGHT,
  VISION_TARGET_HORN,
  VISION_TARGET_PARK_1,
  VISION_TARGET_PARK_2,

  /* 第 8 类（线上 id 7）。动作：停车鸣两声。单独立一个类别而不是复用 id 4 */
  VISION_TARGET_HORN_2
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

/* 因帧间静默而重新对齐的次数。持续增长说明视觉模块在丢字节或波特率有偏差。 */
uint32_t VisionUart_GetResyncCount(void);

const char *VisionUart_TargetName(Vision_Target target);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_UART_H__ */
