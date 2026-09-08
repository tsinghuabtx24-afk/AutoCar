#ifndef __VISION_UART_H__
#define __VISION_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "event.h"

/* ==== 帧同步超时重整 ========================================================
   3 字节帧 0xAA / id / (0xAA^id)。原实现只按字节顺序推进 frame_step，丢字节后
   若 id 或校验位恰好是 0xAA，要等本帧走完才可能重新对齐，中间的帧全部作废。

   视觉模块连续发送时字节间隔是微秒级，帧间隔远大于此。所以：**距上一个字节
   超过这么久，就认为新的一帧开始了，无条件从 step 0 重新对齐**。
   取 20ms：远大于 115200 下的字节间隔（约 87us），也远小于视觉模块的帧周期。
   ============================================================================ */
#define VISION_FRAME_TIMEOUT_MS   20U

/* ==== 识别类别与线上 id ======================================================
   六个类别，按约定顺序：
     1 仓库  2 禁止通行  3 隧道  4 起伏路段  5 友军  6 可清理障碍

   线上 id = VISION_ID_FIRST + 类别序号。视觉端如果改成 0 基编号，
   只改这一个宏即可，不用动映射表。
   ============================================================================ */
#define VISION_ID_FIRST   1U

typedef enum
{
  VISION_TARGET_WAREHOUSE = 0U,      /* 仓库 */
  VISION_TARGET_NO_ENTRY,            /* 禁止通行 */
  VISION_TARGET_TUNNEL,              /* 隧道 */
  VISION_TARGET_ROUGH_ROAD,          /* 起伏路段 */
  VISION_TARGET_FRIENDLY,            /* 友军 */
  VISION_TARGET_CLEARABLE_OBSTACLE,  /* 可清理障碍 */
  VISION_TARGET_COUNT
} Vision_Target;

/* ==== 三个"动作暂不修改"的类别沿用哪个既有动作 ==============================
   仓库 / 禁止通行 / 可清理障碍的动作代码本身未改动，只是换了触发它的类别。
   要调整对应关系，改这三行就够。
   ============================================================================ */
#define VISION_EVENT_WAREHOUSE            EVENT_VISION_PARK_1   /* 闪灯后停住 */
#define VISION_EVENT_NO_ENTRY             EVENT_VISION_TURN_LEFT /* 原地转 90° */
#define VISION_EVENT_CLEARABLE_OBSTACLE   EVENT_VISION_HORN     /* 停车鸣两声 */

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
