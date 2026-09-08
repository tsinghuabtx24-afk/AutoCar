#ifndef __VISION_UART_H__
#define __VISION_UART_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "event.h"

/* ==== 帧同步超时重整 ======================================================== */
#define VISION_FRAME_TIMEOUT_MS   20U

/* ==== 同图案静默窗 ==========================================================
   识别到某个图案后，该图案在这段时间内不再投事件。

   为什么需要：上位机对同一个牌子会连续出好几帧（车还在牌子前面，识别一直
   成立），不去重的话一块牌子会触发多次动作——禁行会连着倒两次车，仓库刚
   转完又被重新触发。

   **按类别独立计窗**，不是全局一个窗：隧道紧跟着仓库出现时，仓库不该被隧道
   的窗口吃掉。8 个类别各有各的 last tick。

   窗口只在这里做，不放到 scheduler：投都不投，队列就不会被同一图案的重复帧
   顶满（EVENT_QUEUE_SIZE 只有 16）。
   ============================================================================ */
#define VISION_REPEAT_SILENCE_MS  3000U

/* ==== 识别类别与线上id ======================================================
   7个类别，顺序与上位机模型的类别号一致：
     0 禁止通行  1 前方慢行  2 友军信号  3 隧道
     4 狭窄街道  5 仓库      6 倒塌房屋

   线上id = VISION_ID_FIRST + 类别序号。基数取 0，即 id 就是类别号本身。
   ⚠️ 枚举顺序**必须**与上表一致：IdToTarget() 靠 id 减基数直接得到枚举值，
      顺序错了就会把牌子认成别的动作。
   ============================================================================ */
#define VISION_ID_FIRST   0U

typedef enum
{
  VISION_TARGET_NO_ENTRY = 0U,      /* 0 禁止通行：后退→右转→前进 */
  VISION_TARGET_SLOW_AHEAD,         /* 1 前方慢行：限速 */
  VISION_TARGET_FRIENDLY,           /* 2 友军信号：停车鸣笛×4 */
  VISION_TARGET_TUNNEL,             /* 3 隧道：白灯常亮 */
  VISION_TARGET_NARROW_STREET,      /* 4 狭窄街道：黄灯闪烁 */
  VISION_TARGET_WAREHOUSE,          /* 5 仓库：进库机动 */
  VISION_TARGET_COLLAPSED_HOUSE,    /* 6 倒塌房屋：走避障 */
  VISION_TARGET_COUNT
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
uint32_t VisionUart_GetResyncCount(void);

/* 因落在同图案静默窗内而被丢弃的帧数，排查"牌子明明识别到了却没动作"时看它。 */
uint32_t VisionUart_GetSilencedCount(void);

const char *VisionUart_TargetName(Vision_Target target);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_UART_H__ */