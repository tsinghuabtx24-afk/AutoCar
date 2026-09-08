/**
  ******************************************************************************
  * @file    event.h
  * @brief   全车事件队列
  ******************************************************************************
  * @note    识别模块只产生事件，不直接驱动底盘。调度器消费事件决定控制模式。
  *
  *          Event_Post() 可从中断调用：入队只做指针搬移，不 printf。队列满时
  *          丢弃最新事件并累加 Event_GetDroppedCount()，供诊断查看。
  *
  *          单事件槽（原 vision_uart 的做法）在任务执行期间会丢事件，这里改成
  *          环形队列。
  ******************************************************************************
  */
#ifndef __EVENT_H__
#define __EVENT_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 队列深度。取 16 足够容纳一次任务执行期间可能到达的事件。 */
#define EVENT_QUEUE_SIZE   16U

typedef enum
{
  EVENT_NONE = 0,

  /* 视觉识别（7个类别）。
     这一段必须连续：scheduler 用 [EVENT_VISION_FIRST, EVENT_VISION_LAST]
     的区间判断"是不是视觉事件"。 */
  EVENT_VISION_FIRST,

  /* 顺序与 Vision_Target 一致（上位机类别号 0~6），便于对照。 */
  EVENT_VISION_NO_ENTRY = EVENT_VISION_FIRST,   /* 0 禁止通行：后退→右转→前进 */
  EVENT_VISION_SLOW_AHEAD,                      /* 1 前方慢行：限速 */
  EVENT_VISION_FRIENDLY,                        /* 2 友军信号：停车鸣笛×4 */
  EVENT_VISION_TUNNEL,                          /* 3 隧道：白灯常亮 */
  EVENT_VISION_NARROW_STREET,                   /* 4 狭窄街道：黄灯闪烁 */
  EVENT_VISION_WAREHOUSE,                       /* 5 仓库：进库机动 */
  EVENT_VISION_COLLAPSED_HOUSE,                 /* 6 倒塌房屋：走避障逻辑 */

  EVENT_VISION_LAST = EVENT_VISION_COLLAPSED_HOUSE,

  /* 避障传感器。红外分左右，超声波只报前方。 */
  EVENT_OBSTACLE_LEFT,
  EVENT_OBSTACLE_RIGHT,
  EVENT_OBSTACLE_BOTH,
  EVENT_OBSTACLE_FRONT,
  EVENT_OBSTACLE_CLEAR,

  /* 按键 */
  EVENT_KEY_START,
  EVENT_KEY_STOP,

  /* 红外遥控。MANUAL_TOGGLE 由 RED 键双击产生，其余键仅在手动模式下有意义。 */
  EVENT_REMOTE_MANUAL_TOGGLE,
  EVENT_REMOTE_FORWARD,
  EVENT_REMOTE_BACKWARD,
  EVENT_REMOTE_ROTATE_LEFT,
  EVENT_REMOTE_ROTATE_RIGHT,
  EVENT_REMOTE_HORN,

  /* 新增遥控命令 */
  EVENT_REMOTE_CALL_ALLY,         /* 按键"9"（0x1A）：呼唤友军：原地鸣笛闪灯8s */
  EVENT_REMOTE_TURN_AROUND,        /* 按键"8"（0x19）：原地旋转180° */

  /* 故障：动作连续失败、传感器异常等 */
  EVENT_FAULT
} Event_Type;

typedef struct
{
  Event_Type type;
  uint8_t    param;   /* 事件附带参数，含义由生产者定义 */
  uint32_t   tick;    /* 入队时刻，用于判断事件是否过期 */
} Event;

void Event_Init(void);

/* 入队。中断安全。返回 1=成功，0=队列满已丢弃。 */
uint8_t Event_Post(Event_Type type, uint8_t param);

/* 出队。返回 1=取到事件，0=队列空。 */
uint8_t Event_Pop(Event *out);

uint8_t  Event_IsEmpty(void);
uint32_t Event_GetDroppedCount(void);
uint32_t Event_GetPostedCount(void);

const char *Event_TypeName(Event_Type type);

#ifdef __cplusplus
}
#endif

#endif /* __EVENT_H__ */