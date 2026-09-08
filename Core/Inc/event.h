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

  /* 视觉识别。
     这一段必须连续：scheduler 用 [EVENT_VISION_SPEED_LIMIT, EVENT_VISION_LAST]
     的区间判断"是不是视觉事件"，新增视觉事件一律加在 EVENT_VISION_LAST 之前。 */
  EVENT_VISION_SPEED_LIMIT,
  EVENT_VISION_SPEED_RELEASE,
  EVENT_VISION_TURN_LEFT,
  EVENT_VISION_TURN_RIGHT,
  EVENT_VISION_HORN,
  EVENT_VISION_PARK_1,
  EVENT_VISION_PARK_2,

  /* 隧道：常亮白灯 5s。不占底盘，循迹继续。 */
  EVENT_VISION_TUNNEL,
  /* 起伏路段：限速 5s 后自动恢复原速。不占底盘。 */
  EVENT_VISION_ROUGH_ROAD,
  /* 友军：RGB 短闪一次 + 一段有音调的旋律。不占底盘。 */
  EVENT_VISION_FRIENDLY,

  EVENT_VISION_LAST = EVENT_VISION_FRIENDLY,

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
