/**
  ******************************************************************************
  * @file    event.c
  * @brief   全车事件队列实现
  ******************************************************************************
  */

#include "event.h"

/* 环形队列。head 出队、tail 入队，空出一格区分空与满，省掉计数变量。 */
static volatile Event    event_queue[EVENT_QUEUE_SIZE];
static volatile uint8_t  event_head;
static volatile uint8_t  event_tail;
static volatile uint32_t event_dropped;
static volatile uint32_t event_posted;

void Event_Init(void)
{
  event_head    = 0U;
  event_tail    = 0U;
  event_dropped = 0U;
  event_posted  = 0U;
}

/**
  * @brief  事件入队
  * @note   可从中断调用。用 PRIMASK 保存/恢复而不是无条件 __enable_irq()，
  *         否则在已关中断的临界区里调用会把中断提前放开。
  */
uint8_t Event_Post(Event_Type type, uint8_t param)
{
  uint32_t primask = __get_PRIMASK();
  uint8_t  next;
  uint8_t  ok;

  __disable_irq();

  next = (uint8_t)((event_tail + 1U) % EVENT_QUEUE_SIZE);
  if (next == event_head)
  {
    /* 队列满。丢弃最新事件并计数，绝不在中断里 printf。 */
    event_dropped++;
    ok = 0U;
  }
  else
  {
    event_queue[event_tail].type  = type;
    event_queue[event_tail].param = param;
    event_queue[event_tail].tick  = HAL_GetTick();
    event_tail = next;
    event_posted++;
    ok = 1U;
  }

  __set_PRIMASK(primask);
  return ok;
}

uint8_t Event_Pop(Event *out)
{
  uint32_t primask = __get_PRIMASK();
  uint8_t  ok;

  if (out == NULL)
  {
    return 0U;
  }

  __disable_irq();

  if (event_head == event_tail)
  {
    ok = 0U;
  }
  else
  {
    out->type  = event_queue[event_head].type;
    out->param = event_queue[event_head].param;
    out->tick  = event_queue[event_head].tick;
    event_head = (uint8_t)((event_head + 1U) % EVENT_QUEUE_SIZE);
    ok = 1U;
  }

  __set_PRIMASK(primask);
  return ok;
}

uint8_t Event_IsEmpty(void)
{
  return (event_head == event_tail) ? 1U : 0U;
}

uint32_t Event_GetDroppedCount(void) { return event_dropped; }
uint32_t Event_GetPostedCount(void)  { return event_posted; }

const char *Event_TypeName(Event_Type type)
{
  switch (type)
  {
    case EVENT_VISION_SPEED_LIMIT:    return "VISION_SPEED_LIMIT";
    case EVENT_VISION_SPEED_RELEASE:  return "VISION_SPEED_RELEASE";
    case EVENT_VISION_TURN_LEFT:      return "VISION_TURN_LEFT";
    case EVENT_VISION_TURN_RIGHT:     return "VISION_TURN_RIGHT";
    case EVENT_VISION_HORN:           return "VISION_HORN";
    case EVENT_VISION_PARK_1:         return "VISION_PARK_1";
    case EVENT_VISION_PARK_2:         return "VISION_PARK_2";
    case EVENT_OBSTACLE_LEFT:         return "OBSTACLE_LEFT";
    case EVENT_OBSTACLE_RIGHT:        return "OBSTACLE_RIGHT";
    case EVENT_OBSTACLE_BOTH:         return "OBSTACLE_BOTH";
    case EVENT_OBSTACLE_FRONT:        return "OBSTACLE_FRONT";
    case EVENT_OBSTACLE_CLEAR:        return "OBSTACLE_CLEAR";
    case EVENT_KEY_START:             return "KEY_START";
    case EVENT_KEY_STOP:              return "KEY_STOP";
    case EVENT_REMOTE_MANUAL_TOGGLE:  return "REMOTE_MANUAL_TOGGLE";
    case EVENT_REMOTE_FORWARD:        return "REMOTE_FORWARD";
    case EVENT_REMOTE_BACKWARD:       return "REMOTE_BACKWARD";
    case EVENT_REMOTE_ROTATE_LEFT:    return "REMOTE_ROTATE_LEFT";
    case EVENT_REMOTE_ROTATE_RIGHT:   return "REMOTE_ROTATE_RIGHT";
    case EVENT_REMOTE_HORN:           return "REMOTE_HORN";
    case EVENT_FAULT:                 return "FAULT";
    default:                          return "NONE";
  }
}
