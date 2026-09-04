/**
  ******************************************************************************
  * @file    manual_task.c
  * @brief   红外遥控手动接管实现
  ******************************************************************************
  */

#include "manual_task.h"
#include "ir_remote.h"
#include "ir_remote_map.h"
#include "car.h"
#include "indicator.h"

#include <stdio.h>

/* RED 键的 NEC command 码，见 ir_remote_map.c 的 IrRemote_KeyName()。 */
#define MANUAL_KEY_RED   0x00U

static uint32_t   manual_last_red_tick;
static uint8_t    manual_red_armed;      /* 已收到第一次 RED，等第二次 */
static Event_Type manual_action;         /* 当前生效的遥控动作 */
static uint32_t   manual_last_key_tick;

void ManualTask_Init(void)
{
  manual_last_red_tick = 0U;
  manual_red_armed     = 0U;
  manual_action        = EVENT_NONE;
  manual_last_key_tick = 0U;
}

/**
  * @brief  把 NEC command 映射成事件
  * @note   沿用 ir_remote_map 已有的键位定义，不在这里重复一张表。
  */
static Event_Type ManualTask_CommandToEvent(uint8_t command)
{
  switch (IrRemote_GetAction(command))
  {
    case IR_REMOTE_ACTION_FORWARD:      return EVENT_REMOTE_FORWARD;
    case IR_REMOTE_ACTION_ROTATE_LEFT:  return EVENT_REMOTE_ROTATE_LEFT;
    case IR_REMOTE_ACTION_ROTATE_RIGHT: return EVENT_REMOTE_ROTATE_RIGHT;
    case IR_REMOTE_ACTION_HORN:         return EVENT_REMOTE_HORN;
    default:                            break;
  }

  /* BACK 键 ir_remote_map 尚未给 action，这里单独认一下，手动模式要能后退。 */
  if (command == 0x08U)
  {
    return EVENT_REMOTE_BACKWARD;
  }
  return EVENT_NONE;
}

/**
  * @brief  处理 RED 键，判定是否构成双击
  */
static void ManualTask_HandleRed(uint32_t now)
{
  if ((manual_red_armed != 0U) &&
      ((uint32_t)(now - manual_last_red_tick) <= MANUAL_DOUBLE_MS))
  {
    /* 双击成立，投递切换事件并解除待判状态。 */
    manual_red_armed = 0U;
    (void)Event_Post(EVENT_REMOTE_MANUAL_TOGGLE, 0U);
    printf("[MANUAL] RED double-click\r\n");
    return;
  }

  /* 第一次 RED，或距上次太久重新起算。 */
  manual_red_armed     = 1U;
  manual_last_red_tick = now;
}

void ManualTask_Sense(void)
{
  IrRemote_Frame frame;
  uint32_t       now = HAL_GetTick();

  /* repeat 帧只用于刷新"按键仍按住"，不参与双击计数。 */
  if (IrRemote_IsRepeat() != 0U)
  {
    manual_last_key_tick = now;
    return;
  }

  if (IrRemote_Read(&frame) == 0U)
  {
    /* 双击窗口过期就解除待判，避免两次相隔很久的 RED 被凑成一次双击。 */
    if ((manual_red_armed != 0U) &&
        ((uint32_t)(now - manual_last_red_tick) > MANUAL_DOUBLE_MS))
    {
      manual_red_armed = 0U;
    }
    return;
  }

  manual_last_key_tick = now;

  if (frame.command == MANUAL_KEY_RED)
  {
    ManualTask_HandleRed(now);
    return;
  }

  /* 其他键投递为动作事件。是否采纳由调度器按当前模式决定：非手动模式下
     这些事件会被丢弃，遥控不介入自动驾驶。 */
  {
    Event_Type type = ManualTask_CommandToEvent(frame.command);

    if (type != EVENT_NONE)
    {
      (void)Event_Post(type, frame.command);
    }
  }
}

void ManualTask_Enter(void)
{
  manual_action        = EVENT_NONE;
  manual_last_key_tick = HAL_GetTick();
  printf("[MANUAL] take over\r\n");
}

void ManualTask_Exit(void)
{
  manual_action = EVENT_NONE;
  printf("[MANUAL] release\r\n");
}

void ManualTask_OnEvent(Event_Type type)
{
  manual_action        = type;
  manual_last_key_tick = HAL_GetTick();
}

void ManualTask_Step(void)
{
  uint32_t now = HAL_GetTick();

  /* 按键静默：停车但不退出手动模式。 */
  if ((uint32_t)(now - manual_last_key_tick) >= MANUAL_KEY_TIMEOUT_MS)
  {
    manual_action = EVENT_NONE;
    Car_Stop();
    return;
  }

  switch (manual_action)
  {
    case EVENT_REMOTE_FORWARD:
      Car_ForwardRun(MANUAL_SPEED);
      break;

    case EVENT_REMOTE_BACKWARD:
      Car_DriveSpeed(-(int16_t)MANUAL_SPEED, -(int16_t)MANUAL_SPEED);
      break;

    case EVENT_REMOTE_ROTATE_LEFT:
      Car_DriveSpeed(-(int16_t)MANUAL_ROTATE_SPEED, (int16_t)MANUAL_ROTATE_SPEED);
      break;

    case EVENT_REMOTE_ROTATE_RIGHT:
      Car_DriveSpeed((int16_t)MANUAL_ROTATE_SPEED, -(int16_t)MANUAL_ROTATE_SPEED);
      break;

    case EVENT_REMOTE_HORN:
      /* 鸣笛不占底盘，只是停车鸣笛。 */
      Car_Stop();
      break;

    default:
      Car_Stop();
      break;
  }

  /* 手动模式的指示：蓝灯常亮表示接管中，按 HORN 时加鸣笛。
     调度器在进入手动模式时已申请蓝灯，这里只更新蜂鸣位。 */
  Indicator_Request(INDICATOR_PRIO_MANUAL,
                    (uint8_t)(manual_action == EVENT_REMOTE_HORN),
                    INDICATOR_BLUE, INDICATOR_BLUE, 0U);
}
