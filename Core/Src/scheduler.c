/**
  ******************************************************************************
  * @file    scheduler.c
  * @brief   整车控制模式调度实现
  ******************************************************************************
  */

#include "scheduler.h"
#include "car.h"
#include "vision_task.h"
#include "manual_task.h"
#include "avoid_task.h"
#include "indicator.h"
#include "ir_avoid.h"
#include "ultrasonic_sense.h"
#include "line_tracker.h"

#include <stdio.h>

static Control_Mode scheduler_mode;
static Control_Mode scheduler_preempted;

static void Scheduler_SetMode(Control_Mode mode)
{
  if (scheduler_mode == mode)
  {
    return;
  }
  printf("[SCHED] %s -> %s\r\n",
         Scheduler_ModeName(scheduler_mode), Scheduler_ModeName(mode));
  scheduler_mode = mode;

  if (mode == CONTROL_LINE_TRACK)
  {
    LineTracker_Reset();
  }
}

void Scheduler_Init(void)
{
  scheduler_mode      = SCHEDULER_START_MODE;
  scheduler_preempted = SCHEDULER_START_MODE;
  printf("[SCHED] init mode=%s\r\n", Scheduler_ModeName(scheduler_mode));
}

Control_Mode Scheduler_GetMode(void)          { return scheduler_mode; }
Control_Mode Scheduler_GetPreemptedMode(void) { return scheduler_preempted; }

const char *Scheduler_ModeName(Control_Mode mode)
{
  switch (mode)
  {
    case CONTROL_LINE_TRACK: return "LINE_TRACK";
    case CONTROL_TASK:       return "TASK";
    case CONTROL_IR_AVOID:   return "IR_AVOID";
    case CONTROL_MANUAL:     return "MANUAL";
    case CONTROL_STOP:       return "STOP";
    default:                 return "IDLE";
  }
}

static uint8_t Scheduler_IsVisionEvent(Event_Type type)
{
  return (uint8_t)((type >= EVENT_VISION_FIRST) &&
                   (type <= EVENT_VISION_LAST));
}

static void Scheduler_EnterStop(void)
{
  Car_ActionAbort();
  VisionTask_Cancel();
  AvoidTask_Cancel();
  LineTracker_DisarmForkLeft();
  Car_Stop();
  scheduler_preempted = CONTROL_LINE_TRACK;
  Scheduler_SetMode(CONTROL_STOP);

  Indicator_Request(INDICATOR_PRIO_FAULT, INDICATOR_BEEPS(2),
                    INDICATOR_RED, INDICATOR_RED, INDICATOR_BLINK_SLOW_MS);
}

static void Scheduler_ToggleManual(void)
{
  if (scheduler_mode == CONTROL_MANUAL)
  {
    ManualTask_Exit();
    Car_Stop();
    Indicator_Release(INDICATOR_PRIO_MANUAL);
    Scheduler_SetMode(scheduler_preempted);
    return;
  }

  if (scheduler_mode == CONTROL_STOP)
  {
    printf("[SCHED] manual rejected: in STOP\r\n");
    return;
  }

  Car_ActionAbort();
  AvoidTask_Cancel();

  scheduler_preempted = scheduler_mode;
  ManualTask_Enter();
  Scheduler_SetMode(CONTROL_MANUAL);

  Indicator_Request(INDICATOR_PRIO_MANUAL, INDICATOR_BUZZER_OFF,
                    INDICATOR_BLUE, INDICATOR_BLUE, 0U);
}

void Scheduler_OnEvent(const Event *e)
{
  if (e == NULL)
  {
    return;
  }

  /* ---- 最高优先级：急停与故障 ---- */
  if ((e->type == EVENT_KEY_STOP) || (e->type == EVENT_FAULT))
  {
    Scheduler_EnterStop();
    return;
  }

  /* ---- 手动接管开关：RED双击 ---- */
  if (e->type == EVENT_REMOTE_MANUAL_TOGGLE)
  {
    Scheduler_ToggleManual();
    return;
  }

  /* ---- 呼唤友军：原地鸣笛闪灯8s，全程只受理一次 ---- */
  if (e->type == EVENT_REMOTE_CALL_ALLY)
  {
    if (scheduler_mode == CONTROL_STOP)
    {
      printf("[SCHED] horn alarm rejected: in STOP\r\n");
      return;
    }

    /* 闩锁拒绝时不能改模式，否则会切进 CONTROL_TASK 却没有任务在跑，
       车停在那里等一个永远不会结束的呼唤。 */
    if (VisionTask_StartCallAlly() == 0U)
    {
      return;
    }

    /* 呼唤要占底盘（原地停住），所以从循迹/避障都抢过来。
       手动模式不抢：接管中由人控制，呼唤只叠加灯笛。 */
    if (scheduler_mode != CONTROL_MANUAL)
    {
      AvoidTask_Cancel();
      scheduler_preempted = CONTROL_LINE_TRACK;
      Scheduler_SetMode(CONTROL_TASK);
    }
    return;
  }

  /* ---- 补给掉头（遥控"8"）：原地180°后继续循迹 ---- */
  if (e->type == EVENT_REMOTE_TURN_AROUND)
  {
    if (scheduler_mode == CONTROL_STOP)
    {
      printf("[SCHED] turn around rejected: in STOP\r\n");
      return;
    }
    if (scheduler_mode == CONTROL_MANUAL)
    {
      ManualTask_OnEvent(e->type);
      return;
    }

    /* 非阻塞：切成任务模式，由 VisionTask_Step() 逐轮推进。转完
       VisionTask_Finish() 让 Scheduler_TaskFinished() 自然切回循迹。 */
    AvoidTask_Cancel();
    if (VisionTask_StartTurnAround() != 0U)
    {
      scheduler_preempted = CONTROL_LINE_TRACK;
      Scheduler_SetMode(CONTROL_TASK);
    }
    return;
  }

  /* ---- 手动模式下：只有遥控事件有意义 ---- */
  if (scheduler_mode == CONTROL_MANUAL)
  {
    if ((e->type >= EVENT_REMOTE_FORWARD) && (e->type <= EVENT_REMOTE_HORN))
    {
      ManualTask_OnEvent(e->type);
    }
    return;
  }

  /* ---- 非手动模式下：其余遥控键不介入 ---- */
  if ((e->type >= EVENT_REMOTE_FORWARD) && (e->type <= EVENT_REMOTE_HORN))
  {
    return;
  }

  /* ---- 急停模式下：只有启动键能退出 ---- */
  if (scheduler_mode == CONTROL_STOP)
  {
    if (e->type == EVENT_KEY_START)
    {
      Indicator_Release(INDICATOR_PRIO_FAULT);
      IrAvoid_Restart();
      UltrasonicSense_Restart();
      Scheduler_SetMode(CONTROL_LINE_TRACK);
    }
    return;
  }

  if (e->type == EVENT_KEY_START)
  {
    if (scheduler_mode == CONTROL_IDLE)
    {
      Scheduler_SetMode(CONTROL_LINE_TRACK);
    }
    return;
  }

  /* ---- 避障事件 ---- */
  if ((e->type >= EVENT_OBSTACLE_LEFT) && (e->type <= EVENT_OBSTACLE_CLEAR))
  {
    if (e->type == EVENT_OBSTACLE_CLEAR)
    {
      return;
    }
    if (scheduler_mode == CONTROL_IR_AVOID)
    {
      return;
    }
    scheduler_preempted = scheduler_mode;
    Scheduler_SetMode(CONTROL_IR_AVOID);
    AvoidTask_Begin();
    return;
  }

  /* ---- 视觉事件 ---- */
  if (Scheduler_IsVisionEvent(e->type) != 0U)
  {
    /* 倒塌房屋：直接进绕行，方向固定右绕。不复用避障的传感器入口——
       视觉识别时红外/超声通常还没报警，那个入口会判成"无障碍"直接放弃。 */
    if (e->type == EVENT_VISION_COLLAPSED_HOUSE)
    {
      if (scheduler_mode == CONTROL_IR_AVOID)
      {
        return;    /* 已经在绕了，别打断重开 */
      }
      scheduler_preempted = CONTROL_LINE_TRACK;
      Scheduler_SetMode(CONTROL_IR_AVOID);
      AvoidTask_BeginDetour(1);
      return;
    }

    if (VisionTask_Begin(e->type) != 0U)
    {
      Scheduler_SetMode(CONTROL_TASK);
    }
    else
    {
      LineTracker_SetBaseSpeed(VisionTask_GetSpeed());
    }
  }
}

void Scheduler_DrainEvents(void)
{
  Event e;

  while (Event_Pop(&e) != 0U)
  {
    Scheduler_OnEvent(&e);
  }
}

void Scheduler_TaskFinished(void)
{
  if (scheduler_mode == CONTROL_TASK)
  {
    Scheduler_SetMode(CONTROL_LINE_TRACK);
  }
}

void Scheduler_AvoidFinished(void)
{
  if (scheduler_mode == CONTROL_IR_AVOID)
  {
    Scheduler_SetMode(scheduler_preempted);
  }
}

void Scheduler_Dispatch(void)
{
  VisionTask_Tick();

  switch (scheduler_mode)
  {
    case CONTROL_STOP:
      Car_Stop();
      break;

    case CONTROL_MANUAL:
      ManualTask_Step();
      break;

    case CONTROL_IR_AVOID:
      if (AvoidTask_Step() == 0U)
      {
        Scheduler_AvoidFinished();
      }
      break;

    case CONTROL_TASK:
      if (VisionTask_Step() == 0U)
      {
        Scheduler_TaskFinished();
      }
      break;

    case CONTROL_LINE_TRACK:
      LineTracker_Step();
      break;

    case CONTROL_IDLE:
    default:
      Car_Stop();
      break;
  }
}