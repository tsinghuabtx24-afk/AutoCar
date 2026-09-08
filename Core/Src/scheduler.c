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
#include "vision_nav.h"

#include <stdio.h>

static Control_Mode scheduler_mode;
/* 被抢占的模式。避障可以打断任务，避障自身不会被同级打断，急停清空一切，
   所以只需记一层，不需要模式栈。 */
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

  /* 重新获得循迹控制权时清掉内部状态：否则会带着被抢占前的原地转方向和
     丢线计数继续，而车身姿态早已被避障或手动操作改变。 */
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

/**
  * @brief  判断事件是否属于视觉类
  */
static uint8_t Scheduler_IsVisionEvent(Event_Type type)
{
  return (uint8_t)((type >= EVENT_VISION_SPEED_LIMIT) &&
                   (type <= EVENT_VISION_LAST));
}

/**
  * @brief  进入急停：放弃当前动作，清除全部任务
  */
static void Scheduler_EnterStop(void)
{
  Car_ActionAbort();
  VisionTask_Cancel();
  AvoidTask_Cancel();
  /* 放弃未走完的视觉机动：急停后车身姿态已不确定，两段式转向的第二段
     再执行只会把车带偏。 */
  VisionNav_Reset();
  Car_Stop();
  scheduler_preempted = CONTROL_LINE_TRACK;
  Scheduler_SetMode(CONTROL_STOP);

  /* 故障指示优先级最高，覆盖一切。蜂鸣只叫两声就静音——急停可能持续很久，
     一直响没有额外信息量。红灯慢闪持续提示状态。 */
  Indicator_Request(INDICATOR_PRIO_FAULT, INDICATOR_BEEPS(2),
                    INDICATOR_RED, INDICATOR_RED, INDICATOR_BLINK_SLOW_MS);
}

/**
  * @brief  切换手动接管状态
  */
static void Scheduler_ToggleManual(void)
{
  if (scheduler_mode == CONTROL_MANUAL)
  {
    /* 交还控制权，回到被抢占的模式。 */
    ManualTask_Exit();
    Car_Stop();
    Indicator_Release(INDICATOR_PRIO_MANUAL);
    Scheduler_SetMode(scheduler_preempted);
    return;
  }

  /* 急停下不接受手动接管，急停优先级更高。 */
  if (scheduler_mode == CONTROL_STOP)
  {
    printf("[SCHED] manual rejected: in STOP\r\n");
    return;
  }

  Car_ActionAbort();

  /* 手动接管优先级高于避障，进入时要把避障告警撤掉，否则灯还在闪。 */
  AvoidTask_Cancel();

  scheduler_preempted = scheduler_mode;
  ManualTask_Enter();
  Scheduler_SetMode(CONTROL_MANUAL);

  /* 手动接管用蓝灯常亮区分，不鸣笛。 */
  Indicator_Request(INDICATOR_PRIO_MANUAL, INDICATOR_BUZZER_OFF,
                    INDICATOR_BLUE, INDICATOR_BLUE, 0U);
}

void Scheduler_OnEvent(const Event *e)
{
  if (e == NULL)
  {
    return;
  }

  /* ---- 最高优先级：急停与故障，任何模式下都接受 ---- */
  if ((e->type == EVENT_KEY_STOP) || (e->type == EVENT_FAULT))
  {
    Scheduler_EnterStop();
    return;
  }

  /* ---- 手动接管开关：仅 RED 双击，任何非急停模式下都接受 ---- */
  if (e->type == EVENT_REMOTE_MANUAL_TOGGLE)
  {
    Scheduler_ToggleManual();
    return;
  }

  /* ---- 手动模式下：只有遥控事件有意义，其余一律忽略 ---- */
  if (scheduler_mode == CONTROL_MANUAL)
  {
    if ((e->type >= EVENT_REMOTE_FORWARD) && (e->type <= EVENT_REMOTE_HORN))
    {
      ManualTask_OnEvent(e->type);
    }
    return;
  }

  /* ---- 非手动模式下：其余遥控键不介入，自动驾驶不受干扰 ---- */
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
      /* 传感器重新起判，避免带着急停前的残留状态直接又触发避障。 */
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
    /* CLEAR 不主动结束避障：是否脱困由 avoid_task 的 VERIFY 相位判定，
       否则障碍刚离开视野就撤销，动作会做半截。 */
    if (e->type == EVENT_OBSTACLE_CLEAR)
    {
      return;
    }

    /* 已在避障中：不重复 Begin，避免每次状态变化都重置重试计数。 */
    if (scheduler_mode == CONTROL_IR_AVOID)
    {
      return;
    }

    /* 抢占当前模式。任务被打断后按最简策略：避障结束回到被抢占的模式，
       任务从被打断的相位继续（§6.3）。 */
    scheduler_preempted = scheduler_mode;
    Scheduler_SetMode(CONTROL_IR_AVOID);
    AvoidTask_Begin();
    return;
  }

  /* ---- 视觉事件 ---- */
  if (Scheduler_IsVisionEvent(e->type) != 0U)
  {
    /* 先过视觉导航的门控：门没开一律丢弃；左/右转信号被转成内部两段式
       机动，同样不在这里执行。返回 1 才是"该照常执行"的事件。 */
    if (VisionNav_OnVisionEvent(e->type) == 0U)
    {
      return;
    }

    /* 限速类只改巡线速度，不建任务、不切模式；任务类建任务并抢占。
       VisionTask_Begin() 返回 1 表示确实起了一个占用底盘的任务。 */
    if (VisionTask_Begin(e->type) != 0U)
    {
      Scheduler_SetMode(CONTROL_TASK);
    }
    else
    {
      /* 限速/恢复限速改的是 task_speed，把它同步到循迹的运行时基础速度，
         否则这个事件只会体现在诊断打印里，对车速没有任何影响。 */
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
  /* 信号类视觉动作（隧道白灯、起伏路段限速、友军旋律）不占底盘，
     必须不分模式每轮推进，否则循迹模式下白灯永不灭、限速永不恢复。 */
  VisionTask_Tick();

  switch (scheduler_mode)
  {
    case CONTROL_STOP:
      /* 持续制动，只有启动键能退出。 */
      Car_Stop();
      break;

    case CONTROL_MANUAL:
      ManualTask_Step();
      break;

    case CONTROL_IR_AVOID:
      if (AvoidTask_Step() == 0U)
      {
        /* 避障结束，回到被抢占的模式。 */
        Scheduler_AvoidFinished();
      }
      break;

    case CONTROL_TASK:
      if (VisionTask_Step() == 0U)
      {
        /* 任务已完成且不再占用底盘，交回循迹。 */
        Scheduler_TaskFinished();
      }
      break;

    case CONTROL_LINE_TRACK:
      /* 默认行为：差速循迹。非阻塞，内部自己把住 10ms 控制周期。 */
      LineTracker_Step();
      break;

    case CONTROL_IDLE:
    default:
      Car_Stop();
      break;
  }
}
