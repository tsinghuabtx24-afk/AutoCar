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

/* ==== 黑线直行期的事件暂存 ==================================================
   黑区暂停后的强制直行只有 LINE_BLACK_CREEP_MS，期间不希望被抢断，否则那段
   "挪出黑区"没走完就切走，回来时车还压在黑上。

   但也**不能丢事件**：视觉牌子只报那几帧，丢了就等于没识别到。所以占底盘的
   事件在直行期内先存进这个槽，直行一结束立刻补上。

   只存一个：同一次直行只有 100ms，正常最多来一个视觉事件。已有暂存时后来的
   丢弃而不是覆盖——保留**先识别到的那个**，符合"不要把原来识别到的视觉挤掉"。
   ============================================================================ */
static Event_Type scheduler_pending_type;
static uint8_t    scheduler_pending_param;
static uint8_t    scheduler_has_pending;

static void Scheduler_SetMode(Control_Mode mode)
{
  if (scheduler_mode == mode)
  {
    return;
  }
  printf("[SCHED] %s -> %s\r\n",
         Scheduler_ModeName(scheduler_mode), Scheduler_ModeName(mode));

  /* ⚠️ 这里**不能** Car_ActionAbort()。SetMode 常在接管方已经发起动作之后
     才被调用（补给掉头就是先 VisionTask_StartTurnAround() 再切模式），在这
     abort 会把刚发起的动作掐掉，车只扭一下就停。
     循迹持有的定角动作由 Scheduler_ReleaseLineTrackAction() 在切模式**之前**
     显式清理。 */
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
  scheduler_has_pending = 0U;
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

/**
  * @brief  交出循迹可能持有的 car 动作
  *
  * @note   循迹平时用瞬时接口（差速、pivot），只有岔道规则的定角 90° 旋转是
  *         car 的"动作"。Car_ActionBegin() 在已有动作未结束时是**拒绝**而不是
  *         覆盖，所以接管方发起自己的定角动作之前必须先清掉它，否则仓库的
  *         左转、禁行的右转会当场被拒。
  *
  *         必须在接管方发起动作**之前**调用，不能放到 Scheduler_SetMode() 里
  *         ——那里往往已经晚了。
  */
static void Scheduler_ReleaseLineTrackAction(void)
{
  if (scheduler_mode == CONTROL_LINE_TRACK)
  {
    Car_ActionAbort();
  }
}

static void Scheduler_EnterStop(void)
{
  Car_ActionAbort();
  VisionTask_Cancel();
  AvoidTask_Cancel();
  LineTracker_DisarmForkLeft();
  Car_Stop();
  scheduler_has_pending = 0U;   /* 急停后暂存的动作作废 */
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
    scheduler_has_pending = 0U;   /* 人接管了，暂存的自动动作作废 */
    Scheduler_ToggleManual();
    return;
  }

  /* ---- 黑区直行期：占底盘的事件先暂存，直行走完再补 ----
     排在急停/故障/手动接管**之后**：那三个任何时候都不能延迟。
     只拦视觉和避障，遥控与按键不拦。 */
  if ((scheduler_mode == CONTROL_LINE_TRACK) &&
      (LineTracker_IsBlackCreeping() != 0U) &&
      ((Scheduler_IsVisionEvent(e->type) != 0U) ||
       ((e->type >= EVENT_OBSTACLE_LEFT) && (e->type <= EVENT_OBSTACLE_CLEAR))))
  {
    if (e->type == EVENT_OBSTACLE_CLEAR)
    {
      return;   /* 本来就不触发动作，不用暂存 */
    }

    if (scheduler_has_pending == 0U)
    {
      scheduler_pending_type  = e->type;
      scheduler_pending_param = e->param;
      scheduler_has_pending   = 1U;
      printf("[SCHED] %s deferred: black creep in progress\r\n",
             Event_TypeName(e->type));
    }
    else
    {
      /* 已有暂存：保留先到的那个。 */
      printf("[SCHED] %s dropped: pending %s already held\r\n",
             Event_TypeName(e->type), Event_TypeName(scheduler_pending_type));
    }
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
    Scheduler_ReleaseLineTrackAction();
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
      /* 占底盘的任务：先交出循迹可能持有的定角动作，否则任务第一个
         Car_StartRotateAngle 会被拒（仓库左转、禁行右转）。
         Begin() 本身不发起 car 动作，动作在首次 Step() 里才发起，所以
         放在 Begin 之后、SetMode 之前是安全的。 */
      Scheduler_ReleaseLineTrackAction();
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

      /* 黑区直行走完了，把期间暂存的事件补上。
         放在 Step() **之后**：直行的结束判定在 Step 里做，先查会白等一轮。 */
      if ((scheduler_has_pending != 0U) &&
          (LineTracker_IsBlackCreeping() == 0U))
      {
        Event pending;

        pending.type  = scheduler_pending_type;
        pending.param = scheduler_pending_param;
        pending.tick  = HAL_GetTick();

        /* 先清标志再派发：OnEvent 可能切模式并再次走到暂存判断，
           不清的话会把刚取出的事件又存回去。 */
        scheduler_has_pending = 0U;

        printf("[SCHED] replay deferred %s\r\n", Event_TypeName(pending.type));
        Scheduler_OnEvent(&pending);
      }
      break;

    case CONTROL_IDLE:
    default:
      Car_Stop();
      break;
  }
}