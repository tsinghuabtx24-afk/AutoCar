#include "vision_task.h"
#include "car.h"
#include "indicator.h"
#include "line_tracker.h"

#include <stdio.h>

/* 限速后的速度必须仍高于死区，否则限速等于停车。 */
_Static_assert(VISION_SPEED_NORMAL > VISION_SPEED_LIMIT_DROP,
               "VISION_SPEED_LIMIT_DROP exceeds VISION_SPEED_NORMAL");
_Static_assert((VISION_SPEED_NORMAL - VISION_SPEED_LIMIT_DROP) >
               (CAR_SPEED_BASE + 2U),
               "limited speed falls into the motor dead zone");

/* 解除限速要恢复到循迹的默认速度，两个宏必须一致，否则一次限速+解除之后
   车速会悄悄变成另一个值。 */
_Static_assert(VISION_SPEED_NORMAL == LINE_BASE_SPEED,
               "VISION_SPEED_NORMAL must match LINE_BASE_SPEED");

static VisionTask_State task_state;
static uint32_t task_start_tick;
static uint32_t task_phase_tick;
static uint8_t task_phase;
static uint8_t task_speed;

/* ---- 鸣笛：独立于 task_state 的一条小状态机 ----
   鸣笛不占底盘，和任何任务（乃至循迹、避障）并行，所以不能占用 task_state。 */
static uint8_t  horn_active;
static uint8_t  horn_phase;      /* 偶数相位=响，奇数=停 */
static uint32_t horn_phase_tick;

/* 当前的灯与蜂鸣申请状态。通过指示层申请而不是直写 GPIO，这样入库闪灯
   不会再被红外的"无障碍则灭灯"覆盖。 */
static uint8_t task_alert_on;
static uint8_t task_buzzer_on;

static void VisionTask_Apply(void)
{
  if ((task_alert_on == 0U) && (task_buzzer_on == 0U))
  {
    Indicator_Release(INDICATOR_PRIO_TASK);
    return;
  }

  /* 闪灯相位由任务自己按 task_phase 控制，所以这里 blink_ms 传 0（常亮），
     由 task_alert_on 的开关体现闪烁。 */
  /* 鸣笛任务自己控制"响几声"的相位，所以这里用持续鸣叫，
     由 task_buzzer_on 的开关体现节奏。 */
  Indicator_Request(INDICATOR_PRIO_TASK,
                    (task_buzzer_on != 0U) ? INDICATOR_BUZZER_ON
                                           : INDICATOR_BUZZER_OFF,
                    (task_alert_on != 0U) ? INDICATOR_YELLOW : INDICATOR_OFF,
                    (task_alert_on != 0U) ? INDICATOR_YELLOW : INDICATOR_OFF,
                    0U);
}

static void VisionTask_Alert(uint8_t on)
{
  task_alert_on = on;
  VisionTask_Apply();
}

static void VisionTask_Buzzer(uint8_t on)
{
  task_buzzer_on = on;
  VisionTask_Apply();
}

/* 内部启动一个任务。公开的 VisionTask_Begin(Event_Type) 是调度器入口，
   两者名字容易混，这个改叫 Start。 */
static void VisionTask_Start(VisionTask_State state)
{
  task_state = state;
  task_start_tick = HAL_GetTick();
  task_phase_tick = task_start_tick;
  task_phase = 0U;
  printf("[VTASK] start %s\r\n", VisionTask_StateName(state));
}

static void VisionTask_Finish(void)
{
  VisionTask_Alert(0U);
  VisionTask_Buzzer(0U);
  task_state = VISION_TASK_IDLE;
  task_phase = 0U;
  printf("[VTASK] done, speed=%u\r\n", (unsigned)task_speed);
}

void VisionTask_Init(void)
{
  task_state = VISION_TASK_IDLE;
  task_start_tick = 0U;
  task_phase_tick = 0U;
  task_phase = 0U;
  task_speed = VISION_SPEED_NORMAL;
  horn_active = 0U;
  horn_phase = 0U;
  horn_phase_tick = 0U;
  VisionTask_Alert(0U);
  VisionTask_Buzzer(0U);
}

uint8_t VisionTask_GetSpeed(void) { return task_speed; }
VisionTask_State VisionTask_GetState(void) { return task_state; }

const char *VisionTask_StateName(VisionTask_State state)
{
  switch (state)
  {
    case VISION_TASK_TURN_LEFT: return "TURN_LEFT";
    case VISION_TASK_TURN_RIGHT: return "TURN_RIGHT";
    case VISION_TASK_PARK_1: return "PARK_1";
    case VISION_TASK_PARK_2: return "PARK_2";
    case VISION_TASK_STOPPED: return "STOPPED";
    default: return "IDLE";
  }
}

/**
  * @brief  按事件创建任务（调度器入口）
  * @retval 1=起了占用底盘的任务，0=只改配置或被忽略
  *
  * @note   限速/解除限速是持续配置，只改巡线速度，不创建任务也不抢占底盘。
  *
  * @note   任务运行中不接收新任务事件，避免反复重启动作。
  *
  * @note   VISION_TASK_STOPPED（1 号库停稳后）**只接受 2 号库**：其余视觉任务
  *         事件一律忽略，车原地不动、继续识别，直到 2 号库出现才重新启动。
  *         改动前这里接受任何任务事件，鸣笛或转向标志都能把车从库里叫走。
  *
  *         限速/解除限速不受此限——它们只改配置不动底盘，停着改完等启动后生效。
  */
uint8_t VisionTask_Begin(Event_Type type)
{
  uint8_t acceptable;

  if (task_state == VISION_TASK_IDLE)
  {
    acceptable = 1U;
  }
  else if (task_state == VISION_TASK_STOPPED)
  {
    /* 停在 1 号库里：只有 2 号库能把车叫起来。 */
    acceptable = (uint8_t)((type == EVENT_VISION_PARK_2) ? 1U : 0U);

    /* 限速类不算"被忽略的任务"，别误报。 */
    if ((acceptable == 0U) &&
        (type != EVENT_VISION_SPEED_LIMIT) &&
        (type != EVENT_VISION_SPEED_RELEASE))
    {
      printf("[VTASK] parked at 1, ignore %s (waiting for PARK_2)\r\n",
             Event_TypeName(type));
    }
  }
  else
  {
    /* 任务进行中（含 1 号库那 2s 闪灯相位）：不接新任务。 */
    acceptable = 0U;
  }

  switch (type)
  {
    case EVENT_VISION_SPEED_LIMIT:
      /* 限速：在正常速度基础上降低固定占空比。
         下限保护交给 LineTracker_SetBaseSpeed()——那里才知道死区和差速需要
         多少余量，在这里判 0 反而会算出一个电机根本不转的值。 */
      task_speed = VISION_SPEED_LIMITED;
      printf("[VTASK] speed limited to %u\r\n", (unsigned)task_speed);
      return 0U;

    case EVENT_VISION_SPEED_RELEASE:
      /* 解除限速：恢复正常巡线速度。 */
      task_speed = VISION_SPEED_NORMAL;
      printf("[VTASK] speed restored to %u\r\n", (unsigned)task_speed);
      return 0U;

    case EVENT_VISION_TURN_LEFT:
      if (acceptable == 0U) return 0U;
      VisionTask_Start(VISION_TASK_TURN_LEFT);
      return 1U;

    case EVENT_VISION_TURN_RIGHT:
      if (acceptable == 0U) return 0U;
      VisionTask_Start(VISION_TASK_TURN_RIGHT);
      return 1U;

    case EVENT_VISION_HORN:
      /* 鸣笛不占底盘、不停车：起一条独立的节奏，返回 0 让调度器保持当前模式
         （通常是循迹），车照常往前走。
         不看 acceptable——停在 1 号库里也可以鸣笛，那不影响"等 2 号库"。 */
      horn_active     = 1U;
      horn_phase      = 0U;
      horn_phase_tick = HAL_GetTick();

      /* 鸣笛之后按减速档行驶，与限速标志同一个档位。返回 0 之后调度器会把
         task_speed 同步到循迹基速（见 scheduler.c 的 SetBaseSpeed 调用），
         所以这里只改变量，不直接写循迹。

         降速是**持续**的，不随两声鸣笛结束而恢复：恢复只由"解除限速"标志或
         绿灯起步触发。 */
      task_speed = VISION_SPEED_LIMITED;
      printf("[VTASK] horn (no stop), speed limited to %u\r\n",
             (unsigned)task_speed);
      return 0U;

    case EVENT_VISION_PARK_1:
      if (acceptable == 0U) return 0U;
      VisionTask_Start(VISION_TASK_PARK_1);
      return 1U;

    case EVENT_VISION_PARK_2:
      if (acceptable == 0U) return 0U;
      VisionTask_Start(VISION_TASK_PARK_2);
      return 1U;

    default:
      return 0U;
  }
}

/**
  * @brief  鸣笛：响 0.5s、停 0.2s，共两声，**期间不停车**
  * @note   由主循环每轮调用，与控制模式无关。不碰底盘，只驱动蜂鸣器。
  */
void VisionTask_StepHorn(void)
{
  uint32_t now;
  uint8_t  beeping;
  uint32_t limit;

  if (horn_active == 0U)
  {
    return;
  }

  now     = HAL_GetTick();
  beeping = (uint8_t)((horn_phase % 2U) == 0U);
  limit   = (beeping != 0U) ? VISION_HORN_ON_MS : VISION_HORN_GAP_MS;

  VisionTask_Buzzer(beeping);

  if ((uint32_t)(now - horn_phase_tick) < limit)
  {
    return;
  }

  horn_phase++;
  horn_phase_tick = now;

  /* 每声占用「响 + 停」两个阶段。 */
  if (horn_phase >= (uint8_t)(VISION_HORN_COUNT * 2U))
  {
    horn_active = 0U;
    VisionTask_Buzzer(0U);
    printf("[VTASK] horn done\r\n");
  }
}

/**
  * @brief  闪灯，返回 1 表示闪灯阶段仍在进行
  */
static uint8_t VisionTask_RunBlink(void)
{
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - task_start_tick) >= VISION_PARK_BLINK_TOTAL_MS)
  {
    VisionTask_Alert(0U);
    return 0U;
  }
  if ((uint32_t)(now - task_phase_tick) >= VISION_PARK_BLINK_MS)
  {
    task_phase_tick = now;
    task_phase ^= 1U;
  }
  VisionTask_Alert(task_phase);
  return 1U;
}

/**
  * @brief  红灯（1 号库）：闪灯后停车并保持停止，直到绿灯
  */
static void VisionTask_RunPark1(void)
{
  Car_Stop();
  if (VisionTask_RunBlink() != 0U)
  {
    return;
  }
  VisionTask_Buzzer(0U);
  task_state = VISION_TASK_STOPPED;
  printf("[VTASK] red light, stopped, waiting for green\r\n");
}

/**
  * @brief  绿灯（2 号库）：设置起步速度后立刻交回循迹
  * @note   不再用 Car_ForwardRun() 盲走一段：那段时间 VisionTask_Step() 返回 1，
  *         底盘被视觉任务独占，循迹完全没有机会介入，弯道必冲出赛道。
  *         现在改成把起步速度写进循迹基速，然后 VisionTask_Finish() 交回底盘，
  *         由 LineTracker_Step() 从当前姿态继续走。
  *
  * @note   起步速度是**正常速度**，不是停之前那个速度。红灯前若因鸣笛或限速
  *         标志处在减速档，绿灯起步一并解除——见 VISION_PARK2_FULL_SPEED 注释。
  *         task_speed 也要一起改回去，否则下一个视觉事件会把减速档又同步下去。
  */
static void VisionTask_RunPark2(void)
{
#if VISION_PARK2_FULL_SPEED
  task_speed = VISION_SPEED_FULL;
#else
  task_speed = VISION_SPEED_NORMAL;
#endif

  LineTracker_SetBaseSpeed(task_speed);
  printf("[VTASK] green light, start at %u\r\n", (unsigned)task_speed);
  VisionTask_Finish();
}

/**
  * @brief  转弯任务：非阻塞推进原地旋转
  * @note   task_phase 0=还没发起动作，1=动作进行中。
  *         car 的角度动作改成状态机后，转弯期间主循环仍能仲裁急停和手动接管。
  */
static void VisionTask_RunTurn(int32_t angle_deg10)
{
  if (task_phase == 0U)
  {
    if (Car_StartRotateAngle(VISION_TURN_SPEED, angle_deg10) != CAR_ACTION_BUSY)
    {
      /* 发起失败（速度低于标定下限、或已有动作在跑）：放弃任务，交回循迹。 */
      printf("[VTASK] turn rejected\r\n");
      VisionTask_Finish();
      return;
    }
    task_phase = 1U;
    return;
  }

  switch (Car_ActionStep())
  {
    case CAR_ACTION_BUSY:
      break;

    case CAR_ACTION_DONE:
      VisionTask_Finish();
      break;

    default:
      /* 超时或失败：car 已制动，任务照样结束，避免卡在这一相。 */
      printf("[VTASK] turn failed\r\n");
      VisionTask_Finish();
      break;
  }
}

uint8_t VisionTask_Step(void)
{
  switch (task_state)
  {
    case VISION_TASK_TURN_LEFT:
      /* 带符号角度：正=左转。 */
      VisionTask_RunTurn((int32_t)VISION_TURN_ANGLE_DEG10);
      return 1U;

    case VISION_TASK_TURN_RIGHT:
      VisionTask_RunTurn(-(int32_t)VISION_TURN_ANGLE_DEG10);
      return 1U;

    /* 鸣笛不在这里：它不占底盘，由主循环直接调 VisionTask_StepHorn()。 */

    case VISION_TASK_PARK_1:
      VisionTask_RunPark1();
      return 1U;

    case VISION_TASK_PARK_2:
      VisionTask_RunPark2();
      return 1U;

    case VISION_TASK_STOPPED:
      /* 1 号库结束后保持停止，仍占用底盘；新的视觉任务事件可重新唤醒
         （见 VisionTask_Begin 对 STOPPED 的处理）。 */
      Car_Stop();
      return 1U;

    default:
      return 0U;
  }
}

void VisionTask_Cancel(void)
{
  if (task_state != VISION_TASK_IDLE)
  {
    printf("[VTASK] cancelled at %s\r\n", VisionTask_StateName(task_state));
  }
  Car_ActionAbort();
  VisionTask_Alert(0U);
  VisionTask_Buzzer(0U);
  task_state = VISION_TASK_IDLE;
  task_phase = 0U;
  /* 鸣笛虽然不占底盘，但急停要静音——那时蜂鸣器归故障告警用。 */
  horn_active = 0U;
}

uint8_t VisionTask_IsBusy(void)
{
  return (task_state != VISION_TASK_IDLE) ? 1U : 0U;
}

