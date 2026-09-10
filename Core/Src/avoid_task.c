/**
  ******************************************************************************
  * @file    avoid_task.c
  * @brief   避障行为状态机实现
  ******************************************************************************
  */

#include "avoid_task.h"
#include "ir_avoid.h"
#include "ultrasonic_sense.h"
#include "car.h"
#include "event.h"
#include "indicator.h"

#include <stdio.h>

static AvoidTask_State avoid_state;
static uint32_t avoid_start_tick;
static uint32_t avoid_phase_tick;
static uint8_t  avoid_attempts;
static uint8_t  avoid_reverses;
static int8_t   avoid_turn_dir;        /* +1 左，-1 右 */
static uint8_t  avoid_step;            /* 绕行序列当前第几步 */
static int8_t   avoid_detour_mirror;   /* +1 右绕（表原样），-1 左绕（转角取反） */

const char *AvoidTask_StateName(AvoidTask_State state)
{
  switch (state)
  {
    case AVOID_BACKING: return "BACKING";
    case AVOID_TURNING: return "TURNING";
    case AVOID_VERIFY:  return "VERIFY";
    case AVOID_DETOUR:  return "DETOUR";
    case AVOID_DONE:    return "DONE";
    case AVOID_FAILED:  return "FAILED";
    default:            return "IDLE";
  }
}

AvoidTask_State AvoidTask_GetState(void) { return avoid_state; }
uint8_t AvoidTask_GetStep(void) { return avoid_step; }

void AvoidTask_Init(void)
{
  avoid_state    = AVOID_IDLE;
  avoid_attempts = 0U;
  avoid_reverses = 0U;
  avoid_turn_dir = 1;
  avoid_step     = 0U;
  avoid_detour_mirror = 1;
}

/* 固定绕行序列，七步写成一张表 */
typedef struct
{
  int32_t  turn_deg10;   /* 旋转角，正=左转，负=右转；0 表示前进 */
  uint32_t forward_mm;   /* 前进距离，仅当 turn_deg10 == 0 时有意义 */
  const char *name;      /* 仅用于前进步的日志；旋转步的方向按镜像现算 */
} AvoidTask_DetourStep;

static const AvoidTask_DetourStep avoid_detour[] =
{
  { -(int32_t)AVOID_DETOUR_TURN_DEG10, 0U,                    NULL          },
  { 0,                                 AVOID_DETOUR_LONG_MM,  "forward 400" },
  {  (int32_t)AVOID_DETOUR_TURN_DEG10, 0U,                    NULL          },
  { 0,                                 AVOID_DETOUR_CROSS_MM, "forward 100" },
  {  (int32_t)AVOID_DETOUR_TURN_DEG10, 0U,                    NULL          },
  { 0,                                 AVOID_DETOUR_LONG_MM,  "forward 400" },
  { -(int32_t)AVOID_DETOUR_TURN_DEG10, 0U,                    NULL          },
};

#define AVOID_DETOUR_STEPS  (sizeof(avoid_detour) / sizeof(avoid_detour[0]))

_Static_assert(AVOID_DETOUR_STEPS == 7U, "detour sequence must have 7 steps");

/**
  * @brief  发起绕行序列的第 step 步
  * @retval 1=已发起，0=发起失败（car 拒绝）
  */
static uint8_t AvoidTask_StartDetourStep(uint8_t step)
{
  const AvoidTask_DetourStep *s;
  Car_ActionStatus st;
  int32_t turn;

  if (step >= AVOID_DETOUR_STEPS)
  {
    return 0U;
  }
  s = &avoid_detour[step];

  /* 左绕时转角取反；距离步不受影响。 */
  turn = s->turn_deg10 * (int32_t)avoid_detour_mirror;

  if (turn != 0)
  {
    st = Car_StartRotateAngle(AVOID_DETOUR_ROTATE_SPEED, turn);
  }
  else
  {
    st = Car_StartForwardDistance(AVOID_DETOUR_FORWARD_SPEED, s->forward_mm);
  }

  if (st != CAR_ACTION_BUSY)
  {
    printf("[AVOID] detour step %u rejected\r\n", (unsigned)step);
    return 0U;
  }

  /* 表里的 name 写的是右绕，左绕时左右反过来报。 */
  printf("[AVOID] detour step %u/%u: %s\r\n",
         (unsigned)(step + 1U), (unsigned)AVOID_DETOUR_STEPS,
         (turn != 0)
           ? ((turn > 0) ? "turn left 60" : "turn right 60")
           : s->name);
  return 1U;
}

/**
  * @brief  按两侧红外原始值选一个更空旷的方向
  * @retval +1 向左转，-1 向右转
  *
  * @note   取代原来的伪随机选向。ADC 语义由 IR_AVOID_OBSTACLE_WHEN_HIGH 决定。
  *         两侧读数接近时固定偏右转，保证行为可复现；要改偏好只动这一处。
  */
static int8_t AvoidTask_ChooseDirection(void)
{
  uint16_t left  = IrAvoid_GetLeftRaw();
  uint16_t right = IrAvoid_GetRightRaw();

#if IR_AVOID_OBSTACLE_WHEN_HIGH
  /* 值越大越近：往读数小的一侧转。 */
  return (left < right) ? 1 : -1;
#else
  /* 值越小越近：往读数大的一侧转。 */
  return (left > right) ? 1 : -1;
#endif
}

/** @brief 当前是否仍被挡住 */
static uint8_t AvoidTask_StillBlocked(void)
{
  return (uint8_t)((IrAvoid_IsAnyBlocked() != 0U) ||
                   (UltrasonicSense_IsBlocked() != 0U));
}

/** @brief 需不需要先后退：正前方或左右同时受阻时直接转向容易剐蹭 */
static uint8_t AvoidTask_NeedsReverse(void)
{
  return (uint8_t)((UltrasonicSense_IsBlocked() != 0U) ||
                   (IrAvoid_GetState() == IR_AVOID_BOTH));
}

/** @brief 进入某一相位 */
static void AvoidTask_EnterPhase(AvoidTask_State state)
{
  avoid_state      = state;
  avoid_phase_tick = HAL_GetTick();
  printf("[AVOID] -> %s (attempt %u)\r\n",
         AvoidTask_StateName(state), (unsigned)avoid_attempts);
}

#if AVOID_STRATEGY_DETOUR

/**
  * @brief  选绕行方向
  * @retval +1 = 右绕（先右转让开），-1 = 左绕，0 = 不该绕行
  *
  * @note   往障碍的**反侧**绕：左方受阻走右绕，右方受阻走左绕。
  *         其余情况（双侧受阻、或只有超声波报前方）固定右绕：
  */
static int8_t AvoidTask_DetourDirection(void)
{
  IrAvoid_State ir = IrAvoid_GetState();

  if (ir == IR_AVOID_RIGHT)
  {
    return -1;   /* 右方受阻 → 左绕 */
  }
  if (ir == IR_AVOID_LEFT)
  {
    return 1;    /* 左方受阻 → 右绕 */
  }

  /* 双侧受阻，或只有前方受阻：固定右绕。 */
  if ((ir == IR_AVOID_BOTH) || (UltrasonicSense_IsBlocked() != 0U))
  {
    return 1;
  }

  return 0;      /* 没有障碍，不该进这里 */
}

#endif /* AVOID_STRATEGY_DETOUR */

/**
  * @brief  发起本轮的避障动作（旧策略）
  * @note   单侧受阻只需小角度轻推；正前方或双侧受阻先后退再大角度转。
  */
static void AvoidTask_StartAttempt(void)
{
  IrAvoid_State ir = IrAvoid_GetState();

  avoid_attempts++;

  if ((AvoidTask_NeedsReverse() != 0U) &&
      (avoid_reverses < AVOID_MAX_REVERSES))
  {
    if (Car_StartBackwardDistance(AVOID_REVERSE_SPEED,
                                  AVOID_REVERSE_DIST_MM) != CAR_ACTION_BUSY)
    {
      printf("[AVOID] reverse rejected\r\n");
      avoid_state = AVOID_FAILED;
      return;
    }
    avoid_reverses++;
    AvoidTask_EnterPhase(AVOID_BACKING);
    return;
  }

  /* 单侧受阻往反方向轻推；双侧受阻且后退次数已用尽时按读数择路。 */
  if (ir == IR_AVOID_LEFT)
  {
    avoid_turn_dir = -1;
  }
  else if (ir == IR_AVOID_RIGHT)
  {
    avoid_turn_dir = 1;
  }
  else
  {
    avoid_turn_dir = AvoidTask_ChooseDirection();
  }
  if (Car_StartRotateAngle(AVOID_ROTATE_SPEED,
                           (int32_t)avoid_turn_dir *
                           (int32_t)AVOID_NUDGE_DEG10) != CAR_ACTION_BUSY)
  {
    printf("[AVOID] nudge rejected\r\n");
    avoid_state = AVOID_FAILED;
    return;
  }
  AvoidTask_EnterPhase(AVOID_TURNING);
}

void AvoidTask_Begin(void)
{
  Car_ActionAbort();
  avoid_start_tick = HAL_GetTick();
  avoid_attempts   = 0U;
  avoid_reverses   = 0U;

  /* 告警覆盖整个避障过程，结束时释放。左右红灯指示受阻方向；蜂鸣持续 */
  Indicator_Request(INDICATOR_PRIO_AVOID, INDICATOR_BUZZER_ON,
                    (IrAvoid_IsLeftBlocked()  != 0U) ? INDICATOR_RED : INDICATOR_OFF,
                    (IrAvoid_IsRightBlocked() != 0U) ? INDICATOR_RED : INDICATOR_OFF,
                    INDICATOR_BLINK_FAST_MS);

  printf("[AVOID] begin ir=%s front=%u\r\n",
         IrAvoid_StateName(IrAvoid_GetState()),
         (unsigned)UltrasonicSense_IsBlocked());

  avoid_step = 0U;

#if AVOID_STRATEGY_DETOUR
  {
    int8_t dir = AvoidTask_DetourDirection();

    if (dir != 0)
    {
      avoid_detour_mirror = dir;
      printf("[AVOID] detour %s\r\n", (dir > 0) ? "RIGHT" : "LEFT");

      if (AvoidTask_StartDetourStep(0U) == 0U)
      {
        avoid_state = AVOID_FAILED;
        return;
      }
      AvoidTask_EnterPhase(AVOID_DETOUR);
      return;
    }

    /* 事件到达与本函数之间隔了至少一轮采样，障碍可能已消失：不必绕。 */
    printf("[AVOID] no obstacle at begin, hand back\r\n");
    avoid_state = AVOID_DONE;
    return;
  }
#endif

  AvoidTask_StartAttempt();
}

void AvoidTask_Cancel(void)
{
  if (avoid_state != AVOID_IDLE)
  {
    printf("[AVOID] cancelled at %s\r\n", AvoidTask_StateName(avoid_state));
  }
  Car_ActionAbort();
  Indicator_Release(INDICATOR_PRIO_AVOID);
  avoid_state = AVOID_IDLE;
  avoid_step  = 0U;
}

/** @brief 结束避障，交还控制权 */
static uint8_t AvoidTask_Finish(uint8_t success)
{
  Car_Stop();
  Indicator_Release(INDICATOR_PRIO_AVOID);
  avoid_state = AVOID_IDLE;
  avoid_step  = 0U;

  /* 传感器丢掉动作前的残留判定，下一轮重新采。 */
  IrAvoid_Restart();
  UltrasonicSense_Restart();

  printf("[AVOID] end %s\r\n", (success != 0U) ? "clear" : "failed");
  return 0U;
}

uint8_t AvoidTask_Step(void)
{
  uint32_t now = HAL_GetTick();

  if (avoid_state == AVOID_IDLE)
  {
    return 0U;
  }

  /* 总超时兜底，防止卡在某一相。不报故障，语义见 avoid_task.h。 */
  {
    uint32_t limit = (avoid_state == AVOID_DETOUR) ? AVOID_DETOUR_TIMEOUT_MS
                                                   : AVOID_TOTAL_TIMEOUT_MS;

    if ((uint32_t)(now - avoid_start_tick) >= limit)
    {
      printf("[AVOID] total timeout at %s step %u, hand back\r\n",
             AvoidTask_StateName(avoid_state), (unsigned)avoid_step);
      return AvoidTask_Finish(0U);
    }
  }

  switch (avoid_state)
  {
    case AVOID_DETOUR:
      switch (Car_ActionStep())
      {
        case CAR_ACTION_BUSY:
          break;

        case CAR_ACTION_DONE:
          avoid_step++;
          if (avoid_step >= AVOID_DETOUR_STEPS)
          {
            /* 朝向与横向都已回正，交还控制权让循迹接上。不判断障碍是否还在。 */
            printf("[AVOID] detour complete\r\n");
            return AvoidTask_Finish(1U);
          }
          if (AvoidTask_StartDetourStep(avoid_step) == 0U)
          {
            return AvoidTask_Finish(0U);
          }
          avoid_phase_tick = now;
          break;

        default:
          /* 某步超时或失败，car 已制动。做半截的序列意味着姿态不确定，不再
             往下走，交还控制权由循迹重新找线。 */
          printf("[AVOID] detour step %u failed\r\n", (unsigned)avoid_step);
          return AvoidTask_Finish(0U);
      }
      break;

    case AVOID_BACKING:
      switch (Car_ActionStep())
      {
        case CAR_ACTION_BUSY:
          break;

        case CAR_ACTION_DONE:
          /* 退够了，此时才有空间转向。 */
          avoid_turn_dir = AvoidTask_ChooseDirection();
          if (Car_StartRotateAngle(AVOID_ROTATE_SPEED,
                                   (int32_t)avoid_turn_dir *
                                   (int32_t)AVOID_ROTATE_DEG10)
              != CAR_ACTION_BUSY)
          {
            printf("[AVOID] rotate rejected\r\n");
            return AvoidTask_Finish(0U);
          }
          AvoidTask_EnterPhase(AVOID_TURNING);
          break;

        default:
          printf("[AVOID] reverse failed\r\n");
          return AvoidTask_Finish(0U);
      }
      break;

    case AVOID_TURNING:
      switch (Car_ActionStep())
      {
        case CAR_ACTION_BUSY:
          break;

        case CAR_ACTION_DONE:
          /* 转完不直接认为脱困，先等新采样确认。 */
          AvoidTask_EnterPhase(AVOID_VERIFY);
          break;

        default:
          printf("[AVOID] rotate failed\r\n");
          return AvoidTask_Finish(0U);
      }
      break;

    case AVOID_VERIFY:
      /* 停着等一轮采样。这里**不能**调 IrAvoid_Restart()，否则 HasSample 恒为 0。 */
      Car_Stop();
      if ((uint32_t)(now - avoid_phase_tick) < AVOID_VERIFY_MS)
      {
        break;
      }
      if (AvoidTask_StillBlocked() == 0U)
      {
        return AvoidTask_Finish(1U);
      }
      /* 仍被挡住：再试一次，方向重新择路。 */
      printf("[AVOID] still blocked, retry\r\n");
      AvoidTask_StartAttempt();
      if (avoid_state == AVOID_FAILED)
      {
        return AvoidTask_Finish(0U);
      }
      break;

    case AVOID_FAILED:
      return AvoidTask_Finish(0U);

    case AVOID_DONE:
      return AvoidTask_Finish(1U);

    default:
      return AvoidTask_Finish(0U);
  }

  return 1U;
}
