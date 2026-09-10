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
/* 摇头搜寻要读循迹传感器。只用到 LineTracker_ReadPattern()（纯 GPIO 读，无副
   作用）和图案宏，不碰循迹的控制状态——写电机的仍然只有本模块。 */
#include "line_tracker.h"

#include <stdio.h>

static AvoidTask_State avoid_state;
static uint32_t avoid_start_tick;
static uint32_t avoid_phase_tick;
static uint8_t  avoid_attempts;
static uint8_t  avoid_reverses;
/* 本次转向方向：+1 左，-1 右。 */
static int8_t   avoid_turn_dir;
/* 绕行序列当前第几步。 */
static uint8_t  avoid_step;
/* 绕行镜像方向：+1 = 右绕（序列表原样），-1 = 左绕（每步转角取反）。 */
static int8_t   avoid_detour_mirror;

/* ---- 末段巡航 ---- */
/* 巡航自己的超时起点，与 avoid_start_tick 分开：开环五步的耗时不占巡航的额度。 */
static uint32_t avoid_cruise_tick;
/* 见线防抖：采样时刻与连续命中次数。 */
static uint32_t avoid_cruise_sample_tick;
static uint8_t  avoid_cruise_hits;

static void AvoidTask_EnterPhase(AvoidTask_State state);

const char *AvoidTask_StateName(AvoidTask_State state)
{
  switch (state)
  {
    case AVOID_BACKING: return "BACKING";
    case AVOID_TURNING: return "TURNING";
    case AVOID_VERIFY:  return "VERIFY";
    case AVOID_DETOUR:  return "DETOUR";
    case AVOID_CRUISE:  return "CRUISE";
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

/* ==== 固定绕行序列 ==========================================================
   七步写成一张表而不是七段几乎相同的代码：加一步、改一个角度只动数据。
   turn_deg10 非 0 表示这一步是旋转（正=左，负=右），否则是前进 forward_mm。

   表里存的是**右绕**（先右转让开）。左绕不再写第二张表，而是把每一步的
   turn_deg10 乘以 -1 镜像——两条路径的距离完全相同，只有转向符号相反。
   镜像方向存在 avoid_detour_mirror：+1 = 右绕（表原样），-1 = 左绕。
   ============================================================================ */
typedef struct
{
  int32_t  turn_deg10;   /* 旋转角，正=左转，负=右转；0 表示这步是前进 */
  uint32_t forward_mm;   /* 前进距离，仅当 turn_deg10 == 0 时有意义 */
  const char *name;      /* 仅用于前进步的日志；旋转步的方向按镜像现算 */
} AvoidTask_DetourStep;

static const AvoidTask_DetourStep avoid_detour[] =
{
  { -(int32_t)AVOID_DETOUR_TURN_DEG10 + 150, 0U,                    NULL          },
  { 0,                                 AVOID_DETOUR_LONG_MM,  "forward 300" },
  {  (int32_t)AVOID_DETOUR_TURN_DEG10 - 100, 0U,                    NULL          },
  { 0,                                 AVOID_DETOUR_CROSS_MM, "forward 500" },
  {  (int32_t)AVOID_DETOUR_TURN_DEG10 - 50, 0U,                    NULL          },
  /* 末段不在表里：最后一次左转之后转入 AVOID_CRUISE，持续前进到见线。 */
};

#define AVOID_DETOUR_STEPS  (sizeof(avoid_detour) / sizeof(avoid_detour[0]))

_Static_assert(AVOID_DETOUR_STEPS == 5U, "detour open-loop part must have 5 steps");

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

  /* 左绕时把转角取反。距离步不受影响——两条路径长度相同。 */
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

  if (turn != 0)
  {
    /* 角度按镜像后的实际值报，写死"60"会在改表或左绕时骗人。 */
    printf("[AVOID] detour step %u/%u: turn %s %ld.%u\r\n",
           (unsigned)(step + 1U), (unsigned)AVOID_DETOUR_STEPS,
           (turn > 0) ? "left" : "right",
           (long)(((turn < 0) ? -turn : turn) / 10),
           (unsigned)(((turn < 0) ? -turn : turn) % 10));
  }
  else
  {
    printf("[AVOID] detour step %u/%u: %s\r\n",
           (unsigned)(step + 1U), (unsigned)AVOID_DETOUR_STEPS, s->name);
  }
  return 1U;
}

#if AVOID_CRUISE_ENABLE

/**
  * @brief  巡航期间轮询循迹传感器，判断是否已经见到线
  * @retval 1 = 连续 AVOID_CRUISE_CONFIRM 次读到非全白，认定见线
  *
  * @note   按固定间隔采样而不是每轮都读。主循环一轮只几十微秒，不限速地读
  *         GPIO 只是把同一个瞬间读上几百遍，防抖等于没做。
  */
static uint8_t AvoidTask_CruiseSeesLine(uint32_t now)
{
  if ((uint32_t)(now - avoid_cruise_sample_tick) < AVOID_CRUISE_SAMPLE_MS)
  {
    return 0U;
  }
  avoid_cruise_sample_tick = now;

  if (LineTracker_ReadPattern() != LINE_PATTERN_ALL_WHITE)
  {
    avoid_cruise_hits++;
    return (uint8_t)(avoid_cruise_hits >= AVOID_CRUISE_CONFIRM);
  }

  /* 中间断了就重新数：要的是**连续**命中。 */
  avoid_cruise_hits = 0U;
  return 0U;
}

/**
  * @brief  开环五步走完，进入末段巡航
  * @retval 1=已进入，0=进不去（应直接结束本轮避障）
  */
static uint8_t AvoidTask_EnterCruise(uint32_t now)
{
  avoid_cruise_tick        = now;
  avoid_cruise_sample_tick = now;
  avoid_cruise_hits        = 0U;

  /* 已经压在线上就不用再往前走了。车此刻是停着的，这一眼不必防抖。 */
  if (LineTracker_ReadPattern() != LINE_PATTERN_ALL_WHITE)
  {
    printf("[AVOID] on line at detour end, no cruise\r\n");
    return 0U;
  }

  if (Car_StartForwardDistance(AVOID_CRUISE_SPEED,
                               AVOID_CRUISE_MAX_MM) != CAR_ACTION_BUSY)
  {
    printf("[AVOID] cruise rejected\r\n");
    return 0U;
  }

  printf("[AVOID] cruise until line (max %umm)\r\n",
         (unsigned)AVOID_CRUISE_MAX_MM);
  AvoidTask_EnterPhase(AVOID_CRUISE);
  return 1U;
}

#endif /* AVOID_CRUISE_ENABLE */

/**
  * @brief  按两侧红外原始值选一个更空旷的方向
  * @retval +1 向左转，-1 向右转
  *
  * @note   取代原来的伪随机选向。ADC 语义由 IR_AVOID_OBSTACLE_WHEN_HIGH 决定：
  *         为 0 时值越小越近，所以"更空旷"= 原始值更大的一侧。
  *
  * @note   两侧读数接近时固定偏右转（返回 -1），保证行为可复现。真要偏好某侧
  *         改这里一处即可。
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

/**
  * @brief  当前是否仍被挡住
  */
static uint8_t AvoidTask_StillBlocked(void)
{
  return (uint8_t)((IrAvoid_IsAnyBlocked() != 0U) ||
                   (UltrasonicSense_IsBlocked() != 0U));
}

/**
  * @brief  需不需要先后退：正前方受阻或左右同时受阻时，直接转向容易剐蹭
  */
static uint8_t AvoidTask_NeedsReverse(void)
{
  return (uint8_t)((UltrasonicSense_IsBlocked() != 0U) ||
                   (IrAvoid_GetState() == IR_AVOID_BOTH));
}

/**
  * @brief  进入某一相位
  */
static void AvoidTask_EnterPhase(AvoidTask_State state)
{
  avoid_state      = state;
  avoid_phase_tick = HAL_GetTick();
  printf("[AVOID] -> %s (attempt %u)\r\n",
         AvoidTask_StateName(state), (unsigned)avoid_attempts);
}

/**
  * @brief  发起本轮的避障动作
  * @note   单侧受阻只需小角度轻推；正前方或双侧受阻先后退再大角度转。
  */
#if AVOID_STRATEGY_DETOUR

/**
  * @brief  选绕行方向
  * @retval +1 = 右绕（先右转让开），-1 = 左绕，0 = 不该绕行
  *
  * @note   往障碍的**反侧**绕：左方受阻走右绕，右方受阻走左绕。
  *
  * @note   判定顺序是刻意的：
  *           - 左右同时受阻（IR_AVOID_BOTH）时两侧都没空间，固定右绕并接受
  *             它可能不成功——绕不开的话序列走完传感器会重新触发。总得选一个，
  *             选固定值是为了行为可复现（同一处障碍每次表现一致，便于调试）。
  *           - 只有超声波报前方、红外两侧都空时也走右绕：正前方障碍往哪边绕
  *             都行，固定右绕同样是为了可复现。
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

static void AvoidTask_StartAttempt(void)
{
  IrAvoid_State ir = IrAvoid_GetState();

  /* 不设上限：持续重试直到脱困。障碍是暂时的，失败不算故障。 */
  avoid_attempts++;

  /* 后退次数有上限，避免正对墙时反复倒退累计出很远的位移。 */
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

  /* 单侧受阻：往反方向轻推。左侧有障碍就右转，右侧有障碍就左转。
     双侧受阻但后退次数已用尽时按读数择路，不能再靠"哪侧有障碍"判断。 */
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

  /* 告警覆盖整个避障过程，结束时释放。左右侧红灯指示受阻方向。 */
  /* 告警覆盖整个避障过程。蜂鸣持续——避障时车在动，需要持续提醒周围的人。 */
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

    /* 传感器已恢复无障碍（事件到达与本函数之间隔了至少一轮采样）。
       没有障碍就不必绕，直接交还控制权。 */
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

/**
  * @brief  结束避障，交还控制权
  */
static uint8_t AvoidTask_Finish(uint8_t success)
{
  Car_Stop();
  Indicator_Release(INDICATOR_PRIO_AVOID);
  avoid_state = AVOID_IDLE;
  avoid_step  = 0U;

  /* 让传感器丢掉动作前的残留判定，下一轮重新采。 */
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

  /* 总超时兜底，防止卡在某一相。**不报故障**：只是结束本轮，交还控制权。
     障碍若仍在，传感器下一轮会重新触发避障，等于自动重来。

     绕行序列用自己的（更短的）超时：它是开环的固定路径，理论耗时可算，
     卡住就该早点结束，不必等 15 秒。 */
  {
    uint32_t limit;
    uint32_t base;

#if AVOID_CRUISE_ENABLE
    if (avoid_state == AVOID_CRUISE)
    {
      /* ⚠️ 巡航的额度**单独起算**。若沿用 avoid_start_tick，开环五步的 6~8s
         已经计入，巡航还没走几厘米就会被判超时。 */
      limit = AVOID_CRUISE_TIMEOUT_MS;
      base  = avoid_cruise_tick;
    }
    else
#endif
    if (avoid_state == AVOID_DETOUR)
    {
      limit = AVOID_DETOUR_TIMEOUT_MS;
      base  = avoid_start_tick;
    }
    else
    {
      limit = AVOID_TOTAL_TIMEOUT_MS;
      base  = avoid_start_tick;
    }

    if ((uint32_t)(now - base) >= limit)
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
            /* 开环五步走完。不判断障碍是否还在——障碍若还挡着，传感器下一轮
               会重新触发。 */
            printf("[AVOID] detour open-loop done\r\n");
#if AVOID_CRUISE_ENABLE
            if (AvoidTask_EnterCruise(now) != 0U)
            {
              break;
            }
            /* 进不去巡航（已经压在线上，或 car 拒绝动作）：直接交还。 */
#endif
            return AvoidTask_Finish(1U);
          }
          if (AvoidTask_StartDetourStep(avoid_step) == 0U)
          {
            return AvoidTask_Finish(0U);
          }
          avoid_phase_tick = now;
          break;

        default:
          /* 某一步超时或失败：car 已制动。序列做半截意味着车身姿态不确定，
             不再往下走，交还控制权由循迹重新找线。 */
          printf("[AVOID] detour step %u failed\r\n", (unsigned)avoid_step);
          return AvoidTask_Finish(0U);
      }
      break;

#if AVOID_CRUISE_ENABLE
    case AVOID_CRUISE:
      /* 见线判定放在 Car_ActionStep 之前：见线立刻停手，不等距离走完。 */
      if (AvoidTask_CruiseSeesLine(now) != 0U)
      {
        printf("[AVOID] cruise hit line, pattern=0x%02X\r\n",
               (unsigned)LineTracker_ReadPattern());
        Car_ActionAbort();
        return AvoidTask_Finish(1U);
      }

      switch (Car_ActionStep())
      {
        case CAR_ACTION_BUSY:
          break;

        case CAR_ACTION_DONE:
          /* 走满上限仍没见线：交还控制权，由循迹自己走丢线逻辑。 */
          printf("[AVOID] cruise reached %umm, no line\r\n",
                 (unsigned)AVOID_CRUISE_MAX_MM);
          return AvoidTask_Finish(0U);

        default:
          printf("[AVOID] cruise failed\r\n");
          return AvoidTask_Finish(0U);
      }
      break;
#endif /* AVOID_CRUISE_ENABLE */

    case AVOID_BACKING:
      switch (Car_ActionStep())
      {
        case CAR_ACTION_BUSY:
          break;

        case CAR_ACTION_DONE:
          /* 退够了。此时才有空间转向，方向按两侧读数择路。 */
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
      /* 停着等传感器出新数据。IrAvoid_Restart() 在这里不调，否则
         HasSample 一直为 0；只需等够一轮采样周期。 */
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
