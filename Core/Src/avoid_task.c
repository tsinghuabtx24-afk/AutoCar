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

/* ---- 摇头搜寻 ---- */
/* 已摆过几腿。 */
static uint8_t  avoid_search_leg;
/* 当前车头朝向，相对搜寻开始时刻，单位 0.1°（正=左）。存位置而不是增量，
   每腿的指令角度现算 target - cur，见 .h 里"位置，不是增量"那段。 */
static int32_t  avoid_search_cur_deg10;
/* 当前半幅，随腿数递增。 */
static int32_t  avoid_search_half_deg10;
/* 搜寻自己的超时起点，与 avoid_start_tick 分开：绕行耗时不占搜寻的额度。 */
static uint32_t avoid_search_tick;
/* 见线防抖：采样时刻与连续命中次数。 */
static uint32_t avoid_search_sample_tick;
static uint8_t  avoid_search_hits;

/* 摇头搜寻的函数放在绕行序列旁边（两者同属固定动作编排，挨着读更顺），
   但要用到下面才定义的 EnterPhase，所以先声明。 */
static void AvoidTask_EnterPhase(AvoidTask_State state);

const char *AvoidTask_StateName(AvoidTask_State state)
{
  switch (state)
  {
    case AVOID_BACKING: return "BACKING";
    case AVOID_TURNING: return "TURNING";
    case AVOID_VERIFY:  return "VERIFY";
    case AVOID_DETOUR:  return "DETOUR";
    case AVOID_SEARCH:  return "SEARCH";
    case AVOID_DONE:    return "DONE";
    case AVOID_FAILED:  return "FAILED";
    default:            return "IDLE";
  }
}

AvoidTask_State AvoidTask_GetState(void) { return avoid_state; }
uint8_t AvoidTask_GetStep(void) { return avoid_step; }
uint8_t AvoidTask_GetSearchLeg(void) { return avoid_search_leg; }

void AvoidTask_Init(void)
{
  avoid_state    = AVOID_IDLE;
  avoid_attempts = 0U;
  avoid_reverses = 0U;
  avoid_turn_dir = 1;
  avoid_step     = 0U;
  avoid_detour_mirror = 1;
  avoid_search_leg    = 0U;
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
  { -(int32_t)AVOID_DETOUR_TURN_DEG10 + 100, 0U,                    NULL          },
  { 0,                                 AVOID_DETOUR_LONG_MM,  "forward 300" },
  {  (int32_t)AVOID_DETOUR_TURN_DEG10, 0U,                    NULL          },
  { 0,                                 AVOID_DETOUR_CROSS_MM, "forward 500" },
  {  (int32_t)AVOID_DETOUR_TURN_DEG10, 0U,                    NULL          },
  { 0,                                 AVOID_DETOUR_LONG_MM,  "forward 300" },
  { -(int32_t)AVOID_DETOUR_TURN_DEG10, 0U,                    NULL          },
};

#define AVOID_DETOUR_STEPS  (sizeof(avoid_detour) / sizeof(avoid_detour[0]))

/* 序列必须回到原朝向、原横向位置，否则绕完接不回原来那条线。
   两次右转 + 两次左转，代数和为 0；两段 400mm 长度相同，横向偏出恰好抵消。 */
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

  /* 表里的 name 写的是右绕，左绕时左右要反过来报，否则日志会骗人。 */
  printf("[AVOID] detour step %u/%u: %s\r\n",
         (unsigned)(step + 1U), (unsigned)AVOID_DETOUR_STEPS,
         (turn != 0)
           ? ((turn > 0) ? "turn left 60" : "turn right 60")
           : s->name);
  return 1U;
}

#if AVOID_SEARCH_ENABLE

/**
  * @brief  摇头搜寻期间轮询循迹传感器，判断是否已经见到线
  * @retval 1 = 连续 AVOID_SEARCH_CONFIRM 次读到非全白，认定见线
  *
  * @note   全白(0x0F)之外的任何图案都算见线，**全黑(0x00)也算**：它出现在路口
  *         和黑区，同样是可跟的东西，没理由继续摆着找。
  *
  * @note   按固定间隔采样而不是每轮都读。主循环一轮只几十微秒，不限速地读
  *         GPIO 只是把同一个瞬间读上几百遍，防抖等于没做。
  */
static uint8_t AvoidTask_SearchSeesLine(uint32_t now)
{
  if ((uint32_t)(now - avoid_search_sample_tick) < AVOID_SEARCH_SAMPLE_MS)
  {
    return 0U;
  }
  avoid_search_sample_tick = now;

  if (LineTracker_ReadPattern() != LINE_PATTERN_ALL_WHITE)
  {
    avoid_search_hits++;
    return (uint8_t)(avoid_search_hits >= AVOID_SEARCH_CONFIRM);
  }

  /* 中间断了就重新数：要的是**连续**命中，累计命中防不住周期性毛刺。 */
  avoid_search_hits = 0U;
  return 0U;
}

/**
  * @brief  发起摇头搜寻的下一腿
  * @retval 1=已发起，0=发起失败（car 拒绝）
  *
  * @note   偶数腿去 +half（左），奇数腿去 -half（右）——先左后右。
  *         指令角度是 target - cur，所以第一腿转 30°，之后每腿都要跨过中线，
  *         转的是两倍振幅。
  */
static uint8_t AvoidTask_StartSearchLeg(void)
{
  int32_t target;
  int32_t delta;

  target = ((avoid_search_leg & 1U) == 0U) ?  avoid_search_half_deg10
                                           : -avoid_search_half_deg10;
  delta  = target - avoid_search_cur_deg10;

  if (delta == 0)
  {
    /* 振幅为 0 才可能走到这里（AVOID_SEARCH_HALF_DEG10 配成 0）。
       car 会拒绝 0 角度，直接当搜寻结束，别在这空转。 */
    return 0U;
  }

  if (Car_StartRotateAngle(AVOID_SEARCH_ROTATE_SPEED, delta) != CAR_ACTION_BUSY)
  {
    printf("[AVOID] search leg %u rejected (%ld)\r\n",
           (unsigned)avoid_search_leg, (long)delta);
    return 0U;
  }

  printf("[AVOID] search leg %u: %s %ld.%u (half %ld.%u)\r\n",
         (unsigned)avoid_search_leg,
         (delta > 0) ? "left" : "right",
         (long)(((delta < 0) ? -delta : delta) / 10),
         (unsigned)(((delta < 0) ? -delta : delta) % 10),
         (long)(avoid_search_half_deg10 / 10),
         (unsigned)(avoid_search_half_deg10 % 10));

  /* 目标即将达成（除非中途被见线打断，那时 cur 不再准确——但那种情况下搜寻
     已经结束，cur 不会再被用到）。 */
  avoid_search_cur_deg10 = target;
  return 1U;
}

/**
  * @brief  绕行走完，进入摇头搜寻
  * @retval 1=已进入，0=进不去（应直接结束本轮避障）
  */
static uint8_t AvoidTask_EnterSearch(uint32_t now)
{
  avoid_search_leg         = 0U;
  avoid_search_cur_deg10   = 0;
  avoid_search_half_deg10  = (int32_t)AVOID_SEARCH_HALF_DEG10;
  avoid_search_tick        = now;
  avoid_search_sample_tick = now;
  avoid_search_hits        = 0U;

  /* 绕行第七步刚结束时 car 已制动并转 IDLE，这里不必 Abort。 */

  /* 先看一眼：绕行结束时可能本来就压在线上，那就不用摆了。
     这一眼不走防抖——它不是"转动中的瞬时读数"，车此刻是停着的，
     没有扫过线的时序问题。 */
  if (LineTracker_ReadPattern() != LINE_PATTERN_ALL_WHITE)
  {
    printf("[AVOID] on line at detour end, no search\r\n");
    return 0U;
  }

  if (AvoidTask_StartSearchLeg() == 0U)
  {
    return 0U;
  }

  AvoidTask_EnterPhase(AVOID_SEARCH);
  return 1U;
}

#endif /* AVOID_SEARCH_ENABLE */

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
  avoid_search_leg = 0U;

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
  avoid_search_leg = 0U;
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
  avoid_search_leg = 0U;

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

#if AVOID_SEARCH_ENABLE
    if (avoid_state == AVOID_SEARCH)
    {
      /* ⚠️ 搜寻的额度**单独起算**。若沿用 avoid_start_tick，绕行七步的 6~8s
         已经计入，搜寻还没摆两腿就会被判超时——摇头等于没加。 */
      limit = AVOID_SEARCH_TIMEOUT_MS;
      base  = avoid_search_tick;
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
            /* 七步走完。不判断障碍是否还在——固定序列的定义就是走完这条路径；
               障碍若还挡着，传感器下一轮会重新触发。 */
            printf("[AVOID] detour complete\r\n");
#if AVOID_SEARCH_ENABLE
            /* 理论上朝向与横向都回到了原样，但七步是开环的，误差一路累加，
               实际常常偏出线外。先摇头找线，找到再交还控制权。 */
            if (AvoidTask_EnterSearch(now) != 0U)
            {
              break;
            }
            /* 进不去搜寻（已经压在线上，或 car 拒绝旋转）：按原样直接交还。 */
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

#if AVOID_SEARCH_ENABLE
    case AVOID_SEARCH:
      /* 见线判定放在 Car_ActionStep 之前：这一腿可能刚好在本轮转完，先判定
         就能在"转完"和"见线"同时成立时优先认见线，少摆一腿。 */
      if (AvoidTask_SearchSeesLine(now) != 0U)
      {
        printf("[AVOID] search hit line at leg %u, pattern=0x%02X\r\n",
               (unsigned)avoid_search_leg, (unsigned)LineTracker_ReadPattern());
        /* 打断旋转。这是本状态存在的意义——不转完再看，见线立刻停手，
           姿态停在压着线的那一刻，循迹接上时偏差最小。 */
        Car_ActionAbort();
        return AvoidTask_Finish(1U);
      }

      switch (Car_ActionStep())
      {
        case CAR_ACTION_BUSY:
          break;

        case CAR_ACTION_DONE:
          /* 这一腿摆完还没见线，换方向摆下一腿，并把振幅张开一点。 */
          avoid_search_leg++;
          avoid_search_half_deg10 += (int32_t)AVOID_SEARCH_GROW_DEG10;
          if (avoid_search_half_deg10 > (int32_t)AVOID_SEARCH_MAX_HALF_DEG10)
          {
            avoid_search_half_deg10 = (int32_t)AVOID_SEARCH_MAX_HALF_DEG10;
          }
          if (AvoidTask_StartSearchLeg() == 0U)
          {
            printf("[AVOID] search cannot continue, hand back\r\n");
            return AvoidTask_Finish(0U);
          }
          avoid_phase_tick = now;
          break;

        default:
          /* 某一腿超时或失败：car 已制动。交还控制权，让循迹走自己的丢线逻辑。 */
          printf("[AVOID] search leg %u failed\r\n", (unsigned)avoid_search_leg);
          return AvoidTask_Finish(0U);
      }
      break;
#endif /* AVOID_SEARCH_ENABLE */

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
