/**
  ******************************************************************************
  * @file    line_tracker.c
  * @brief   四路红外黑线循迹实现
  ******************************************************************************
  */

#include "line_tracker.h"
#include "car.h"
#include "event.h"
#include "encoder.h"
#include "retarget.h"

#include <stdio.h>

/* 死区约束：最大档差速减下来必须仍在死区之上，否则内侧轮会直接停转，变成
   急转而不是修正。

   基础速度现在是运行时可变的（视觉限速要生效），所以这条约束改为运行时钳位，
   见 LineTracker_EffDiff()。这里的编译期检查只针对**默认值**，确保默认配置
   本身是合理的。 */
_Static_assert((LINE_BASE_SPEED - 2U * LINE_DIFF_STEP) > CAR_SPEED_BASE,
               "default LINE_BASE_SPEED too low for LINE_DIFF_STEP: inner "
               "wheel would fall into the motor dead zone");
_Static_assert((LINE_BASE_SPEED + 2U * LINE_DIFF_STEP) <= MOTOR_SPEED_MAX,
               "LINE_BASE_SPEED + max diff exceeds MOTOR_SPEED_MAX");

/* 阈值高于窗长的话永远凑不满，边缘图案就再也不会改判原地转。 */
_Static_assert(LINE_EDGE_WHITE_COUNT <= LINE_EDGE_WATCH_CYCLES,
               "LINE_EDGE_WHITE_COUNT exceeds LINE_EDGE_WATCH_CYCLES: edge "
               "patterns could never escalate to pivot");
_Static_assert(LINE_EDGE_WHITE_COUNT >= 1U,
               "LINE_EDGE_WHITE_COUNT must be at least 1");

/* 循迹内部状态。原地转是一个持续若干周期的相位，需要记住方向。 */
typedef enum
{
  LINE_MODE_DIFF = 0,       /* 差速连续修正 */
  LINE_MODE_PIVOT,          /* 大偏差原地转，边转边看传感器 */
  LINE_MODE_LOST            /* 判定丢线，已制动 */
} LineTracker_Mode;

static uint8_t  line_last_pattern = 0xFFU;
static uint32_t line_last_tick;
static LineTracker_Mode line_mode;
static int8_t   line_pivot_dir;      /* +1 向左转，-1 向右转 */
static uint32_t line_pivot_tick;
static uint16_t line_lost_count;
/* 基础速度。默认为 LINE_BASE_SPEED，视觉限速/恢复时由调度器改写。 */
static uint8_t  line_base_speed = LINE_BASE_SPEED;
/* 视觉旋转的起始编码器角度快照，用于打印实测转角供标定。 */

/* ---- 偏离图案（0001/1000/0011/1100）的观察窗 ----
   见 line_tracker.h。看到这些图案时开窗，窗口内数全白次数，够多就改判原地转。 */
static uint8_t  line_edge_watch;      /* 0=未开窗 */
static int8_t   line_edge_dir;        /* 开窗时记下的方向：+1 左转，-1 右转 */
static uint8_t  line_edge_left;       /* 观察窗剩余周期数 */
static uint8_t  line_edge_white;      /* 窗口内累计的全白次数 */

/* 黑线暂停：连续全黑计数、暂停起始时刻、是否允许再次触发 */
static uint8_t  line_black_count;
static uint32_t line_black_pause_tick;   /* 0 = 不在暂停中 */
static uint8_t  line_black_armed;

/* 补给岔道规则：0=未武装，1=已武装等全黑，2=见过全黑等全白 */
static uint8_t  line_fork_stage;
static uint32_t line_fork_arm_tick;   /* 武装时刻，用于总超时 */
static uint32_t line_fork_black_tick; /* 见到全黑的时刻，用于等白时限 */

static void LineTracker_EdgeWatchCancel(void);

uint8_t LineTracker_ReadPattern(void)
{
  uint8_t pattern = 0U;

  pattern |= (HAL_GPIO_ReadPin(LINE_SENSOR_S1_PORT, LINE_SENSOR_S1_PIN)
              == LINE_WHITE_LEVEL) ? 0x08U : 0U;
  pattern |= (HAL_GPIO_ReadPin(LINE_SENSOR_S2_PORT, LINE_SENSOR_S2_PIN)
              == LINE_WHITE_LEVEL) ? 0x04U : 0U;
  pattern |= (HAL_GPIO_ReadPin(LINE_SENSOR_S3_PORT, LINE_SENSOR_S3_PIN)
              == LINE_WHITE_LEVEL) ? 0x02U : 0U;
  pattern |= (HAL_GPIO_ReadPin(LINE_SENSOR_S4_PORT, LINE_SENSOR_S4_PIN)
              == LINE_WHITE_LEVEL) ? 0x01U : 0U;

  return pattern;
}

/**
  * @brief  按当前基础速度算出可用的一档差速
  * @note   死区运行时钳位：base - 2×diff 必须 > CAR_SPEED_BASE。
  *         速度 70 → 6（与改动前一致）；限速到 60 → 4。
  */
static uint8_t LineTracker_EffDiff(void)
{
  uint8_t room;

  if (line_base_speed <= (uint8_t)(CAR_SPEED_BASE + 1U))
  {
    return 0U;   /* 没有差速空间，只能直行 */
  }
  room = (uint8_t)((line_base_speed - CAR_SPEED_BASE - 1U) / 2U);

  return (room < LINE_DIFF_STEP) ? room : (uint8_t)LINE_DIFF_STEP;
}

void LineTracker_SetBaseSpeed(uint8_t speed)
{
  /* 下限：至少要留出一档差速的空间，否则限速等于让车走不动。 */
  uint8_t lo = (uint8_t)(CAR_SPEED_BASE + 3U);
  uint8_t s  = (speed < lo) ? lo : speed;

  if (s > MOTOR_SPEED_MAX)
  {
    s = MOTOR_SPEED_MAX;
  }
  if (s == line_base_speed)
  {
    return;
  }
  line_base_speed = s;
  printf("[LINE] base speed = %u (diff step %u)\r\n",
         (unsigned)line_base_speed, (unsigned)LineTracker_EffDiff());
}

uint8_t LineTracker_GetBaseSpeed(void) { return line_base_speed; }

void LineTracker_Reset(void)
{
  LineTracker_EdgeWatchCancel();

  line_mode        = LINE_MODE_DIFF;
  line_pivot_dir   = 0;
  line_lost_count  = 0U;
  line_last_pattern = 0xFFU;
  line_last_tick   = 0U;   /* 立刻允许下一次控制计算 (see fork note below) */

  /* 黑线暂停状态清掉：被抢占（视觉任务/避障/手动）后回到循迹时，那次暂停的
     计时早已过期，留着会让恢复后的第一个周期误判成"还在暂停中"。
     armed 置 1 = 恢复后遇到下一片黑区可以正常再停一次。 */
  line_black_count      = 0U;
  line_black_pause_tick = 0U;
  line_black_armed      = 1U;

  /* 基础速度**不重置**：限速是持续配置，被避障打断后应该保持，
     由视觉的"解除限速"事件恢复。 */

  /* 岔道武装状态**也不重置**，这一条是必需的而非可选：
     补给掉头在 VisionTask 里武装，紧接着调度器就会切回 CONTROL_LINE_TRACK
     并调到这里。若在此清掉，刚武装的规则当场失效，岔道永远等不到左转。
     要解除请显式调 LineTracker_DisarmForkLeft()（急停走这条路）。 */
}

void LineTracker_Init(void)
{
  /* Reset() 有意不清岔道状态（见其中注释），所以上电初值在这里给。 */
  line_fork_stage      = 0U;
  line_fork_arm_tick   = 0U;
  line_fork_black_tick = 0U;

  LineTracker_Reset();
  Car_Stop();
  printf("[LINE] init S1..S4: black=0 white=1, base=%u diff=%u\r\n",
         (unsigned)LINE_BASE_SPEED, (unsigned)LINE_DIFF_STEP);
}

/**
  * @brief  把传感器图案翻译成偏差档位
  * @param  pattern: 四路图案，bit3=S1(左) ... bit0=S4(右)
  * @param  level:   输出偏差档位，正=需向左修正，负=需向右修正，0=居中
  * @retval 1=图案已定义，0=未定义图案
  *
  * @note   方向约定与原实现完全一致，未做改动——那套映射是实车验证过的。
  *         这里只是把"转多少度"换成"偏差几档"，由调用方决定用差速还是原地转。
  */
static uint8_t LineTracker_Decode(uint8_t pattern, int8_t *level)
{
  switch (pattern)
  {
    case 0x09U: *level =  0; return 1U;  /* 1001：轨迹中心 */

    /* 0001：先按微右偏处理，同时开观察窗；窗口内全白够多则改判原地转。
       见 line_tracker.h 的"单侧边缘图案的两段判定"。 */
    case 0x01U: *level =  1; return 1U;
    case 0x0BU: *level =  1; return 1U;  /* 1011：微右偏 */
    /* 0011：轨迹右端。先按 2 档处理，同样开观察窗。 */
    case 0x03U: *level =  2; return 1U;
    case 0x07U: *level =  3; return 1U;  /* 0111：轨迹极右 */

    /* 1000：同上，先按微左偏处理并开观察窗。 */
    case 0x08U: *level = -1; return 1U;
    case 0x0DU: *level = -1; return 1U;  /* 1101：微左偏 */
    /* 1100：轨迹左端。先按 2 档处理，同样开观察窗。 */
    case 0x0CU: *level = -2; return 1U;
    case 0x0EU: *level = -3; return 1U;  /* 1110：轨迹极左 */

    default:    *level =  0; return 0U;
  }
}

/**
  * @brief  差速修正：车持续前进，靠左右轮速差纠正航向
  * @param  level: 偏差档位，正=向左修正
  *
  * @note   向左修正 = 左轮减速、右轮加速，与 Car_RotateLeft 的
  *         (-left, +right) 方向约定一致。
  */
static void LineTracker_DriveDiff(int8_t level)
{
  int16_t step  = (int16_t)LineTracker_EffDiff();
  int16_t diff  = (int16_t)((int16_t)level * step);
  int16_t left  = (int16_t)((int16_t)line_base_speed - diff);
  int16_t right = (int16_t)((int16_t)line_base_speed + diff);

  Car_DriveSpeed(left, right);
}

/**
  * @brief  关掉边缘观察窗
  */
static void LineTracker_EdgeWatchCancel(void)
{
  line_edge_watch = 0U;
  line_edge_dir   = 0;
  line_edge_left  = 0U;
  line_edge_white = 0U;
}

/**
  * @brief  这个图案要开观察窗吗
  * @note   只有含义有歧义的四个：0001/1000（1 档）和 0011/1100（2 档）。
  *         1011/1101 中间那路还压着线，不歧义，不开。
  */
static uint8_t LineTracker_IsWatchPattern(uint8_t pattern)
{
  return (uint8_t)((pattern == LINE_PATTERN_EDGE_R) ||
                   (pattern == LINE_PATTERN_EDGE_L) ||
                   (pattern == LINE_PATTERN_END_R)  ||
                   (pattern == LINE_PATTERN_END_L));
}

/**
  * @brief  看到偏离图案，开观察窗
  * @param  level: Decode 给出的偏差档位，其符号就是要转的方向
  *
  * @note   同方向的窗口已经开着就不重开——偏离图案通常连着好几个周期都在，
  *         每次都重置计数的话窗口永远走不完，全白次数也永远凑不满。
  *         这也让 0011→0001 这种"越偏越多"的序列共用一个窗口，不会因为
  *         图案换了就把已经数到的全白丢掉。
  */
static void LineTracker_EdgeWatchArm(int8_t level)
{
  int8_t dir = (level > 0) ? 1 : -1;

  if ((line_edge_watch != 0U) && (line_edge_dir == dir))
  {
    return;
  }

  line_edge_watch = 1U;
  line_edge_dir   = dir;
  line_edge_left  = LINE_EDGE_WATCH_CYCLES;
  line_edge_white = 0U;
}

/**
  * @brief  推进观察窗一个周期
  * @param  pattern: 本周期读到的图案
  * @retval 1=全白够多，应改判原地转（方向在 *level 里）；0=不需要
  *
  * @note   窗口自然到期就作废，不留状态：那说明只是微偏，差速已经在修了。
  */
static uint8_t LineTracker_EdgeWatchStep(uint8_t pattern, int8_t *level)
{
  if (line_edge_watch == 0U)
  {
    return 0U;
  }

  if (pattern == LINE_PATTERN_ALL_WHITE)
  {
    line_edge_white++;

    if (line_edge_white >= LINE_EDGE_WHITE_COUNT)
    {
      /* 翻案：这不是微偏，是弯道。方向用开窗时记下的那个——此刻四路全白，
         当前图案已经给不出方向了。 */
      *level = (int8_t)(line_edge_dir * 3);
      printf("[LINE] edge escalated to pivot (%u white in window)\r\n",
             (unsigned)line_edge_white);
      LineTracker_EdgeWatchCancel();
      return 1U;
    }
  }

  if (line_edge_left > 0U)
  {
    line_edge_left--;
  }
  if (line_edge_left == 0U)
  {
    LineTracker_EdgeWatchCancel();
  }

  return 0U;
}

/**
  * @brief  进入大偏差原地转
  * @note   不用 Car_StartRotateAngle()：那是"转到指定角度就停"，而循迹要的是
  *         "转到压回线就停"。后者是传感器闭环，比按角度开环更准，也不依赖
  *         CAR_ROT_COMP_* 滑移补偿表。用瞬时接口 + 每周期判退出即可，
  *         同样是非阻塞的。
  */
void LineTracker_ArmForkLeft(void)
{
  line_fork_stage      = 1U;
  line_fork_arm_tick   = HAL_GetTick();
  line_fork_black_tick = 0U;
  printf("[LINE] fork-left armed\r\n");
}

void LineTracker_DisarmForkLeft(void)
{
  if (line_fork_stage != 0U)
  {
    printf("[LINE] fork-left disarmed\r\n");
  }
  line_fork_stage = 0U;
}

uint8_t LineTracker_IsForkArmed(void)
{
  return (uint8_t)((line_fork_stage != 0U) ? 1U : 0U);
}

/**
  * @brief  推进岔道规则的两段判定
  * @param  pattern 本周期图案
  * @param  now     当前 tick
  * @retval 1=此刻应当左转，0=不动作
  */
static uint8_t LineTracker_ForkStep(uint8_t pattern, uint32_t now)
{
  if (line_fork_stage == 0U)
  {
    return 0U;
  }

  /* 总超时：挂太久说明这趟没碰到岔道，别一直等着。 */
  if ((uint32_t)(now - line_fork_arm_tick) >= LINE_FORK_ARM_TIMEOUT_MS)
  {
    printf("[LINE] fork-left timeout, disarm\r\n");
    line_fork_stage = 0U;
    return 0U;
  }

  if (line_fork_stage == 1U)
  {
    /* 等岔道口的全黑。 */
    if (pattern == LINE_PATTERN_ALL_BLACK)
    {
      line_fork_stage      = 2U;
      line_fork_black_tick = now;
      printf("[LINE] fork-left: all-black seen, waiting white\r\n");
    }
    return 0U;
  }

  /* stage 2：见过全黑，等全白。 */
  if (pattern == LINE_PATTERN_ALL_WHITE)
  {
    printf("[LINE] fork-left: white after black -> pivot left\r\n");
    line_fork_stage = 0U;    /* 一次性，转完不再触发 */
    return 1U;
  }

  /* 等白超时：这次全黑不是岔道口，退回等下一次全黑。
     不直接解除武装——真正的岔道可能还在后面。 */
  if ((uint32_t)(now - line_fork_black_tick) >= LINE_FORK_WHITE_WAIT_MS)
  {
    line_fork_stage = 1U;
    printf("[LINE] fork-left: white wait expired, rearm for black\r\n");
  }
  return 0U;
}

static void LineTracker_EnterPivot(int8_t level)
{
  /* 已经在转了，观察窗没意义了。 */
  LineTracker_EdgeWatchCancel();

  line_pivot_dir  = (level > 0) ? 1 : -1;
  line_pivot_tick = HAL_GetTick();
  line_mode       = LINE_MODE_PIVOT;
  printf("[LINE] pivot %s\r\n", (line_pivot_dir > 0) ? "left" : "right");
}

static void LineTracker_DrivePivot(void)
{
  int16_t v = (int16_t)LINE_PIVOT_SPEED;

  if (line_pivot_dir > 0)
  {
    Car_DriveSpeed((int16_t)(-v), v);   /* 向左原地转 */
  }
  else
  {
    Car_DriveSpeed(v, (int16_t)(-v));   /* 向右原地转 */
  }
}

/**
  * @brief  推进黑线暂停：全黑攒够周期就停 0.5s
  * @param  pattern 本周期图案
  * @param  now     当前 tick
  * @retval 1=本周期应保持停止（调用方立即 return），0=照常循迹
  */
static uint8_t LineTracker_BlackPauseStep(uint8_t pattern, uint32_t now)
{
  /* 暂停进行中：停够就放行，并保持闩锁（离开黑区才会重新武装）。 */
  if (line_black_pause_tick != 0U)
  {
    if ((uint32_t)(now - line_black_pause_tick) < LINE_BLACK_PAUSE_MS)
    {
      Car_Stop();
      return 1U;
    }
    line_black_pause_tick = 0U;
    line_black_count      = 0U;
    printf("[LINE] black pause done, resume at base %u\r\n",
           (unsigned)line_base_speed);
    /* 不 return：本周期直接按当前图案继续循迹，不浪费一个周期。 */
  }

  if (pattern == LINE_PATTERN_ALL_BLACK)
  {
    if (line_black_count < 0xFFU)
    {
      line_black_count++;
    }

    if ((line_black_armed != 0U) &&
        (line_black_count >= LINE_BLACK_PAUSE_COUNT))
    {
      /* 闩住：停完若仍在黑区上，不会立刻再停一次。 */
      line_black_armed      = 0U;
      line_black_pause_tick = (now != 0U) ? now : 1U;

      /* 观察窗在暂停期间推不动，且停完车身姿态未变但节奏已断，直接作废，
         免得恢复后拿着旧方向误判一次原地转。 */
      LineTracker_EdgeWatchCancel();

      Car_Stop();
      printf("[LINE] all-black %u cycles, pause %u ms\r\n",
             (unsigned)LINE_BLACK_PAUSE_COUNT, (unsigned)LINE_BLACK_PAUSE_MS);
      return 1U;
    }
    return 0U;
  }

  /* 离开黑区：清计数并重新武装，下一片黑可以再停。 */
  line_black_count = 0U;
  line_black_armed = 1U;
  return 0U;
}

/**
  * @brief  判定丢线并制动
  */
static void LineTracker_EnterLost(void)
{
  line_mode = LINE_MODE_LOST;
  Car_Stop();
  printf("[LINE] line lost, brake\r\n");
  (void)Event_Post(EVENT_FAULT, 0U);
}

void LineTracker_Step(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t  pattern;
  uint8_t  changed;
  int8_t   level = 0;
  uint8_t  known;

  /* 固定控制周期：未到就维持上次轮速直接返回。主循环每轮几十微秒，
     不把住周期的话修正会一次接一次连着来，比原来的节奏激进得多。 */
  if ((uint32_t)(now - line_last_tick) < LINE_CONTROL_PERIOD_MS)
  {
    return;
  }
  line_last_tick = now;

  pattern = LineTracker_ReadPattern();
  changed = (pattern != line_last_pattern) ? 1U : 0U;

  if (changed != 0U)
  {
    line_last_pattern = pattern;
#if LINE_DEBUG_PATTERN
    printf("[LINE] pattern=%u%u%u%u (0x%02X)\r\n",
           (pattern >> 3) & 1U,
           (pattern >> 2) & 1U,
           (pattern >> 1) & 1U,
           pattern & 1U,
           (unsigned)pattern);
#endif
  }

  known = LineTracker_Decode(pattern, &level);

  /* ---- 边缘观察窗：数全白，够多就把之前的"微偏"改判成原地转 ----
     必须放在全白判定**之前**：全白那一段会 return，放后面就永远数不到
     它要数的那个图案。只在 DIFF 模式下改判——已经在 pivot 里没必要重来，
     LOST 时该做的是重新找线而不是转。 */
  {
    int8_t esc_level = 0;

    if ((LineTracker_EdgeWatchStep(pattern, &esc_level) != 0U) &&
        (line_mode == LINE_MODE_DIFF))
    {
      line_lost_count = 0U;   /* 转弯中四路离线是正常的，不算丢线 */
      LineTracker_EnterPivot(esc_level);
      LineTracker_DrivePivot();
      return;
    }
  }

  /* ---- 补给岔道规则：全黑 → 全白 → 原地左转 ----
     必须放在紧接着的全黑/全白两段之前，那两段都会 return，
     放后面就永远看不到它要等的图案。

     也必须排在黑线暂停**之前**：暂停期间 Step 会提前 return，岔道规则推不动。
     顺序反过来的话那次全黑会被暂停吃掉，岔道永远停在"等全黑"。 */
  if (LineTracker_ForkStep(pattern, now) != 0U)
  {
    line_lost_count = 0U;      /* 岔道口的全白是预期的，不算丢线 */
    LineTracker_EnterPivot(1); /* level>0 = 向左 */
    LineTracker_DrivePivot();
    return;
  }

  /* ---- 黑线暂停：全黑攒够 2 周期就停 0.5s，然后原速继续 ---- */
  if (LineTracker_BlackPauseStep(pattern, now) != 0U)
  {
    line_lost_count = 0U;      /* 全黑不是丢线 */
    return;
  }

  /* ---- 全黑：不停车、不判丢线，维持上一周期轮速 ----
     Vision 赛道把全黑当作路口信号而不是异常。停在路口上视觉即使识别到了
     也没法继续走，所以这里保持原有状态继续前进。

     注意这里的"不停车"与上面的黑线暂停不冲突：暂停是一次性的 0.5s，闩锁
     保证同一片黑区只停一次，停完就落到这条路径上继续走。

     注意：这一段必须在"未定义图案 → 停车"之前，因为 0x00 在 Decode 里
     同样落 default。 */
  if (pattern == LINE_PATTERN_ALL_BLACK)
  {
    line_lost_count = 0U;
    return;
  }

  /* ---- 丢线持续判定：全白且持续足够多个周期 ---- */
  if (pattern == LINE_PATTERN_ALL_WHITE)
  {
    if (line_lost_count < 0xFFFFU)
    {
      line_lost_count++;
    }
    if (line_lost_count >= LINE_LOST_COUNT)
    {
      if (line_mode != LINE_MODE_LOST)
      {
        LineTracker_EnterLost();
      }
      return;
    }
    /* 还没到判定阈值：保持上次动作，可能只是跨越白缝。 */
    return;
  }
  line_lost_count = 0U;

  /* 已判丢线后重新看到线：恢复正常循迹。 */
  if (line_mode == LINE_MODE_LOST)
  {
    printf("[LINE] line reacquired\r\n");
    line_mode = LINE_MODE_DIFF;
  }

  /* ---- 原地转进行中：边转边看传感器，压回线就退出 ---- */
  if (line_mode == LINE_MODE_PIVOT)
  {
    if ((known != 0U) && (level > -3) && (level < 3))
    {
      /* 已经压回到 2 档以内，交回差速接着走。 */
      printf("[LINE] pivot done, level=%d\r\n", (int)level);
      line_mode = LINE_MODE_DIFF;
      LineTracker_DriveDiff(level);
      return;
    }

    if ((uint32_t)(now - line_pivot_tick) >= LINE_PIVOT_TIMEOUT_MS)
    {
      printf("[LINE] pivot timeout\r\n");
      LineTracker_EnterLost();
      return;
    }

    LineTracker_DrivePivot();
    return;
  }

  /* ---- 未定义图案 ---- */
  if (known == 0U)
  {
#if LINE_STOP_ON_UNKNOWN
    Car_Stop();
#endif
    if (changed != 0U)
    {
#if LINE_DEBUG_PATTERN
      printf("[LINE] unknown pattern, stop\r\n");
#endif
    }
    return;
  }

  /* ---- 3 档极偏：切原地转；其余走差速 ---- */
  if ((level >= 3) || (level <= -3))
  {
    LineTracker_EnterPivot(level);
    LineTracker_DrivePivot();
    return;
  }

  /* ---- 偏离图案：按原有档位差速走，同时开观察窗盯住后面几个周期 ----
     0001/1000（1 档）和 0011/1100（2 档）都开窗，见 LineTracker_IsWatchPattern()。 */
  if (LineTracker_IsWatchPattern(pattern) != 0U)
  {
    LineTracker_EdgeWatchArm(level);
  }

  LineTracker_DriveDiff(level);
}

/**
  * @brief  独立循迹调试入口
  * @note   内部无限循环，绕过调度器，视觉/遥控/急停全部失效。
  *         正式集成走调度器的 CONTROL_LINE_TRACK 模式。
  */
void LineTracker_Run(void)
{
  LineTracker_Init();

  while (1)
  {
    LineTracker_Step();
    HAL_Delay(1U);
  }
}
