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
#include "vision_nav.h"

#include <stdio.h>

/* 死区约束见 line_tracker.h。基础速度运行时可变，实际钳位在 LineTracker_EffDiff()；
   这里只检查**默认值**，确保默认配置本身合理。 */
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

/* 原地转是持续若干周期的相位，所以需要记住方向。 */
typedef enum
{
  LINE_MODE_DIFF = 0,       /* 差速连续修正 */
  LINE_MODE_PIVOT,          /* 大偏差原地转，边转边看传感器 */
  LINE_MODE_VISION_TURN,    /* 视觉导航的定角旋转（vision_nav 发起） */
  LINE_MODE_LOST            /* 判定丢线，已制动 */
} LineTracker_Mode;

static uint8_t  line_last_pattern = 0xFFU;
static uint32_t line_last_tick;
static LineTracker_Mode line_mode;
static int8_t   line_pivot_dir;      /* +1 向左转，-1 向右转 */
static uint32_t line_pivot_tick;
static uint16_t line_lost_count;
/* 视觉限速/恢复时由调度器改写。 */
static uint8_t  line_base_speed = LINE_BASE_SPEED;
/* 视觉旋转的起始角度快照，用于打印实测转角供标定。 */
static int32_t  line_turn_start_deg10;

/* 偏离图案的观察窗，见 line_tracker.h。 */
static uint8_t  line_edge_watch;      /* 0=未开窗 */
static int8_t   line_edge_dir;        /* 开窗时记下的方向：+1 左转，-1 右转 */
static uint8_t  line_edge_left;       /* 观察窗剩余周期数 */
static uint8_t  line_edge_white;      /* 窗口内累计的全白次数 */

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
  * @note   死区的运行时钳位：base - 2×diff 必须 > CAR_SPEED_BASE。
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
  /* 视觉旋转途中被抢占（避障/手动接管都会 Car_ActionAbort()）时必须告知
     vision_nav 放弃本次机动，否则它永远停在 TURN_*_RUN 等一个再也不会来的
     OnTurnDone()，视觉门此后再也打不开。放弃整个机动而不是"接着转"：被抢占后
     车身姿态已变，第二段反向转会把车带偏。 */
  if (line_mode == LINE_MODE_VISION_TURN)
  {
    VisionNav_Reset();
  }

  LineTracker_EdgeWatchCancel();

  line_mode        = LINE_MODE_DIFF;
  line_pivot_dir   = 0;
  line_lost_count  = 0U;
  line_last_pattern = 0xFFU;
  line_last_tick   = 0U;   /* 立刻允许下一次控制计算 */
  /* 基础速度**不重置**：限速是持续配置，被避障打断后应保持，由视觉的
     "解除限速"事件恢复。 */
}

void LineTracker_Init(void)
{
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
  * @note   这套图案→方向的映射是实车验证过的，不要改。本函数只给出"偏差几档"，
  *         用差速还是原地转由调用方决定。
  */
static uint8_t LineTracker_Decode(uint8_t pattern, int8_t *level)
{
  switch (pattern)
  {
    /* 标 (窗) 的四个先按此档差速，同时开观察窗，可能改判原地转。 */
    case 0x09U: *level =  0; return 1U;  /* 1001：轨迹中心 */

    case 0x01U: *level =  1; return 1U;  /* 0001：微右偏 (窗) */
    case 0x0BU: *level =  1; return 1U;  /* 1011：微右偏 */
    case 0x03U: *level =  2; return 1U;  /* 0011：轨迹右端 (窗) */
    case 0x07U: *level =  3; return 1U;  /* 0111：轨迹极右 */

    case 0x08U: *level = -1; return 1U;  /* 1000：微左偏 (窗) */
    case 0x0DU: *level = -1; return 1U;  /* 1101：微左偏 */
    case 0x0CU: *level = -2; return 1U;  /* 1100：轨迹左端 (窗) */
    case 0x0EU: *level = -3; return 1U;  /* 1110：轨迹极左 */

    default:    *level =  0; return 0U;
  }
}

/**
  * @brief  差速修正：车持续前进，靠左右轮速差纠正航向
  * @param  level: 偏差档位，正=向左修正（左轮减速、右轮加速）
  */
static void LineTracker_DriveDiff(int8_t level)
{
  int16_t step  = (int16_t)LineTracker_EffDiff();
  int16_t diff  = (int16_t)((int16_t)level * step);
  int16_t left  = (int16_t)((int16_t)line_base_speed - diff);
  int16_t right = (int16_t)((int16_t)line_base_speed + diff);

  Car_DriveSpeed(left, right);
}

/** @brief 关掉边缘观察窗 */
static void LineTracker_EdgeWatchCancel(void)
{
  line_edge_watch = 0U;
  line_edge_dir   = 0;
  line_edge_left  = 0U;
  line_edge_white = 0U;
}

/** @brief 这个图案要开观察窗吗（只有含义有歧义的那四个要） */
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
  * @note   同方向的窗口已开着就**不重开**：偏离图案通常连着好几个周期都在，每次
  *         重置计数的话窗口永远走不完、全白次数也永远凑不满。这也让 0011→0001
  *         这种"越偏越多"的序列共用一个窗口，不会因图案变了就丢掉已数到的全白。
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
      /* 翻案：不是微偏而是弯道。方向必须用开窗时记下的那个——此刻四路全白，
         当前图案给不出方向。 */
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
  * @note   故意不用 Car_StartRotateAngle()：那是"转到指定角度就停"，循迹要的是
  *         "转到压回线就停"。后者是传感器闭环，比按角度开环准，也不依赖
  *         CAR_ROT_COMP_* 补偿表。瞬时接口 + 每周期判退出，同样非阻塞。
  */
static void LineTracker_EnterPivot(int8_t level)
{
  LineTracker_EdgeWatchCancel();   /* 已经在转了，窗口没意义 */

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

/** @brief 判定丢线并制动 */
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

  /* 固定控制周期：未到就维持上次轮速返回。主循环每轮几十微秒，不把住周期的话
     修正会一次接一次连着来，激进得多。 */
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

  /* 把图案告知视觉导航（只读，不影响下面的循迹判断）：全黑在 ARMED 下开启视觉，
     0000/0001/1000 在 WAIT 下构成旋转触发。 */
  VisionNav_OnPattern(pattern);
  VisionNav_Step();

  /* 视觉旋转进行中：推进定角动作，转完交回差速 */
  if (line_mode == LINE_MODE_VISION_TURN)
  {
    Car_ActionStatus st = Car_ActionStep();

    if (st == CAR_ACTION_BUSY)
    {
      return;
    }

    /* DONE/TIMEOUT/FAILED 都算旋转结束（car 已制动）。实测转角报给 vision_nav
       打印，供标定 VNAV_TURN_DEG10。 */
    VisionNav_OnTurnDone(Encoder_GetRotationDeg10(CAR_WHEEL_TRACK_MM) -
                         line_turn_start_deg10);
    if (st != CAR_ACTION_DONE)
    {
      printf("[LINE] vision turn ended abnormally (%d)\r\n", (int)st);
    }
    line_mode = LINE_MODE_DIFF;
    return;
  }

  /* 视觉导航要求旋转：让位给定角动作。放在丢线判定之前——旋转触发图案含全黑，
     而全黑不参与丢线计数，两者不冲突。 */
  {
    int32_t turn_deg10 = 0;

    if ((line_mode != LINE_MODE_LOST) &&
        (VisionNav_TakeTurnRequest(&turn_deg10) != 0U))
    {
      Car_ActionAbort();   /* 循迹用的是瞬时接口，这里确保没有残留动作 */
      if (Car_StartRotateAngle(VNAV_TURN_SPEED, turn_deg10) == CAR_ACTION_BUSY)
      {
        line_turn_start_deg10 = Encoder_GetRotationDeg10(CAR_WHEEL_TRACK_MM);
        /* 观察窗直接作废：旋转期间 Step 提前 return 推不动它，而转完车身姿态已变，
           窗口的前提（正在跟线做微偏修正）不再成立，留着会拿旧方向误判一次原地转。 */
        LineTracker_EdgeWatchCancel();
        line_mode = LINE_MODE_VISION_TURN;
        return;
      }
      /* 发起失败：告知 vision_nav 结束这一相，避免它卡在 RUN 状态。 */
      printf("[LINE] vision turn rejected\r\n");
      VisionNav_OnTurnDone(0);
    }
  }

  /* 边缘观察窗：数全白，够多就把之前的"微偏"改判成原地转。必须放在全白判定
     **之前**——全白那一段会 return，放后面就永远数不到它要数的图案。只在 DIFF
     下改判：已在 pivot 里没必要重来，LOST 时该做的是重新找线而不是转。 */
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

  /* 全黑：不停车、不判丢线，维持上一周期轮速（理由见 line_tracker.h）。
     必须在"未定义图案 → 停车"之前，因为 0x00 在 Decode 里同样落 default。 */
  if (pattern == LINE_PATTERN_ALL_BLACK)
  {
    line_lost_count = 0U;
    return;
  }

  /* 丢线持续判定：全白且持续足够多个周期 */
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

  /* 原地转进行中：边转边看传感器，压回线就退出 */
  if (line_mode == LINE_MODE_PIVOT)
  {
    if ((known != 0U) && (level > -3) && (level < 3))
    {
      /* 压回到 2 档以内，交回差速接着走。 */
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

  /* 未定义图案 */
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

  /* 3 档极偏切原地转，其余走差速 */
  if ((level >= 3) || (level <= -3))
  {
    LineTracker_EnterPivot(level);
    LineTracker_DrivePivot();
    return;
  }

  /* 偏离图案：按档位差速走，同时开观察窗盯住后面几个周期 */
  if (LineTracker_IsWatchPattern(pattern) != 0U)
  {
    LineTracker_EdgeWatchArm(level);
  }

  LineTracker_DriveDiff(level);
}

/**
  * @brief  独立循迹调试入口
  * @note   内部无限循环，绕过调度器，视觉/遥控/急停**全部失效**。正式集成走
  *         调度器的 CONTROL_LINE_TRACK 模式。
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
