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

_Static_assert(LINE_BACKUP_MODE <= 2U,
               "LINE_BACKUP_MODE must be 0 (off), 1 (by distance) or "
               "2 (by time)");
_Static_assert(LINE_BACKUP_ON_LEVEL3 <= 1U,
               "LINE_BACKUP_ON_LEVEL3 must be 0 or 1");
#if LINE_BACKUP_MODE != 0U
/* 退不动就等于没退，而且 car 会直接返回 FAILED。 */
_Static_assert(LINE_BACKUP_SPEED > CAR_SPEED_BASE,
               "LINE_BACKUP_SPEED is inside the motor dead zone: the car "
               "would not move at all");
_Static_assert(LINE_BACKUP_SPEED <= MOTOR_SPEED_MAX,
               "LINE_BACKUP_SPEED exceeds MOTOR_SPEED_MAX");
#endif
#if LINE_BACKUP_MODE == 1U
/* 距离为 0 时 car 判参数非法。 */
_Static_assert(LINE_BACKUP_DIST_MM >= 1U,
               "LINE_BACKUP_DIST_MM must be at least 1mm");
#elif LINE_BACKUP_MODE == 2U
_Static_assert(LINE_BACKUP_TIME_MS >= 1U,
               "LINE_BACKUP_TIME_MS must be at least 1ms");
_Static_assert(LINE_BACKUP_TIME_MS < LINE_BACKUP_GUARD_MS,
               "LINE_BACKUP_GUARD_MS must exceed LINE_BACKUP_TIME_MS or the "
               "watchdog would fire before the backup can finish");
#endif

/* 循迹内部状态。原地转是一个持续若干周期的相位，需要记住方向。 */
typedef enum
{
  LINE_MODE_DIFF = 0,       /* 差速连续修正 */
  LINE_MODE_BACKUP,         /* 空窗改判后的后退，退完接 PIVOT */
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
/* 基础速度。默认为 LINE_BASE_SPEED，视觉限速/恢复时由调度器改写。 */
static uint8_t  line_base_speed = LINE_BASE_SPEED;
/* 视觉旋转的起始编码器角度快照，用于打印实测转角供标定。 */
static int32_t  line_turn_start_deg10;

/* ---- 空窗改判后的后退相位 ----
   后退期间四路仍然全白，方向没法从当前图案再算一次，所以发起后退时就把
   "退完要往哪转"记下来。 */
static int8_t   line_backup_dir;      /* +1 退完向左转，-1 向右转 */
static uint32_t line_backup_tick;     /* 后退起始 tick，喂看门狗 */
/* 上一次后退**结束**的 tick，喂 LINE_BACKUP_COOLDOWN_MS。被冷却挡掉的那次
   不更新它，否则持续振荡会把冷却无限续下去。 */
static uint32_t line_backup_done_tick;

/* ---- 偏离图案（0001/1000/0011/1100）的观察窗 ----
   见 line_tracker.h。看到这些图案时开窗，窗口内数全白次数，够多就改判原地转。 */
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
  /* 若在视觉旋转途中被抢占（避障/手动接管都会 Car_ActionAbort()），必须告知
     vision_nav 放弃本次机动。否则它永远停在 TURN_*_RUN 等一个再也不会来的
     OnTurnDone()，视觉门此后再也不会打开。
     选择放弃整个机动而不是"接着转"：被抢占后车身姿态已变，第二段反向转会
     把车带偏——与急停时的处理一致。 */
  if (line_mode == LINE_MODE_VISION_TURN)
  {
    VisionNav_Reset();
  }

  /* 后退相位途中被抢占：必须显式放弃 car 的动作，否则状态机还被占着，
     后面 Car_StartRotateAngle()/Car_StartBackwardDistance() 都会被拒。
     抢占方通常已经 Car_ActionAbort() 过，这里再来一次是幂等的。 */
  if (line_mode == LINE_MODE_BACKUP)
  {
    Car_ActionAbort();
  }

  LineTracker_EdgeWatchCancel();

  /* 冷却置为"已过期"，让恢复控制权后的第一次原地转能正常后退。不这样做的话
     开机后头 LINE_BACKUP_COOLDOWN_MS 内的急弯会被冷却白挡一次。
     无符号回绕算术，HAL_GetTick() 小于冷却值时结果依然正确。 */
  line_backup_done_tick =
      HAL_GetTick() - (uint32_t)LINE_BACKUP_COOLDOWN_MS;

  line_mode        = LINE_MODE_DIFF;
  line_backup_dir  = 0;
  line_pivot_dir   = 0;
  line_lost_count  = 0U;
  line_last_pattern = 0xFFU;
  line_last_tick   = 0U;   /* 立刻允许下一次控制计算 */
  /* 基础速度**不重置**：限速是持续配置，被避障打断后应该保持，
     由视觉的"解除限速"事件恢复。 */
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
  * @brief  空窗改判后先退一段，退完再原地转
  * @param  level: 改判给出的档位，符号即退完要转的方向
  *
  * @note   与 pivot 不同，后退用的是 car 的**非阻塞动作状态机**而不是瞬时接口：
  *         要的是"退够 20mm/100ms 就停"这种有终点的动作，瞬时接口没有终点。
  *         代价是后退期间要独占 car 的动作状态机，所以先 Abort 一次。
  *
  *         退不动不是致命错误：发起失败就退化成原来的"立即转"，不能因为退不了
  *         就放弃这次改判——那会让急弯彻底没人管。
  */
static void LineTracker_EnterBackupThenPivot(int8_t level)
{
#if LINE_BACKUP_MODE == 0U
  /* 后退关闭：保持旧行为，方便和开启后做 A/B 对比。 */
  LineTracker_EnterPivot(level);
  LineTracker_DrivePivot();
#else
  int8_t           dir = (level > 0) ? 1 : -1;
  Car_ActionStatus st;

  /* ---- 冷却检查：距上次后退太近就跳过后退，直接转 ----
     原地转**不产生前进位移**，所以"退一段→转→又偏到 3 档→再退一段"这个振荡
     每轮净往后走一次后退的距离，车会一路倒着退出赛道。冷却让振荡最多只吃到
     第一次后退。挡掉的这次不刷新 line_backup_done_tick，否则冷却会被无限续。 */
  if ((uint32_t)(HAL_GetTick() - line_backup_done_tick) <
      (uint32_t)LINE_BACKUP_COOLDOWN_MS)
  {
    printf("[LINE] backup on cooldown, pivot directly\r\n");
    LineTracker_EnterPivot(level);
    LineTracker_DrivePivot();
    return;
  }

  /* 已经在转的话观察窗没意义了；顺手清掉，和 EnterPivot 保持一致。 */
  LineTracker_EdgeWatchCancel();

  /* 循迹平时只用瞬时接口，动作状态机理论上是空的。但视觉旋转刚异常结束等
     情况下可能有残留，不 Abort 会让下面的 Start 直接被拒。 */
  Car_ActionAbort();

#if LINE_BACKUP_MODE == 1U
  st = Car_StartBackwardDistance(LINE_BACKUP_SPEED, LINE_BACKUP_DIST_MM);
#else
  st = Car_StartTimed(-(int16_t)LINE_BACKUP_SPEED,
                      -(int16_t)LINE_BACKUP_SPEED, LINE_BACKUP_TIME_MS);
#endif

  if (st != CAR_ACTION_BUSY)
  {
    printf("[LINE] backup rejected (%d), pivot directly\r\n", (int)st);
    LineTracker_EnterPivot(level);
    LineTracker_DrivePivot();
    return;
  }

  line_backup_dir  = dir;
  line_backup_tick = HAL_GetTick();
  line_mode        = LINE_MODE_BACKUP;
  printf("[LINE] backup then pivot %s\r\n", (dir > 0) ? "left" : "right");
#endif
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

  /* 把图案告知视觉导航（只读，不影响下面的循迹判断）。全黑在 ARMED 状态下
     会开启视觉；0000/0001/1000 在 WAIT 状态下会构成旋转触发。 */
  VisionNav_OnPattern(pattern);
  VisionNav_Step();

  /* ---- 视觉旋转进行中：推进定角动作，转完交回差速 ---- */
  if (line_mode == LINE_MODE_VISION_TURN)
  {
    Car_ActionStatus st = Car_ActionStep();

    if (st == CAR_ACTION_BUSY)
    {
      return;
    }

    /* DONE / TIMEOUT / FAILED 都算这次旋转结束（car 已制动）。把实测转角
       报给 vision_nav 打印，供标定 VNAV_TURN_DEG10。 */
    VisionNav_OnTurnDone(Encoder_GetRotationDeg10(CAR_WHEEL_TRACK_MM) -
                         line_turn_start_deg10);
    if (st != CAR_ACTION_DONE)
    {
      printf("[LINE] vision turn ended abnormally (%d)\r\n", (int)st);
    }
    line_mode = LINE_MODE_DIFF;
    return;
  }

  /* ---- 后退相位进行中：退完接原地转 ----
     放在视觉旋转之后、视觉旋转请求之前。后退只有 100ms 量级，让它走完比被视觉
     抢断再重来更简单，也保证 line_backup_dir 一定有人消费、不会悬着。 */
  if (line_mode == LINE_MODE_BACKUP)
  {
    Car_ActionStatus st = Car_ActionStep();

    if (st == CAR_ACTION_BUSY)
    {
      if ((uint32_t)(now - line_backup_tick) < LINE_BACKUP_GUARD_MS)
      {
        return;
      }
      /* 轮子被卡住之类：不能一直等 car 那个 60s 的兜底超时。 */
      printf("[LINE] backup guard timeout, pivot anyway\r\n");
      Car_ActionAbort();
    }
    else if (st != CAR_ACTION_DONE)
    {
      printf("[LINE] backup ended abnormally (%d)\r\n", (int)st);
    }

    /* 后退到此结束，冷却从这一刻起算。 */
    line_backup_done_tick = now;

    /* 退得好不好都要转：后退是为了提高成功率，不是原地转的前置条件。
       PIVOT 的超时也从这一刻重新起算，后退耗时不占额度。 */
    LineTracker_EnterPivot((int8_t)(line_backup_dir * 3));
    LineTracker_DrivePivot();
    return;
  }

  /* ---- 视觉导航要求旋转：让位给定角动作 ----
     放在丢线判定之前：旋转触发图案里包含全黑，而全黑不参与丢线计数，
     两者不冲突。 */
  {
    int32_t turn_deg10 = 0;

    if ((line_mode != LINE_MODE_LOST) &&
        (VisionNav_TakeTurnRequest(&turn_deg10) != 0U))
    {
      Car_ActionAbort();   /* 循迹用的是瞬时接口，这里确保没有残留动作 */
      if (Car_StartRotateAngle(VNAV_TURN_SPEED, turn_deg10) == CAR_ACTION_BUSY)
      {
        line_turn_start_deg10 = Encoder_GetRotationDeg10(CAR_WHEEL_TRACK_MM);
        /* 视觉旋转期间 Step 会提前 return，观察窗推不动。转完车身姿态已经变了，
           窗口的前提（正在跟线做微偏修正）不再成立，直接作废，免得旋转结束后
           拿着旧方向误判一次原地转。 */
        LineTracker_EdgeWatchCancel();
        line_mode = LINE_MODE_VISION_TURN;
        return;
      }
      /* 发起失败：告知 vision_nav 结束这一相，避免它卡在 RUN 状态。 */
      printf("[LINE] vision turn rejected\r\n");
      VisionNav_OnTurnDone(0);
    }
  }

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
      /* 改判这条路径先退再转：走到这里说明车已经冲过弯道入口，转轴在线外。
         3 档极偏（下面那条路径）不走后退，它本来还压着线。 */
      LineTracker_EnterBackupThenPivot(esc_level);
      return;
    }
  }

  /* ---- 全黑：不停车、不判丢线，维持上一周期轮速 ----
     Vision 赛道把全黑当作路口信号而不是异常。停在路口上视觉即使识别到了
     也没法继续走，所以这里保持原有状态继续前进。
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
#if LINE_BACKUP_ON_LEVEL3
    /* 与观察窗改判共用同一套后退逻辑（含冷却与看门狗）。0111/1110 虽然还压着
       最外侧一路，但走到 3 档说明车身已经斜得很厉害，退一段再转同样更容易
       把旋转中心拉回线上。 */
    LineTracker_EnterBackupThenPivot(level);
#else
    LineTracker_EnterPivot(level);
    LineTracker_DrivePivot();
#endif
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
