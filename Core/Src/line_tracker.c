/**
  ******************************************************************************
  * @file    line_tracker.c
  * @brief   四路红外黑线循迹实现
  ******************************************************************************
  */

#include "line_tracker.h"
#include "car.h"
#include "event.h"
#include "retarget.h"

#include <stdio.h>

/* 死区约束编译期检查：最大档差速减下来必须仍在死区之上，否则内侧轮会
   直接停转，变成急转而不是修正。改速度参数时这里会先报错，不用等上车发现。 */
_Static_assert((LINE_BASE_SPEED - 2U * LINE_DIFF_STEP) > CAR_SPEED_BASE,
               "LINE_BASE_SPEED too low for LINE_DIFF_STEP: inner wheel "
               "would fall into the motor dead zone");
_Static_assert((LINE_BASE_SPEED + 2U * LINE_DIFF_STEP) <= MOTOR_SPEED_MAX,
               "LINE_BASE_SPEED + max diff exceeds MOTOR_SPEED_MAX");

/* 循迹内部状态。原地转是一个持续若干周期的相位，需要记住方向。 */
typedef enum
{
  LINE_MODE_DIFF = 0,   /* 差速连续修正 */
  LINE_MODE_PIVOT,      /* 大偏差原地转，边转边看传感器 */
  LINE_MODE_LOST        /* 判定丢线，已制动 */
} LineTracker_Mode;

static uint8_t  line_last_pattern = 0xFFU;
static uint32_t line_last_tick;
static LineTracker_Mode line_mode;
static int8_t   line_pivot_dir;      /* +1 向左转，-1 向右转 */
static uint32_t line_pivot_tick;
static uint16_t line_lost_count;

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

void LineTracker_Reset(void)
{
  line_mode        = LINE_MODE_DIFF;
  line_pivot_dir   = 0;
  line_lost_count  = 0U;
  line_last_pattern = 0xFFU;
  line_last_tick   = 0U;   /* 立刻允许下一次控制计算 */
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

    case 0x01U:                          /* 0001：微右偏 */
    case 0x0BU: *level =  1; return 1U;  /* 1011：微右偏 */
    case 0x03U: *level =  2; return 1U;  /* 0011：轨迹右端 */
    case 0x07U: *level =  3; return 1U;  /* 0111：轨迹极右 */

    case 0x08U:                          /* 1000：微左偏 */
    case 0x0DU: *level = -1; return 1U;  /* 1101：微左偏 */
    case 0x0CU: *level = -2; return 1U;  /* 1100：轨迹左端 */
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
  int16_t diff = (int16_t)((int16_t)level * (int16_t)LINE_DIFF_STEP);
  int16_t left  = (int16_t)((int16_t)LINE_BASE_SPEED - diff);
  int16_t right = (int16_t)((int16_t)LINE_BASE_SPEED + diff);

  Car_DriveSpeed(left, right);
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

  /* ---- 丢线持续判定：全白且持续足够多个周期 ---- */
  if (pattern == 0x0FU)
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
