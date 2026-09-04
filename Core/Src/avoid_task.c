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
/* 本次转向方向：+1 左，-1 右。 */
static int8_t   avoid_turn_dir;

const char *AvoidTask_StateName(AvoidTask_State state)
{
  switch (state)
  {
    case AVOID_BACKING: return "BACKING";
    case AVOID_TURNING: return "TURNING";
    case AVOID_VERIFY:  return "VERIFY";
    case AVOID_DONE:    return "DONE";
    case AVOID_FAILED:  return "FAILED";
    default:            return "IDLE";
  }
}

AvoidTask_State AvoidTask_GetState(void) { return avoid_state; }

void AvoidTask_Init(void)
{
  avoid_state    = AVOID_IDLE;
  avoid_attempts = 0U;
  avoid_reverses = 0U;
  avoid_turn_dir = 1;
}

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
}

/**
  * @brief  结束避障，交还控制权
  */
static uint8_t AvoidTask_Finish(uint8_t success)
{
  Car_Stop();
  Indicator_Release(INDICATOR_PRIO_AVOID);
  avoid_state = AVOID_IDLE;

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
     障碍若仍在，传感器下一轮会重新触发避障，等于自动重来。 */
  if ((uint32_t)(now - avoid_start_tick) >= AVOID_TOTAL_TIMEOUT_MS)
  {
    printf("[AVOID] total timeout after %u attempts, hand back\r\n",
           (unsigned)avoid_attempts);
    return AvoidTask_Finish(0U);
  }

  switch (avoid_state)
  {
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
