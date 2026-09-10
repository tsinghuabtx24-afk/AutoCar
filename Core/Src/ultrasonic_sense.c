/**
  ******************************************************************************
  * @file    ultrasonic_sense.c
  * @brief   超声波前方障碍传感器实现
  ******************************************************************************
  */

#include "ultrasonic_sense.h"
#include "ultrasonic.h"
#include "event.h"

#include <stdio.h>

/* 采样周期不够长会让 Ultrasonic_Start() 被反复拒绝，实际采样率变得不可预期。
   在这里拦住，不用等上车才发现。 */
_Static_assert(ULTRASONIC_SENSE_PERIOD_MS >= ULTRASONIC_MIN_GAP_MS,
               "ULTRASONIC_SENSE_PERIOD_MS must be >= ULTRASONIC_MIN_GAP_MS, "
               "otherwise the previous burst is mistaken for this echo");

static uint32_t us_last_tick;
static uint8_t  us_blocked;
static uint16_t us_last_mm;

void UltrasonicSense_Init(void)
{
  us_last_tick = 0U;
  us_blocked   = 0U;
  us_last_mm   = 0U;
}

void UltrasonicSense_Restart(void)
{
  us_last_tick = 0U;   /* 立刻允许下一次测距 */
  us_blocked   = 0U;
}

/**
  * @brief  推进超声波采样，只产生事件
  * @note   全程非阻塞：到周期发起测距，之后每轮推进状态机，回波到了才判阈值。
  *         不能改回阻塞版 Ultrasonic_ReadMm()——空旷无回波时卡 35ms，会把循迹
  *         的 10ms 控制周期整整拖过三轮。
  */
void UltrasonicSense_Sense(void)
{
  uint32_t now = HAL_GetTick();
  uint16_t distance_mm;
  uint8_t  blocked;

  /* 空闲：到周期就发起下一次测距 */
  if (Ultrasonic_GetStatus() == ULTRASONIC_IDLE)
  {
    if ((uint32_t)(now - us_last_tick) < ULTRASONIC_SENSE_PERIOD_MS)
    {
      return;
    }
    /* 被最小间隔拒了就下一轮再试，故意不推进 us_last_tick。 */
    if (Ultrasonic_Start() != ULTRASONIC_BUSY)
    {
      return;
    }
    us_last_tick = now;
    return;
  }

  /* 测距进行中：推进一步 */
  switch (Ultrasonic_Step())
  {
    case ULTRASONIC_BUSY:
      return;   /* 回波还没到，本轮不判 */

    case ULTRASONIC_DONE:
      distance_mm = Ultrasonic_GetLastMm();
      us_last_mm  = distance_mm;

      /* 迟滞：已判障碍时要退到 threshold+hyst 之外才解除。 */
      if (us_blocked != 0U)
      {
        blocked = (uint8_t)(distance_mm <
                            (ULTRASONIC_SENSE_THRESHOLD_MM +
                             ULTRASONIC_SENSE_HYSTERESIS_MM));
      }
      else
      {
        blocked = (uint8_t)(distance_mm < ULTRASONIC_SENSE_THRESHOLD_MM);
      }
      break;

    default:
      /* 超时/超量程：空旷时没有回波，按无障碍处理。 */
      blocked = 0U;
      break;
  }

  if (blocked != us_blocked)
  {
    us_blocked = blocked;
    printf("[US] %s at %umm\r\n",
           (blocked != 0U) ? "BLOCKED" : "clear", (unsigned)us_last_mm);
    (void)Event_Post((blocked != 0U) ? EVENT_OBSTACLE_FRONT
                                     : EVENT_OBSTACLE_CLEAR, 0U);
  }
}

uint8_t  UltrasonicSense_IsBlocked(void)  { return us_blocked; }
uint16_t UltrasonicSense_GetLastMm(void)  { return us_last_mm; }
