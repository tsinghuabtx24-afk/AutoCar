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

void UltrasonicSense_Sense(void)
{
  uint32_t now = HAL_GetTick();
  uint16_t distance_mm;
  uint8_t  blocked;

  if ((uint32_t)(now - us_last_tick) < ULTRASONIC_SENSE_PERIOD_MS)
  {
    return;
  }
  us_last_tick = now;

  if (Ultrasonic_ReadMm(&distance_mm) != HAL_OK)
  {
    /* 空旷时超出量程可能没有回波，按无障碍处理。 */
    blocked = 0U;
  }
  else
  {
    us_last_mm = distance_mm;

    /* 迟滞：已判障碍时要退到 threshold+hyst 之外才算解除。 */
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
