#include "line_tracker.h"
#include "car.h"
#include "retarget.h"

#include <stdio.h>

static uint8_t line_last_pattern = 0xFFU;

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

void LineTracker_Init(void)
{
  line_last_pattern = 0xFFU;
  Car_Stop();
  printf("[LINE] init S1..S4: black=0 white=1\r\n");
}

void LineTracker_Step(void)
{
  uint8_t pattern = LineTracker_ReadPattern();
  uint8_t changed = (pattern != line_last_pattern) ? 1U : 0U;

  if (changed != 0U)
  {
    line_last_pattern = pattern;
    printf("[LINE] pattern=%u%u%u%u (0x%02X)\r\n",
           (pattern >> 3) & 1U,
           (pattern >> 2) & 1U,
           (pattern >> 1) & 1U,
           pattern & 1U,
           (unsigned)pattern);
  }

  switch (pattern)
  {
    case 0x09U: /* 1001：轨迹中心，直行 */
      Car_ForwardRun(LINE_FORWARD_SPEED);
      break;
    case 0x01U: /* 0001：按约定微右偏，向左修正 5° */
    case 0x0BU: /* 1011：按约定微右偏，向左修正 5° */
      Car_RotateLeftAngle(LINE_FORWARD_SPEED, LINE_CORRECT_ANGLE_DEG10);
      break;
    case 0x03U: /* 0011：轨迹右端，向左修正 10° */
      Car_RotateLeftAngle(LINE_FORWARD_SPEED, 2 * LINE_CORRECT_ANGLE_DEG10);
      break;
    case 0x08U: /* 1000：按约定微左偏，向右修正 5° */
    case 0x0DU: /* 1101：按约定微左偏，向右修正 5° */
      Car_RotateRightAngle(LINE_FORWARD_SPEED, LINE_CORRECT_ANGLE_DEG10);
      break;
    case 0x0CU: /* 1100：轨迹左端，向右修正 10° */
      Car_RotateRightAngle(LINE_FORWARD_SPEED, 2 * LINE_CORRECT_ANGLE_DEG10);
      break;
    default:
#if LINE_STOP_ON_UNKNOWN
      Car_Stop();
#endif
      if (changed != 0U)
      {
        printf("[LINE] unknown pattern, stop\r\n");
      }
      break;
  }
}

void LineTracker_Run(void)
{
  LineTracker_Init();

  while (1)
  {
    LineTracker_Step();
    HAL_Delay(LINE_SAMPLE_PERIOD_MS);
  }
}
