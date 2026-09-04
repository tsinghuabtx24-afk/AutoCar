/**
  ******************************************************************************
  * @file    ultrasonic_avoid.c
  * @brief   超声波避障（已被 avoid_task 取代，保留作独立调试入口）
  ******************************************************************************
  * @note    **正式路径不要用本模块。** 阶段 4 起超声波只做传感器
  *          （ultrasonic_sense），避障行为统一在 avoid_task，红外和超声波
  *          共用一份动作逻辑。
  *
  *          本文件保留是为了不丢失原有的独立调试入口，但它有两个已知问题：
  *            - 直写 RGB 和蜂鸣器，绕过指示层，启用时会和正式告警互相覆盖；
  *            - 内部调用阻塞式 Car_* 动作，会卡住主循环。
  ******************************************************************************
  */

#include "ultrasonic_avoid.h"
#include "ultrasonic.h"
#include "car.h"

#include <stdio.h>

static uint32_t ultrasonic_avoid_last_tick;
static uint32_t ultrasonic_avoid_random = 0x739A21D5UL;

static uint8_t UltrasonicAvoid_RandomBit(void)
{
  ultrasonic_avoid_random ^= HAL_GetTick();
  ultrasonic_avoid_random ^= ((uint32_t)Ultrasonic_GetLastMm() << 16);
  ultrasonic_avoid_random ^= ultrasonic_avoid_random << 13;
  ultrasonic_avoid_random ^= ultrasonic_avoid_random >> 17;
  ultrasonic_avoid_random ^= ultrasonic_avoid_random << 5;
  return (uint8_t)(ultrasonic_avoid_random & 1U);
}

static void UltrasonicAvoid_Alert(uint8_t enabled)
{
  if (enabled != 0U)
  {
    /* 超声波没有左右信息，两侧 RGB 同时告警。 */
    RGB_SetColor(1U, 1U, 0U);
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
  }
  else
  {
    RGB_SetColor(0U, 0U, 0U);
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
  }
}

void UltrasonicAvoid_Init(void)
{
  ultrasonic_avoid_last_tick = 0U;
  UltrasonicAvoid_Alert(0U);
}

uint8_t UltrasonicAvoid_Handle(void)
{
  uint32_t now = HAL_GetTick();
  uint16_t distance_mm;

  if ((uint32_t)(now - ultrasonic_avoid_last_tick) <
      ULTRASONIC_AVOID_SAMPLE_MS)
  {
    /* 尚未到下一次测距：保持当前默认运动，不占用底盘。 */
    return 0U;
  }
  ultrasonic_avoid_last_tick = now;

  if (Ultrasonic_ReadMm(&distance_mm) != HAL_OK)
  {
    /* 空旷时超出有效量程可能没有回波，按无障碍处理。 */
    printf("[US AVOID] timeout, treat as clear\r\n");
    UltrasonicAvoid_Alert(0U);
    return 0U;
  }

  printf("[US AVOID] distance=%umm\r\n", (unsigned)distance_mm);

  if (distance_mm == 0U || distance_mm > ULTRASONIC_AVOID_THRESHOLD_MM)
  {
    UltrasonicAvoid_Alert(0U);
    return 0U;
  }

  Car_Stop();
  UltrasonicAvoid_Alert(1U);
  printf("[US AVOID] obstacle, reverse %umm\r\n",
         (unsigned)ULTRASONIC_AVOID_REVERSE_MM);
  Car_BackwardDistance(ULTRASONIC_AVOID_SPEED,
                       ULTRASONIC_AVOID_REVERSE_MM);

  if (UltrasonicAvoid_RandomBit() != 0U)
  {
    printf("[US AVOID] rotate left 30 deg\r\n");
    Car_RotateLeftAngle(ULTRASONIC_AVOID_ROTATE_SPEED,
                        ULTRASONIC_AVOID_ROTATE_DEG10);
  }
  else
  {
    printf("[US AVOID] rotate right 30 deg\r\n");
    Car_RotateRightAngle(ULTRASONIC_AVOID_ROTATE_SPEED,
                         ULTRASONIC_AVOID_ROTATE_DEG10);
  }

  UltrasonicAvoid_Alert(0U);
  ultrasonic_avoid_last_tick = 0U;
  return 1U;
}
