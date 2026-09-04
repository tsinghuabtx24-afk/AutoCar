#include "ultrasonic.h"

static uint16_t ultrasonic_last_mm;
static uint8_t ultrasonic_valid;
static uint32_t ultrasonic_cycles_per_us;

static void Ultrasonic_DwtInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0U;
  ultrasonic_cycles_per_us = SystemCoreClock / 1000000U;
  if (ultrasonic_cycles_per_us == 0U) ultrasonic_cycles_per_us = 1U;
}

static uint32_t Ultrasonic_NowUs(void) { return DWT->CYCCNT / ultrasonic_cycles_per_us; }

static void Ultrasonic_DelayUs(uint32_t us)
{
  uint32_t start = Ultrasonic_NowUs();
  while ((uint32_t)(Ultrasonic_NowUs() - start) < us) { }
}

void Ultrasonic_Init(void)
{
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
  ultrasonic_last_mm = 0U;
  ultrasonic_valid = 0U;
  Ultrasonic_DwtInit();
}

HAL_StatusTypeDef Ultrasonic_ReadMm(uint16_t *distance_mm)
{
  uint32_t start_tick;
  uint32_t start_us;
  uint32_t pulse_us;
  if (distance_mm == NULL) return HAL_ERROR;

  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
  Ultrasonic_DelayUs(2U);
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
  Ultrasonic_DelayUs(10U);
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  start_tick = HAL_GetTick();
  while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_RESET)
  {
    if ((uint32_t)(HAL_GetTick() - start_tick) >= ULTRASONIC_TIMEOUT_MS)
    {
      ultrasonic_valid = 0U;
      return HAL_TIMEOUT;
    }
  }

  start_us = Ultrasonic_NowUs();
  while (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
  {
    if ((uint32_t)(HAL_GetTick() - start_tick) >= ULTRASONIC_TIMEOUT_MS)
    {
      ultrasonic_valid = 0U;
      return HAL_TIMEOUT;
    }
  }
  pulse_us = (uint32_t)(Ultrasonic_NowUs() - start_us);
  /* 声速换算：距离(mm) = 回波时间(us) / 5.8。 */
  ultrasonic_last_mm = (uint16_t)((pulse_us * 10U) / 58U);
  ultrasonic_valid = 1U;
  *distance_mm = ultrasonic_last_mm;
  return HAL_OK;
}

uint16_t Ultrasonic_GetLastMm(void) { return ultrasonic_last_mm; }
uint8_t Ultrasonic_IsValid(void) { return ultrasonic_valid; }
