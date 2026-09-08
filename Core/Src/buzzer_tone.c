/**
  ******************************************************************************
  * @file    buzzer_tone.c
  * @brief   蜂鸣器音调 / 旋律播放实现（TIM6 方波）
  ******************************************************************************
  */

#include "buzzer_tone.h"
#include "indicator.h"

/* TIM6 计数频率。取 1MHz：一个计数 = 1us，ARR 上限 65535 对应最低约 7.6Hz，
   覆盖全部可听音高，且分辨率对音准足够。 */
#define BTONE_TIMER_HZ    1000000U

/* 频率下限：低于这个值 ARR 会溢出 16 位，按休止处理。 */
#define BTONE_FREQ_MIN    16U

static TIM_HandleTypeDef btone_htim;

static const Buzzer_Note *btone_notes;
static uint8_t  btone_count;
static uint8_t  btone_index;
static uint32_t btone_note_tick;
static uint8_t  btone_playing;

/* 友军信号：短促上行三音 + 收尾高音。总长约 420ms。 */
const Buzzer_Note BUZZER_MELODY_FRIENDLY[] = {
  { BUZZER_NOTE_C5,  110U },
  { BUZZER_NOTE_E5,  110U },
  { BUZZER_NOTE_G5,  110U },
  { BUZZER_NOTE_REST, 30U },
  { BUZZER_NOTE_C6,  180U },
};
const uint8_t BUZZER_MELODY_FRIENDLY_LEN =
  (uint8_t)(sizeof(BUZZER_MELODY_FRIENDLY) / sizeof(BUZZER_MELODY_FRIENDLY[0]));

static void BuzzerTone_PinWrite(uint8_t on)
{
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,
                    (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief  设定当前发声频率
  * @param  freq_hz 0 或低于下限表示静音
  * @note   翻转频率是音高的两倍，所以周期取半。
  */
static void BuzzerTone_SetFreq(uint16_t freq_hz)
{
  uint32_t half_period;

  (void)HAL_TIM_Base_Stop_IT(&btone_htim);

  if (freq_hz < BTONE_FREQ_MIN)
  {
    BuzzerTone_PinWrite(0U);
    return;
  }

  half_period = BTONE_TIMER_HZ / (2U * (uint32_t)freq_hz);
  if (half_period == 0U) half_period = 1U;

  __HAL_TIM_SET_AUTORELOAD(&btone_htim, (uint32_t)(half_period - 1U));
  __HAL_TIM_SET_COUNTER(&btone_htim, 0U);
  (void)HAL_TIM_Base_Start_IT(&btone_htim);
}

void BuzzerTone_Init(void)
{
  /* TIM6 是基本定时器，只有时基。APB1 定时器时钟 = 72MHz（预分频 !=1 时 ×2）。 */
  __HAL_RCC_TIM6_CLK_ENABLE();

  btone_htim.Instance           = TIM6;
  btone_htim.Init.Prescaler     = (uint32_t)((HAL_RCC_GetPCLK1Freq() * 2U) /
                                             BTONE_TIMER_HZ) - 1U;
  btone_htim.Init.CounterMode   = TIM_COUNTERMODE_UP;
  btone_htim.Init.Period        = 1000U - 1U;   /* 播放时按音高改写 */
  btone_htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  (void)HAL_TIM_Base_Init(&btone_htim);

  /* 优先级压在 UART / 编码器之下：漏一次翻转只是音色瑕疵，不影响控制。 */
  HAL_NVIC_SetPriority(TIM6_IRQn, 3U, 0U);
  HAL_NVIC_EnableIRQ(TIM6_IRQn);

  btone_notes     = NULL;
  btone_count     = 0U;
  btone_index     = 0U;
  btone_note_tick = 0U;
  btone_playing   = 0U;
  BuzzerTone_PinWrite(0U);
}

void BuzzerTone_Play(const Buzzer_Note *notes, uint8_t count)
{
  if ((notes == NULL) || (count == 0U)) return;

  btone_notes     = notes;
  btone_count     = count;
  btone_index     = 0U;
  btone_note_tick = HAL_GetTick();
  btone_playing   = 1U;

  /* 抢下蜂鸣器：播放期间指示层不再写这个引脚。 */
  Indicator_SuspendBuzzer(1U);
  BuzzerTone_SetFreq(notes[0].freq_hz);
}

void BuzzerTone_Stop(void)
{
  (void)HAL_TIM_Base_Stop_IT(&btone_htim);
  BuzzerTone_PinWrite(0U);
  btone_playing = 0U;
  btone_notes   = NULL;
  btone_count   = 0U;
  btone_index   = 0U;
  Indicator_SuspendBuzzer(0U);
}

void BuzzerTone_Step(void)
{
  uint32_t now;

  if ((btone_playing == 0U) || (btone_notes == NULL)) return;

  now = HAL_GetTick();
  if ((uint32_t)(now - btone_note_tick) < (uint32_t)btone_notes[btone_index].ms)
  {
    return;
  }

  btone_index++;
  btone_note_tick = now;

  if (btone_index >= btone_count)
  {
    BuzzerTone_Stop();
    return;
  }
  BuzzerTone_SetFreq(btone_notes[btone_index].freq_hz);
}

uint8_t BuzzerTone_IsPlaying(void) { return btone_playing; }

/**
  * @brief  TIM6 更新中断：翻转蜂鸣器引脚
  * @note   直接定义中断入口，不经 stm32f1xx_it.c，避免 CubeMX 重新生成时冲突。
  *         向量表里的 TIM6_IRQHandler 是 weak 符号，这里的强定义会覆盖它。
  */
void TIM6_IRQHandler(void)
{
  if (__HAL_TIM_GET_FLAG(&btone_htim, TIM_FLAG_UPDATE) != RESET)
  {
    __HAL_TIM_CLEAR_IT(&btone_htim, TIM_IT_UPDATE);
    HAL_GPIO_TogglePin(Buzzer_GPIO_Port, Buzzer_Pin);
  }
}
