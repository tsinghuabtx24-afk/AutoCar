/**
  ******************************************************************************
  * @file    indicator.c
  * @brief   蜂鸣器与 RGB 指示层实现
  ******************************************************************************
  */

#include "indicator.h"

typedef struct
{
  uint8_t         active;
  uint8_t         buzzer;
  Indicator_Color left;
  Indicator_Color right;
  uint16_t        blink_ms;

  /* 点鸣状态：已完成的"声"数与本相位起点。仅 buzzer 为 BEEPS(n) 时有意义。 */
  uint8_t         beep_phase;
  uint32_t        beep_tick;

  /* 定时申请：hold_ms 非 0 时，start_tick 起算满 hold_ms 就自动撤销。
     0 = 一直有效直到显式 Release。 */
  uint16_t        hold_ms;
  uint32_t        start_tick;
} Indicator_Slot;

static Indicator_Slot indicator_slots[INDICATOR_PRIO_COUNT];

/* 闪烁相位由本层统一维护，申请方只给周期。 */
static uint32_t indicator_blink_tick;
static uint8_t  indicator_blink_on;

/**
  * @brief  按位写一侧 RGB
  * @note   直接写引脚，不经过 main.c 的 RGB_SetColor()。
  *
  * @note   **已知硬件差异**：左侧 RGB 的 R/G 两个引脚与 main.h 里的命名相反，
  *         所以申请 INDICATOR_RED 时左侧实际亮绿色、右侧正常亮红色。表现就是
  *         避障/急停告警时"左绿右红"。
  */
static void Indicator_WriteLeft(Indicator_Color color)
{
  HAL_GPIO_WritePin(LRGB_G_GPIO_Port, LRGB_G_Pin,
                    ((color & INDICATOR_RED)   != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LRGB_R_GPIO_Port, LRGB_R_Pin,
                    ((color & INDICATOR_GREEN) != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LRGB_B_GPIO_Port, LRGB_B_Pin,
                    ((color & INDICATOR_BLUE)  != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Indicator_WriteRight(Indicator_Color color)
{
  HAL_GPIO_WritePin(RRGB_R_GPIO_Port, RRGB_R_Pin,
                    ((color & INDICATOR_RED)   != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RRGB_G_GPIO_Port, RRGB_G_Pin,
                    ((color & INDICATOR_GREEN) != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RRGB_B_GPIO_Port, RRGB_B_Pin,
                    ((color & INDICATOR_BLUE)  != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Indicator_WriteBuzzer(uint8_t on)
{
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,
                    (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Indicator_Init(void)
{
  for (uint8_t i = 0U; i < (uint8_t)INDICATOR_PRIO_COUNT; i++)
  {
    indicator_slots[i].active  = 0U;
    indicator_slots[i].hold_ms = 0U;
  }
  indicator_blink_tick = HAL_GetTick();
  indicator_blink_on   = 1U;

  Indicator_WriteBuzzer(0U);
  Indicator_WriteLeft(INDICATOR_OFF);
  Indicator_WriteRight(INDICATOR_OFF);
}

void Indicator_RequestTimed(Indicator_Prio prio, uint8_t buzzer,
                            Indicator_Color left, Indicator_Color right,
                            uint16_t blink_ms, uint16_t hold_ms)
{
  Indicator_Slot *slot;

  if ((prio == INDICATOR_PRIO_NONE) || (prio >= INDICATOR_PRIO_COUNT))
  {
    return;
  }

  slot = &indicator_slots[prio];

  /* 参数完全相同就不重启点鸣计数——否则调用方每轮主循环重复申请时，蜂鸣器会被反复重置成第一声 */
  if ((slot->active   != 0U)      && (slot->buzzer == buzzer) &&
      (slot->left     == left)    && (slot->right  == right)  &&
      (slot->blink_ms == blink_ms))
  {
    /* 定时申请例外：重复申请当作"续期"，把 hold_ms 从现在重新起算。点鸣计数仍然不动。 */
    if (hold_ms != 0U)
    {
      slot->hold_ms    = hold_ms;
      slot->start_tick = HAL_GetTick();
    }
    return;
  }

  slot->active     = 1U;
  slot->buzzer     = buzzer;
  slot->left       = left;
  slot->right      = right;
  slot->blink_ms   = blink_ms;
  slot->beep_phase = 0U;
  slot->beep_tick  = HAL_GetTick();
  slot->hold_ms    = hold_ms;
  slot->start_tick = slot->beep_tick;
}

void Indicator_Request(Indicator_Prio prio, uint8_t buzzer,
                       Indicator_Color left, Indicator_Color right,
                       uint16_t blink_ms)
{
  /* hold_ms=0：不自动撤销，行为与改动前完全一致。 */
  Indicator_RequestTimed(prio, buzzer, left, right, blink_ms, 0U);
}

void Indicator_Release(Indicator_Prio prio)
{
  if ((prio == INDICATOR_PRIO_NONE) || (prio >= INDICATOR_PRIO_COUNT))
  {
    return;
  }
  indicator_slots[prio].active = 0U;
}

void Indicator_ReleaseAll(void)
{
  for (uint8_t i = 0U; i < (uint8_t)INDICATOR_PRIO_COUNT; i++)
  {
    indicator_slots[i].active = 0U;
  }
  Indicator_WriteBuzzer(0U);
  Indicator_WriteLeft(INDICATOR_OFF);
  Indicator_WriteRight(INDICATOR_OFF);
}

/**
  * @brief  算出本轮蜂鸣器该不该响
  * @retval 1=响，0=静
  *
  * @note   三种模式：
  *           OFF        —— 恒静
  *           ON         —— 恒响，直到 Release
  *           BEEPS(n)   —— "响-停"交替 n 轮后恒静。相位编号偶数为响、
  *                         奇数为停，到 2n 就停住不再计时。
  */
static uint8_t Indicator_BuzzerNow(Indicator_Slot *slot, uint32_t now)
{
  uint8_t  beeping;
  uint32_t limit;

  if (slot->buzzer == INDICATOR_BUZZER_OFF)
  {
    return 0U;
  }
  if (slot->buzzer == INDICATOR_BUZZER_ON)
  {
    return 1U;
  }

  /* 点鸣：每声占"响 + 停"两个相位。 */
  if (slot->beep_phase >= (uint8_t)(slot->buzzer * 2U))
  {
    return 0U;   /* 叫完了，保持静音但不影响灯继续闪 */
  }

  beeping = (uint8_t)((slot->beep_phase % 2U) == 0U);
  limit   = (beeping != 0U) ? INDICATOR_BEEP_ON_MS : INDICATOR_BEEP_GAP_MS;

  if ((uint32_t)(now - slot->beep_tick) >= limit)
  {
    slot->beep_phase++;
    slot->beep_tick = now;
  }
  return beeping;
}

void Indicator_Step(void)
{
  Indicator_Slot *winner = NULL;
  uint32_t now = HAL_GetTick();
  uint8_t  show;

  /* 定时申请到期就自动撤销，调用方不必自己记时间。必须在裁决**之前**做。 */
  for (uint8_t i = 1U; i < (uint8_t)INDICATOR_PRIO_COUNT; i++)
  {
    Indicator_Slot *s = &indicator_slots[i];

    if ((s->active != 0U) && (s->hold_ms != 0U) &&
        ((uint32_t)(now - s->start_tick) >= (uint32_t)s->hold_ms))
    {
      s->active = 0U;
    }
  }

  /* 从高优先级往低找第一个有效申请。 */
  for (int8_t i = (int8_t)INDICATOR_PRIO_COUNT - 1; i > 0; i--)
  {
    if (indicator_slots[i].active != 0U)
    {
      winner = &indicator_slots[i];
      break;
    }
  }

  if (winner == NULL)
  {
    /* 没有任何申请：全部关闭。相位复位，下次申请从"亮"开始。 */
    indicator_blink_on   = 1U;
    indicator_blink_tick = now;
    Indicator_WriteBuzzer(0U);
    Indicator_WriteLeft(INDICATOR_OFF);
    Indicator_WriteRight(INDICATOR_OFF);
    return;
  }

  if (winner->blink_ms == 0U)
  {
    show = 1U;
  }
  else
  {
    if ((uint32_t)(now - indicator_blink_tick) >= (uint32_t)winner->blink_ms)
    {
      indicator_blink_tick = now;
      indicator_blink_on   = (indicator_blink_on == 0U) ? 1U : 0U;
    }
    show = indicator_blink_on;
  }

  /* 蜂鸣不跟随闪烁相位：持续告警要一直响，点鸣按自己的节奏。 */
  Indicator_WriteBuzzer(Indicator_BuzzerNow(winner, now));
  Indicator_WriteLeft((show != 0U)  ? winner->left  : INDICATOR_OFF);
  Indicator_WriteRight((show != 0U) ? winner->right : INDICATOR_OFF);
}
