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
} Indicator_Slot;

static Indicator_Slot indicator_slots[INDICATOR_PRIO_COUNT];

/* 闪烁相位由本层统一维护，申请方只给周期。 */
static uint32_t indicator_blink_tick;
static uint8_t  indicator_blink_on;

/**
  * @brief  按位写一侧 RGB
  * @note   直接写引脚，不经过 main.c 的 RGB_SetColor()——那个函数的 r/g 参数
  *         与左右侧是交叉映射的，绕开它可以避免继续传播那个坑。
  *
  * @note   **已知硬件差异**：左侧 RGB 的 R/G 两个引脚与 main.h 里的命名相反，
  *         所以申请 INDICATOR_RED 时左侧实际亮绿色、右侧正常亮红色。表现就是
  *         避障/急停告警时"左绿右红"。
  *
  *         原 RGB_SetColor() 是靠交叉传参掩盖这一点的（左侧 R 引脚喂 g 参数）。
  *         这里按引脚名直写，所以差异暴露出来了。属于接线问题，不是逻辑错误，
  *         已经在软件里补偿。
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

/* 置 1 时本层不碰蜂鸣器引脚，由 buzzer_tone 独占（见 Indicator_SuspendBuzzer）。 */
static uint8_t indicator_buzzer_suspended;

static void Indicator_WriteBuzzer(uint8_t on)
{
  if (indicator_buzzer_suspended != 0U) return;

  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,
                    (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void Indicator_SuspendBuzzer(uint8_t suspend)
{
  indicator_buzzer_suspended = suspend;
}

void Indicator_Init(void)
{
  for (uint8_t i = 0U; i < (uint8_t)INDICATOR_PRIO_COUNT; i++)
  {
    indicator_slots[i].active = 0U;
  }
  indicator_blink_tick = HAL_GetTick();
  indicator_blink_on   = 1U;

  Indicator_WriteBuzzer(0U);
  Indicator_WriteLeft(INDICATOR_OFF);
  Indicator_WriteRight(INDICATOR_OFF);
}

void Indicator_Request(Indicator_Prio prio, uint8_t buzzer,
                       Indicator_Color left, Indicator_Color right,
                       uint16_t blink_ms)
{
  Indicator_Slot *slot;

  if ((prio == INDICATOR_PRIO_NONE) || (prio >= INDICATOR_PRIO_COUNT))
  {
    return;
  }

  slot = &indicator_slots[prio];

  /* 参数完全相同就只是"续订"，不重启点鸣计数——否则调用方每轮主循环重复
     申请时，蜂鸣器会被反复重置成第一声，变成一直响。 */
  if ((slot->active   != 0U)      && (slot->buzzer == buzzer) &&
      (slot->left     == left)    && (slot->right  == right)  &&
      (slot->blink_ms == blink_ms))
  {
    return;
  }

  slot->active     = 1U;
  slot->buzzer     = buzzer;
  slot->left       = left;
  slot->right      = right;
  slot->blink_ms   = blink_ms;
  slot->beep_phase = 0U;
  slot->beep_tick  = HAL_GetTick();
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
