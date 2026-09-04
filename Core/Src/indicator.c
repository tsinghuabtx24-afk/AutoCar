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
} Indicator_Slot;

static Indicator_Slot indicator_slots[INDICATOR_PRIO_COUNT];

/* 闪烁相位由本层统一维护，申请方只给周期。 */
static uint32_t indicator_blink_tick;
static uint8_t  indicator_blink_on;

/**
  * @brief  按位写一侧 RGB
  * @note   直接写引脚，不经过 main.c 的 RGB_SetColor()——那个函数的 r/g 参数
  *         与左右侧是交叉映射的，绕开它可以避免继续传播那个坑。
  */
static void Indicator_WriteLeft(Indicator_Color color)
{
  HAL_GPIO_WritePin(LRGB_R_GPIO_Port, LRGB_R_Pin,
                    ((color & INDICATOR_RED)   != 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LRGB_G_GPIO_Port, LRGB_G_Pin,
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
  slot->active   = 1U;
  slot->buzzer   = buzzer;
  slot->left     = left;
  slot->right    = right;
  slot->blink_ms = blink_ms;
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

void Indicator_Step(void)
{
  const Indicator_Slot *winner = NULL;
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

  /* 蜂鸣不跟随闪烁相位：告警要持续响，闪灯只是视觉提示。 */
  Indicator_WriteBuzzer(winner->buzzer);
  Indicator_WriteLeft((show != 0U)  ? winner->left  : INDICATOR_OFF);
  Indicator_WriteRight((show != 0U) ? winner->right : INDICATOR_OFF);
}
