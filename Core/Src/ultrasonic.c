/**
  ******************************************************************************
  * @file    ultrasonic.c
  * @brief   HC-SR04 超声波测距实现（EXTI 双边沿 + DWT 时间戳，非阻塞）
  ******************************************************************************
  */

#include "ultrasonic.h"

/* 声速换算：距离(mm) = 回波时间(us) / 5.8。用整数写成 ×10/58。 */
#define ULTRASONIC_US_PER_MM_X10   58U

static volatile Ultrasonic_Status ultrasonic_status;
static volatile uint32_t ultrasonic_rise_cyc;   /* 上升沿的 CYCCNT 快照 */
static volatile uint32_t ultrasonic_pulse_cyc;  /* 回波宽度，单位 CPU 周期 */
static volatile uint8_t  ultrasonic_got_rise;
static volatile uint8_t  ultrasonic_got_echo;   /* 下降沿已到，结果待换算 */

static uint16_t ultrasonic_last_mm;
static uint8_t  ultrasonic_valid;
static uint32_t ultrasonic_cycles_per_us;
static uint32_t ultrasonic_start_tick;          /* 本次测距发起的 HAL tick */
static uint32_t ultrasonic_last_done_tick;      /* 上次测距结束的 HAL tick */

static void Ultrasonic_DwtInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  ultrasonic_cycles_per_us = SystemCoreClock / 1000000U;
  if (ultrasonic_cycles_per_us == 0U) ultrasonic_cycles_per_us = 1U;
}

/* 只在 TRIG 那 10us 脉冲里用。先算周期差再除，跨 CYCCNT 回绕仍正确。 */
static void Ultrasonic_DelayUs(uint32_t us)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t want  = us * ultrasonic_cycles_per_us;

  while ((uint32_t)(DWT->CYCCNT - start) < want) { }
}

/**
  * @brief  把 ECHO 引脚配成双边沿中断
  * @note   不改 gpio.c：那是 CubeMX 生成的文件，重新生成会覆盖。EXTI12 当前
  *         没有别的引脚占用（IR_IN 在 EXTI11，按键在 EXTI3/4/5），所以直接
  *         在这里重配 PF12 不会抢掉谁的中断源。
  *         EXTI15_10 的 NVIC 已由 MX_GPIO_Init 使能，无需重复。
  */
static void Ultrasonic_EchoExtiInit(void)
{
  GPIO_InitTypeDef init = {0};

  init.Pin  = ECHO_Pin;
  init.Mode = GPIO_MODE_IT_RISING_FALLING;
  init.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &init);
}

void Ultrasonic_Init(void)
{
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
  ultrasonic_last_mm        = 0U;
  ultrasonic_valid          = 0U;
  ultrasonic_status         = ULTRASONIC_IDLE;
  ultrasonic_got_rise       = 0U;
  ultrasonic_got_echo       = 0U;
  ultrasonic_pulse_cyc      = 0U;
  ultrasonic_start_tick     = 0U;
  /* 上电时让第一次测距立刻可发起，不用等一个 MIN_GAP。 */
  ultrasonic_last_done_tick = HAL_GetTick() - ULTRASONIC_MIN_GAP_MS;
  Ultrasonic_DwtInit();
  Ultrasonic_EchoExtiInit();
}

/**
  * @brief  ECHO 双边沿中断
  * @note   中断里只打时间戳，不做除法、不 printf。上升沿记起点，下降沿算差值
  *         并置标志，换算留给 Step。
  */
void Ultrasonic_EXTI_Callback(uint16_t GPIO_Pin)
{
  uint32_t now;

  if (GPIO_Pin != ECHO_Pin) return;

  now = DWT->CYCCNT;

  /* 只在测距进行中才理会边沿，杂波不会污染下一次测距。 */
  if (ultrasonic_status != ULTRASONIC_BUSY) return;

  if (HAL_GPIO_ReadPin(ECHO_GPIO_Port, ECHO_Pin) == GPIO_PIN_SET)
  {
    /* 上升沿：回波计时开始。 */
    ultrasonic_rise_cyc = now;
    ultrasonic_got_rise = 1U;
    return;
  }

  /* 下降沿：没见过上升沿就说明这一沿不属于本次测距，丢掉。 */
  if (ultrasonic_got_rise == 0U) return;

  ultrasonic_pulse_cyc = (uint32_t)(now - ultrasonic_rise_cyc);
  ultrasonic_got_rise  = 0U;
  ultrasonic_got_echo  = 1U;
}

Ultrasonic_Status Ultrasonic_Start(void)
{
  uint32_t now = HAL_GetTick();

  if (ultrasonic_status != ULTRASONIC_IDLE)
  {
    return ultrasonic_status;
  }

  /* 间隔不够就不发：上次的余波会被当成这次的回波。 */
  if ((uint32_t)(now - ultrasonic_last_done_tick) < ULTRASONIC_MIN_GAP_MS)
  {
    return ULTRASONIC_IDLE;
  }

  ultrasonic_got_rise  = 0U;
  ultrasonic_got_echo  = 0U;
  ultrasonic_pulse_cyc = 0U;
  ultrasonic_start_tick = now;
  ultrasonic_status     = ULTRASONIC_BUSY;

  /* TRIG：拉低 2us 稳定，再给 10us 高电平。HC-SR04 的时序要求，只能忙等，
     但总共 12us，可忽略。 */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
  Ultrasonic_DelayUs(2U);
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
  Ultrasonic_DelayUs(10U);
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  return ULTRASONIC_BUSY;
}

Ultrasonic_Status Ultrasonic_Step(void)
{
  uint32_t pulse_cyc;
  uint32_t pulse_us;
  uint32_t distance_mm;

  if (ultrasonic_status != ULTRASONIC_BUSY)
  {
    return ultrasonic_status;
  }

  /* ---- 回波已到：换算 ---- */
  if (ultrasonic_got_echo != 0U)
  {
    __disable_irq();
    pulse_cyc = ultrasonic_pulse_cyc;
    ultrasonic_got_echo = 0U;
    __enable_irq();

    pulse_us    = pulse_cyc / ultrasonic_cycles_per_us;
    distance_mm = (pulse_us * 10U) / ULTRASONIC_US_PER_MM_X10;

    ultrasonic_last_done_tick = HAL_GetTick();

    /* 终态直接转 IDLE：Step 把终态返回给调用方一次就够了，状态变量不留
       残余，GetStatus() 因此是无副作用的纯查询。语义与 Car_ActionStep 一致。 */
    ultrasonic_status = ULTRASONIC_IDLE;

    if ((distance_mm == 0U) || (distance_mm > ULTRASONIC_MAX_MM))
    {
      /* 超量程或零长脉冲：当作无效回波，别把鬼影当成近距离障碍。 */
      ultrasonic_valid = 0U;
      return ULTRASONIC_TIMEOUT;
    }

    ultrasonic_last_mm = (uint16_t)distance_mm;
    ultrasonic_valid   = 1U;
    return ULTRASONIC_DONE;
  }

  /* ---- 超时：ECHO 没来，或回波长得离谱 ---- */
  if ((uint32_t)(HAL_GetTick() - ultrasonic_start_tick) >= ULTRASONIC_TIMEOUT_MS)
  {
    ultrasonic_got_rise       = 0U;
    ultrasonic_got_echo       = 0U;
    ultrasonic_valid          = 0U;
    ultrasonic_last_done_tick = HAL_GetTick();
    ultrasonic_status         = ULTRASONIC_IDLE;
    return ULTRASONIC_TIMEOUT;
  }

  return ULTRASONIC_BUSY;
}

Ultrasonic_Status Ultrasonic_GetStatus(void) { return ultrasonic_status; }

uint16_t Ultrasonic_GetLastMm(void) { return ultrasonic_last_mm; }
uint8_t Ultrasonic_IsValid(void) { return ultrasonic_valid; }

/**
  * @brief  阻塞式单次测距（仅标定用）
  * @note   Start + 循环 Step 直到终态。正式主循环不要调用：它会卡住调度器
  *         最长 ULTRASONIC_TIMEOUT_MS + MIN_GAP。
  */
HAL_StatusTypeDef Ultrasonic_ReadMm(uint16_t *distance_mm)
{
  Ultrasonic_Status s;

  if (distance_mm == NULL) return HAL_ERROR;

  /* 已有测距在跑（非阻塞路径正在用）就不插队。 */
  if (ultrasonic_status != ULTRASONIC_IDLE) return HAL_BUSY;

  /* 等够 HC-SR04 的最小间隔再发，否则 Start 会拒绝。 */
  while (Ultrasonic_Start() != ULTRASONIC_BUSY) { }

  do
  {
    s = Ultrasonic_Step();
  } while (s == ULTRASONIC_BUSY);

  if (s != ULTRASONIC_DONE)
  {
    return HAL_TIMEOUT;
  }

  *distance_mm = ultrasonic_last_mm;
  return HAL_OK;
}
