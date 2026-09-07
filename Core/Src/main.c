/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "car.h"
#include "encoder.h"
#include "ir_avoid.h"
#include "line_tracker.h"
#include "retarget.h"
#include "oled.h"
#include "ir_remote.h"
#include "ultrasonic.h"
#include "ir_remote_debug.h"
#include "ultrasonic_avoid.h"
#include "vision_uart.h"
#include "vision_task.h"
#include "event.h"
#include "scheduler.h"
#include "manual_task.h"
#include "avoid_task.h"
#include "indicator.h"
#include "ultrasonic_sense.h"

#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* RGB LED timing: 0.2s on, 0.2s off */
#define RGB_PERIOD_MS  200U

/* 轨迹间的停顿时间 */
#define TRAJ_GAP_MS    2000U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Buzzer control flag */
volatile uint8_t buzzer_on = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  Set RGB LED color
  * @param  r: Red state (1 = on, 0 = off)
  * @param  g: Green state (1 = on, 0 = off)
  * @param  b: Blue state (1 = on, 0 = off)
  * @retval None
  */
void RGB_SetColor(uint8_t r, uint8_t g, uint8_t b)
{
  HAL_GPIO_WritePin(RRGB_R_GPIO_Port, RRGB_R_Pin, r ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RRGB_G_GPIO_Port, RRGB_G_Pin, g ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(RRGB_B_GPIO_Port, RRGB_B_Pin, b ? GPIO_PIN_SET : GPIO_PIN_RESET);


  HAL_GPIO_WritePin(LRGB_R_GPIO_Port, LRGB_R_Pin, g ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LRGB_G_GPIO_Port, LRGB_G_Pin, r ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LRGB_B_GPIO_Port, LRGB_B_Pin, b ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief  EXTI line detection callback
  * @param  GPIO_Pin: Specifies the pins connected EXTI line
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  IrRemote_EXTI_Callback(GPIO_Pin);
  /* 超声波 ECHO 双边沿：中断里只打 DWT 时间戳，换算留给 Ultrasonic_Step()。 */
  Ultrasonic_EXTI_Callback(GPIO_Pin);

  if (GPIO_Pin == key1_Pin || GPIO_Pin == key3_Pin)
  {
    /* Key1 or Key3 pressed: turn on buzzer */
    buzzer_on = 1;
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);

    /* Key3 兼作急停。中断里只投事件，不做动作。 */
    if (GPIO_Pin == key3_Pin)
    {
      (void)Event_Post(EVENT_KEY_STOP, 0U);
    }
  }
  else if (GPIO_Pin == key2_Pin)
  {
    /* Key2 pressed: turn off buzzer */
    buzzer_on = 0;
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);

    /* Key2 兼作启动。 */
    (void)Event_Post(EVENT_KEY_START, 0U);
  }
}

/* 电机与小车运动函数分别位于 tim.c 和 car.c */

/**
  * @brief  周期输出四个轮子的编码器诊断数据
  * @note   放在主循环中调用，不能放到 SysTick 中断里，否则阻塞式串口发送
  *         会影响系统时基和编码器采样。
  */
static void Encoder_PrintStatus(void)
{
  static uint32_t last_report_tick = 0U;
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - last_report_tick) < 500U)
  {
    return;
  }
  last_report_tick = now;

  printf("[ENC %lums] M1: count=%ld distance=%ldmm rpm=%d\r\n",
         (unsigned long)now,
         (long)Encoder_GetCount(MOTOR_1),
         (long)Encoder_GetDistance(MOTOR_1),
         (int)Encoder_GetSpeedRpm(MOTOR_1));
  printf("             M2: count=%ld distance=%ldmm rpm=%d\r\n",
         (long)Encoder_GetCount(MOTOR_2),
         (long)Encoder_GetDistance(MOTOR_2),
         (int)Encoder_GetSpeedRpm(MOTOR_2));
  printf("             M3: count=%ld distance=%ldmm rpm=%d\r\n",
         (long)Encoder_GetCount(MOTOR_3),
         (long)Encoder_GetDistance(MOTOR_3),
         (int)Encoder_GetSpeedRpm(MOTOR_3));
  printf("             M4: count=%ld distance=%ldmm rpm=%d\r\n",
         (long)Encoder_GetCount(MOTOR_4),
         (long)Encoder_GetDistance(MOTOR_4),
         (int)Encoder_GetSpeedRpm(MOTOR_4));
  printf("             avg=%ldmm left=%ldmm right=%ldmm\r\n",
         (long)Encoder_GetDistanceAvg(),
         (long)Encoder_GetDistanceLeft(),
         (long)Encoder_GetDistanceRight());
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_ADC3_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* Initialize LED2 as on (buzzer is off by default) */
  HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);

  /* printf → USART1(PA9)。必须在 MX_USART1_UART_Init() 之后 */
  Retarget_Init();
  printf("\r\n=== AutoCar boot ===\r\n");
  printf("encoder: %u counts/rev (PPR %u x%u, gear %u), wheel %umm\r\n",
         (unsigned)ENC_COUNTS_PER_REV, (unsigned)ENC_PPR,
         (unsigned)ENC_QUAD_FACTOR, (unsigned)ENC_GEAR_RATIO,
         (unsigned)ENC_WHEEL_DIA_MM);

  /* 启动 8 路电机 PWM，初始为静止 */
  Car_Init();

  /* 启动四路编码器。之后由 SysTick 每 1ms 自动采样，无需在主循环里调用。
     标定用的观察量（调试器里加 Watch）：
       Encoder_GetCount(MOTOR_1)   四路计数，前进时应全为正
       Encoder_GetDistanceAvg()    车体里程 mm
       Encoder_GetSpeedRpm(...)    轮子转速 rpm */
    Encoder_Init();
  IrAvoid_Init();
    Oled_Init();
  IrRemote_Init();
  IrRemoteDebug_Init();
  Ultrasonic_Init();
  UltrasonicAvoid_Init();
  /* 事件队列必须在任何可能投递事件的模块之前初始化。 */
  Event_Init();
  Indicator_Init();
  VisionUart_Init();
  VisionTask_Init();
  ManualTask_Init();
  UltrasonicSense_Init();
  AvoidTask_Init();
  LineTracker_Init();
  Scheduler_Init();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



    /* ==== 正式主循环：采样 → 调度 → 分派 → 诊断 ====================
       循环里没有 HAL_Delay，每轮都能重新仲裁优先级。同一周期只有
       Scheduler_Dispatch() 选中的那一个模块会写电机。 */

    /* 1. 采样层：识别模块只产生事件，不驱动底盘。
          遥控必须每轮采样——RED 双击是进入手动模式的唯一入口。
          视觉帧由 UART2 中断直接投递事件，这里无需轮询。 */
    ManualTask_Sense();
    // IrAvoid_Sense();
    UltrasonicSense_Sense();

    /* 2. 调度层：消费事件，决定控制模式。 */
    Scheduler_DrainEvents();

    /* 3. 分派：本轮只有一个模块获得底盘控制权。 */
    Scheduler_Dispatch();

    /* 4. 指示层：按优先级裁决蜂鸣与 RGB，各模块只申请不直写。 */
    Indicator_Step();

    /* 5. 诊断：按 tick 判周期，绝不每轮都 printf。 */
    {
      static uint32_t last_vision_debug_tick;
      uint32_t now = HAL_GetTick();
      if ((uint32_t)(now - last_vision_debug_tick) >= 1000U)
      {
        last_vision_debug_tick = now;
        printf("[DIAG] mode=%s speed=%u ir=%s front=%umm | RX count=%lu "
               "last=0x%02X errors=%lu resync=%lu code=0x%08lX | "
               "REMOTE dropped=%lu | EVT posted=%lu dropped=%lu | "
               "AVOID %s step=%u\r\n",
               Scheduler_ModeName(Scheduler_GetMode()),
               (unsigned)VisionTask_GetSpeed(),
               IrAvoid_StateName(IrAvoid_GetState()),
               (unsigned)UltrasonicSense_GetLastMm(),
               (unsigned long)VisionUart_GetRxCount(),
               (unsigned)VisionUart_GetLastByte(),
               (unsigned long)VisionUart_GetErrorCount(),
               (unsigned long)VisionUart_GetResyncCount(),
               (unsigned long)VisionUart_GetLastError(),
               (unsigned long)IrRemote_GetDroppedCount(),
               (unsigned long)Event_GetPostedCount(),
               (unsigned long)Event_GetDroppedCount(),
               AvoidTask_StateName(AvoidTask_GetState()),
               (unsigned)AvoidTask_GetStep());
      }
    }


    /* 按编码器距离运动示例（需要测试时取消对应注释） */
    // Car_ForwardDistance(100U, 160U);    /* 前进 2000mm 后制动 */
    // HAL_Delay(4000U);
    // Car_BackwardDistance(70U, 1000U);   /* 后退 200mm 后制动 */
    // HAL_Delay(10000U);
    // HAL_Delay(3000U);
    // Car_RotateLeftAngle(100U, 1800U);       /* 原地左转 90.0° */
    // HAL_Delay(2000U);
    // Car_RotateRightAngle(100U, 1800U);      /* 原地右转 90.0° */
    // HAL_Delay(2000U);
    // Car_TurnLeftAngle(100U, 150U, 3600U);  /* 半径 150mm 左转 90.0° */
    // HAL_Delay(2000U);
    // Car_TurnRightAngle(100U, 150U, 3600U); /* 半径 150mm 右转 90.0° */

    /* 超声波避障已并入 avoid_task（阶段 4），正式路径由调度器驱动。
       旧的 UltrasonicAvoid_Handle() 保留为独立调试入口，但它直写 RGB 和
       蜂鸣器、绕过指示层，启用时会和正式告警互相覆盖。 */
    // if (UltrasonicAvoid_Handle() == 0U)
    // {
    //   Car_ForwardRun(70U);
    // }
    // HAL_Delay(1U);

    /* 红外遥控与 OLED 调试，需要时单独启用。 */
    // IrRemoteDebug_Update();

    /* 红外避障诊断：正式路径已由上面的 IrAvoid_Sense() 非阻塞采样，
       状态变化时自动投事件。需要阻塞式单次采样标定阈值时用：
       IrAvoid_UpdateBlocking(); */
    // IrAvoid_UpdateBlocking();

    /* 四路红外黑线循迹（集成避障时使用 LineTracker_Step()，不要使用
       内部无限循环的 LineTracker_Run()）。独立循迹调试时可单独启用： */
    // LineTracker_Run();

    /* 变速直线：先加速再减速，走一个梭形速度曲线 */
    // RGB_SetColor(0, 0, 1);                      /* 蓝色 - 正在变速直线 */
    // Car_ForwardVary(RAMP_SPEED_LO, RAMP_SPEED_HI, RAMP_MS);   /* 加速 */
    // Car_ForwardVary(RAMP_SPEED_HI, RAMP_SPEED_LO, RAMP_MS);   /* 减速 */
    // Car_Brake(TRAJ_GAP_MS);


  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

