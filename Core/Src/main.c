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
  if (GPIO_Pin == key1_Pin || GPIO_Pin == key3_Pin)
  {
    /* Key1 or Key3 pressed: turn on buzzer */
    buzzer_on = 1;
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
  }
  else if (GPIO_Pin == key2_Pin)
  {
    /* Key2 pressed: turn off buzzer */
    buzzer_on = 0;
    HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
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
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
    MX_USART1_UART_Init();
  MX_I2C1_Init();
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

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* 按编码器距离运动示例（需要测试时取消对应注释） */
    // Car_ForwardDistance(70U, 1000U);    /* 前进 200mm 后制动 */
    // HAL_Delay(10000U);
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
    

    /* 红外模块优先；无障碍时先保持直线行驶。后续接入循迹时，将下面的 Car_ForwardRun() 替换为 LineTracker_Step()，
    不要调用 LineTracker_Run()，因为 Run() 内部有无限循环，会阻塞上层仲裁。 */
    if (IrAvoid_Handle() == 0U)
    {
      Car_ForwardRun(70U);
      /* LineTracker_Step(); */
    }
    HAL_Delay(1U);

    /* 红外避障诊断：每 20ms 采样，串口输出左右原始 ADC 与状态。
       正式避障运行时由 IrAvoid_Handle() 内部调用；需要单独诊断时可改为：
       IrAvoid_Update(); */
    // IrAvoid_Update();

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
