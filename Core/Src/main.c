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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* RGB LED timing: 0.2s on, 0.2s off */
#define RGB_PERIOD_MS  200U

/* 电机接线极性补偿：左侧（M1/M2）实测转向与右侧（M3/M4）相反，故取 -1。
   若整车前后方向相反，把下面两个宏同时取反。 */
#define MOTOR_LEFT_POLARITY   (-1)
#define MOTOR_RIGHT_POLARITY  (1)

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

  /* 左侧 RGB 的红、绿两路实际接线互换，此处交换补偿，使两侧显示同色 */
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

/**
  * @brief  设置单个电机的转向
  * @param  A_Port/A_Pin: 电机A相引脚
  * @param  B_Port/B_Pin: 电机B相引脚
  * @param  dir: 方向，+1 = 正转，-1 = 反转，0 = 停止
  * @retval None
  */
static void Motor_SetDir(GPIO_TypeDef *A_Port, uint16_t A_Pin,
                         GPIO_TypeDef *B_Port, uint16_t B_Pin, int8_t dir)
{
  HAL_GPIO_WritePin(A_Port, A_Pin, (dir > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(B_Port, B_Pin, (dir < 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/**
  * @brief  按车体方向驱动左右两侧车轮
  * @param  left:  左侧方向，+1 = 向前，-1 = 向后，0 = 停止
  * @param  right: 右侧方向，+1 = 向前，-1 = 向后，0 = 停止
  * @retval None
  * @note   M1 左前、M2 左后、M3 右前、M4 右后。
  *         左侧两只电机接线极性与右侧相反，用 MOTOR_LEFT_POLARITY 统一补偿，
  *         若整车方向相反，把两个极性宏同时取反即可。
  */
static void Motor_Drive(int8_t left, int8_t right)
{
  Motor_SetDir(M1A_GPIO_Port, M1A_Pin, M1B_GPIO_Port, M1B_Pin, MOTOR_LEFT_POLARITY  * left);
  Motor_SetDir(M2A_GPIO_Port, M2A_Pin, M2B_GPIO_Port, M2B_Pin, MOTOR_LEFT_POLARITY  * left);
  Motor_SetDir(M3A_GPIO_Port, M3A_Pin, M3B_GPIO_Port, M3B_Pin, MOTOR_RIGHT_POLARITY * right);
  Motor_SetDir(M4A_GPIO_Port, M4A_Pin, M4B_GPIO_Port, M4B_Pin, MOTOR_RIGHT_POLARITY * right);
}

/**
  * @brief  停止所有电机
  * @retval None
  */
void Motor_Stop(void)
{
  Motor_Drive(0, 0);
}

/**
  * @brief  小车前进
  * @param  time: 运动时长（毫秒）
  * @retval None
  */
void Car_Forward(int16_t time)
{
  Motor_Drive(1, 1);   /* 两侧同时向前 */
  HAL_Delay(time);
  Motor_Stop();
}

/**
  * @brief  小车后退
  * @param  time: 运动时长（毫秒）
  * @retval None
  */
void Car_Backward(int16_t time)
{
  Motor_Drive(-1, -1); /* 两侧同时向后 */
  HAL_Delay(time);
  Motor_Stop();
}

/**
  * @brief  小车左转（左侧轮慢，右侧轮快）
  * @param  time: 运动时长（毫秒）
  * @retval None
  */
void Car_TurnLeft(int16_t time)
{
  Motor_Drive(0, 1);   /* 左侧停、右侧向前，以左轮为中心左转 */
  HAL_Delay(time);
  Motor_Stop();
}

/**
  * @brief  小车右转（右侧轮慢，左侧轮快）
  * @param  time: 运动时长（毫秒）
  * @retval None
  */
void Car_TurnRight(int16_t time)
{
  Motor_Drive(1, 0);   /* 左侧向前、右侧停，以右轮为中心右转 */
  HAL_Delay(time);
  Motor_Stop();
}

/**
  * @brief  小车左旋转（原地左转）
  * @param  time: 运动时长（毫秒）
  * @retval None
  */
void Car_RotateLeft(int16_t time)
{
  Motor_Drive(-1, 1);  /* 左后右前，原地左旋 */
  HAL_Delay(time);
  Motor_Stop();
}

/**
  * @brief  小车右旋转（原地右转）
  * @param  time: 运动时长（毫秒）
  * @retval None
  */
void Car_RotateRight(int16_t time)
{
  Motor_Drive(1, -1);  /* 左前右后，原地右旋 */
  HAL_Delay(time);
  Motor_Stop();
}

/**
  * @brief  小车制动（立即停止）
  * @retval None
  */
void Car_Brake(void)
{
  Motor_Stop();
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
  /* USER CODE BEGIN 2 */

  /* Initialize LED2 as on (buzzer is off by default) */
  HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* 小车运动演示程序 */

    /* 1. 前进2秒 */
    // RGB_SetColor(1, 0, 0);  /* 红色 - 前进 */
    // Car_Forward(2000);
    // HAL_Delay(500);  /* 间隔 */

    // /* 2. 后退2秒 */
    // RGB_SetColor(1, 1, 0);  /* 黄色 - 后退 */
    // Car_Backward(2000);
    // HAL_Delay(500);

    // /* 3. 左转1秒 */
    // RGB_SetColor(0, 1, 0);  /* 绿色 - 左转 */
    // Car_TurnLeft(1000);
    // HAL_Delay(500);

    // /* 4. 右转1秒 */
    // RGB_SetColor(0, 1, 1);  /* 青色 - 右转 */
    // Car_TurnRight(1000);
    // HAL_Delay(500);

    // /* 5. 左旋转1秒 */
    // RGB_SetColor(0, 0, 1);  /* 蓝色 - 左旋转 */
    // Car_RotateLeft(1000);
    // HAL_Delay(500);

    // /* 6. 右旋转1秒 */
    // RGB_SetColor(1, 0, 1);  /* 品红色 - 右旋转 */
    // Car_RotateRight(1000);
    // HAL_Delay(500);

    // /* 7. 制动并等待 */
    // RGB_SetColor(0, 0, 0);  /* 关闭RGB */
    // Car_Brake();
    // HAL_Delay(2000);  /* 停止2秒后重复 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
