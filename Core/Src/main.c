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
  HAL_GPIO_WritePin(LRGB_R_GPIO_Port, LRGB_R_Pin, r ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LRGB_G_GPIO_Port, LRGB_G_Pin, g ? GPIO_PIN_SET : GPIO_PIN_RESET);
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

  /* Rainbow colors array: R, G, B */
  const uint8_t rainbow[][3] = {
    {1, 0, 0},  /* Red */
    {1, 1, 0},  /* Yellow */
    {0, 1, 0},  /* Green */
    {0, 1, 1},  /* Cyan */
    {0, 0, 1},  /* Blue */
    {1, 0, 1},  /* Magenta */
  };
  uint8_t color_index = 0;

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* Display current rainbow color */
    RGB_SetColor(rainbow[color_index][0], rainbow[color_index][1], rainbow[color_index][2]);
    HAL_Delay(RGB_PERIOD_MS);

    /* Turn off RGB */
    RGB_SetColor(0, 0, 0);
    HAL_Delay(RGB_PERIOD_MS);

    /* Move to next color */
    color_index++;
    if (color_index >= 6)
    {
      color_index = 0;
    }
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
