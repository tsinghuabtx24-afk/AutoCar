/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.h
  * @brief   This file contains all the function prototypes for
  *          the tim.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TIM_H__
#define __TIM_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern TIM_HandleTypeDef htim1;

extern TIM_HandleTypeDef htim8;

/* USER CODE BEGIN Private defines */

/* 四路电机编号：M1 左前、M2 左后、M3 右前、M4 右后
   每只电机占用两路 PWM（A/B 相），共 8 路：
     M1A=TIM8_CH1(PC6)  M1B=TIM8_CH2(PC7)
     M2A=TIM8_CH3(PC8)  M2B=TIM8_CH4(PC9)
     M3A=TIM1_CH1(PE9)  M3B=TIM1_CH2(PE11)
     M4A=TIM1_CH3(PE13) M4B=TIM1_CH4(PE14) */
typedef enum
{
  MOTOR_1 = 0,   /* 左前 */
  MOTOR_2,       /* 左后 */
  MOTOR_3,       /* 右前 */
  MOTOR_4,       /* 右后 */
  MOTOR_NUM
} Motor_ID;

/* PWM 计数周期：72MHz / 3600 = 20kHz，超出人耳范围，电机不会啸叫 */
#define MOTOR_PWM_PERIOD     3600U

/* 速度上限（百分比）*/
#define MOTOR_SPEED_MAX      100U

/* 最小有效占空比：低于此值电机堵转、只会发出嗡嗡声而不转动，
   因此所有非零速度都会被抬到这个下限。实测可按整车负载调整。 */
#define MOTOR_SPEED_MIN      30U

/* USER CODE END Private defines */

void MX_TIM1_Init(void);
void MX_TIM8_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN Prototypes */

void Motor_Init(void);
void Motor_Forward(Motor_ID motor, uint8_t speed);
void Motor_Backward(Motor_ID motor, uint8_t speed);
void Motor_Brake(Motor_ID motor);
void Motor_Coast(Motor_ID motor);
void Motor_SetSpeed(Motor_ID motor, int8_t speed);
void Motor_BrakeAll(void);

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __TIM_H__ */

