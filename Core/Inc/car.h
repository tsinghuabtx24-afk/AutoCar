/**
  ******************************************************************************
  * @file    car.h
  * @brief   智能小车运动控制接口
  ******************************************************************************
  * @note    车轮布局（俯视，车头朝上）：
  *
  *              车头
  *          M1 ┌────┐ M3
  *             │    │
  *          M2 └────┘ M4
  *              车尾
  *
  *          M1/M2 为左侧，M3/M4 为右侧。所有运动函数的 speed 为 0~100 的
  *          百分比，time 为持续毫秒数，动作结束后自动制动。
  ******************************************************************************
  */
#ifndef __CAR_H__
#define __CAR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* Exported defines ----------------------------------------------------------*/
/* 转弯时内侧车轮的速度占外侧的比例（百分比），用于实现带转弯半径的左/右转 */
#define CAR_TURN_INNER_RATIO   40U

/* Exported functions prototypes ---------------------------------------------*/
void Car_Init(void);

void Car_Forward(uint8_t speed, uint16_t time);
void Car_Backward(uint8_t speed, uint16_t time);
void Car_TurnLeft(uint8_t speed, uint16_t time);
void Car_TurnRight(uint8_t speed, uint16_t time);
void Car_RotateLeft(uint8_t speed, uint16_t time);
void Car_RotateRight(uint8_t speed, uint16_t time);
void Car_Brake(uint16_t time);

#ifdef __cplusplus
}
#endif

#endif /* __CAR_H__ */
