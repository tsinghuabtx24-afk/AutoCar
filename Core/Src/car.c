/**
  ******************************************************************************
  * @file    car.c
  * @brief   智能小车运动控制实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "car.h"

/* Private define ------------------------------------------------------------*/
/* 电机接线极性补偿：左侧（M1/M2）实测转向与右侧（M3/M4）相反，故取 -1。
   若整车前后方向相反，把下面两个宏同时取反。 */
#define CAR_LEFT_POLARITY    (-1)
#define CAR_RIGHT_POLARITY   (1)

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  按车体方向驱动左右两侧车轮
  * @param  left:  左侧速度，-100~100，正数向前
  * @param  right: 右侧速度，-100~100，正数向前
  * @note   通过极性宏把"车体方向"换算成各电机的实际转向，
  *         上层函数只需关心车往哪走，不必关心电机怎么接线。
  */
static void Car_Drive(int8_t left, int8_t right)
{
  Motor_SetSpeed(MOTOR_1, (int8_t)(CAR_LEFT_POLARITY  * left));
  Motor_SetSpeed(MOTOR_2, (int8_t)(CAR_LEFT_POLARITY  * left));
  Motor_SetSpeed(MOTOR_3, (int8_t)(CAR_RIGHT_POLARITY * right));
  Motor_SetSpeed(MOTOR_4, (int8_t)(CAR_RIGHT_POLARITY * right));
}

/**
  * @brief  把速度限制在 0~100
  */
static uint8_t Car_ClampSpeed(uint8_t speed)
{
  return (speed > MOTOR_SPEED_MAX) ? (uint8_t)MOTOR_SPEED_MAX : speed;
}

/**
  * @brief  保持当前动作一段时间后制动
  * @param  time: 毫秒
  */
static void Car_RunFor(uint16_t time)
{
  HAL_Delay(time);
  Motor_BrakeAll();
}

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  小车运动控制初始化
  * @note   内部调用 Motor_Init() 启动 8 路 PWM，调用前需先完成
  *         MX_TIM1_Init() 与 MX_TIM8_Init()。
  */
void Car_Init(void)
{
  Motor_Init();
}

/**
  * @brief  前进
  * @param  speed: 速度百分比 0~100
  * @param  time:  持续时间（毫秒），结束后自动制动
  */
void Car_Forward(uint8_t speed, uint16_t time)
{
  int8_t v = (int8_t)Car_ClampSpeed(speed);
  Car_Drive(v, v);
  Car_RunFor(time);
}

/**
  * @brief  后退
  * @param  speed: 速度百分比 0~100
  * @param  time:  持续时间（毫秒），结束后自动制动
  */
void Car_Backward(uint8_t speed, uint16_t time)
{
  int8_t v = (int8_t)Car_ClampSpeed(speed);
  Car_Drive((int8_t)(-v), (int8_t)(-v));
  Car_RunFor(time);
}

/**
  * @brief  左转（差速转弯，左轮慢右轮快，边走边拐）
  * @param  speed: 外侧（右侧）车轮速度百分比 0~100
  * @param  time:  持续时间（毫秒），结束后自动制动
  */
void Car_TurnLeft(uint8_t speed, uint16_t time)
{
  int8_t outer = (int8_t)Car_ClampSpeed(speed);
  int8_t inner = (int8_t)((uint32_t)outer * CAR_TURN_INNER_RATIO / 100U);
  Car_Drive(inner, outer);
  Car_RunFor(time);
}

/**
  * @brief  右转（差速转弯，右轮慢左轮快，边走边拐）
  * @param  speed: 外侧（左侧）车轮速度百分比 0~100
  * @param  time:  持续时间（毫秒），结束后自动制动
  */
void Car_TurnRight(uint8_t speed, uint16_t time)
{
  int8_t outer = (int8_t)Car_ClampSpeed(speed);
  int8_t inner = (int8_t)((uint32_t)outer * CAR_TURN_INNER_RATIO / 100U);
  Car_Drive(outer, inner);
  Car_RunFor(time);
}

/**
  * @brief  左旋转（原地左转，左轮后退右轮前进）
  * @param  speed: 速度百分比 0~100
  * @param  time:  持续时间（毫秒），结束后自动制动
  */
void Car_RotateLeft(uint8_t speed, uint16_t time)
{
  int8_t v = (int8_t)Car_ClampSpeed(speed);
  Car_Drive((int8_t)(-v), v);
  Car_RunFor(time);
}

/**
  * @brief  右旋转（原地右转，左轮前进右轮后退）
  * @param  speed: 速度百分比 0~100
  * @param  time:  持续时间（毫秒），结束后自动制动
  */
void Car_RotateRight(uint8_t speed, uint16_t time)
{
  int8_t v = (int8_t)Car_ClampSpeed(speed);
  Car_Drive(v, (int8_t)(-v));
  Car_RunFor(time);
}

/**
  * @brief  制动
  * @param  time: 制动状态保持的时间（毫秒），传 0 则立即返回
  */
void Car_Brake(uint16_t time)
{
  Motor_BrakeAll();
  if (time > 0U)
  {
    HAL_Delay(time);
  }
}
