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

/* Private function prototypes -----------------------------------------------*/
static uint8_t Car_ToEff(uint8_t speed);
static int16_t Car_EffToDuty(int16_t eff);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  按车体方向驱动左右两侧车轮
  * @param  left:  左侧有效速度，-CAR_SPEED_SPAN~CAR_SPEED_SPAN，正数向前
  * @param  right: 右侧有效速度，同上
  * @note   入参是**有效速度**（已减掉死区），这里负责加回 CAR_SPEED_BASE
  *         再下发，并通过极性宏把"车体方向"换算成各电机的实际转向。
  *         上层函数只在有效速度刻度上做比例运算，不必关心死区和接线。
  */
static void Car_Drive(int16_t left, int16_t right)
{
  int16_t dl = Car_EffToDuty(left);
  int16_t dr = Car_EffToDuty(right);

  Motor_SetSpeedRaw(MOTOR_1, (int8_t)(CAR_LEFT_POLARITY  * dl));
  Motor_SetSpeedRaw(MOTOR_2, (int8_t)(CAR_LEFT_POLARITY  * dl));
  Motor_SetSpeedRaw(MOTOR_3, (int8_t)(CAR_RIGHT_POLARITY * dr));
  Motor_SetSpeedRaw(MOTOR_4, (int8_t)(CAR_RIGHT_POLARITY * dr));
}

/**
  * @brief  占空比刻度 → 有效速度
  * @param  speed: 占空比刻度 0~100
  * @retval 有效速度 0~CAR_SPEED_SPAN
  *
  * @note   减去死区。不超过 CAR_SPEED_BASE 的输入一律返回 0，因为那一段
  *         占空比电机根本不转，抬到 BASE+1 反而变成"用户要停我却在动"。
  */
static uint8_t Car_ToEff(uint8_t speed)
{
  if (speed <= (uint8_t)CAR_SPEED_BASE)
  {
    return 0U;
  }
  if (speed >= (uint8_t)MOTOR_SPEED_MAX)
  {
    return (uint8_t)CAR_SPEED_SPAN;
  }
  return (uint8_t)(speed - CAR_SPEED_BASE);
}

/**
  * @brief  有效速度 → 占空比刻度
  * @param  eff: 有效速度，-CAR_SPEED_SPAN~CAR_SPEED_SPAN，正数向前
  * @retval 占空比刻度，符号保留
  *
  * @note   加回死区。有效速度 0 输出 0 而不是 BASE：输出 BASE 电机
  *         照样不转，却白白通电发热，还让被拖动的轮子多了阻力。
  */
static int16_t Car_EffToDuty(int16_t eff)
{
  int16_t mag = (eff >= 0) ? eff : (int16_t)(-eff);

  if (mag == 0)
  {
    return 0;
  }
  if (mag > (int16_t)CAR_SPEED_SPAN)
  {
    mag = (int16_t)CAR_SPEED_SPAN;
  }
  mag = (int16_t)(mag + (int16_t)CAR_SPEED_BASE);

  return (eff >= 0) ? mag : (int16_t)(-mag);
}

/**
  * @brief  按转弯半径算出内侧轮的有效速度
  * @param  outer:     外侧轮**有效速度** 0~CAR_SPEED_SPAN
  * @param  radius_mm: 转弯半径（车体中心线到圆心），单位 mm
  * @retval 内侧轮有效速度 0~CAR_SPEED_SPAN
  *
  * @note   差速运动学：两轮走同心圆、角速度相同，故
  *             v₂ / (R + D/2) = v₁ / (R - D/2)
  *         整理即 2R(v₂ - v₁) = D(v₁ + v₂)，解出内轮速
  *             v₁ = v₂ · (2R - D) / (2R + D)
  *         R = D/2 时 v₁ = 0（绕内侧轮打转），R → ∞ 时 v₁ → v₂（直线）。
  *
  * @note   v 必须是**有效速度**而不是占空比。占空比含 50% 的死区偏置，
  *         按占空比取比例会把偏置也按比例缩放，速比失真、半径偏大。
  */
static int16_t Car_InnerSpeed(uint8_t outer, uint16_t radius_mm)
{
  int32_t d  = (int32_t)CAR_WHEEL_TRACK_MM;
  int32_t r2 = 2 * (int32_t)radius_mm;

  if (radius_mm <= CAR_RADIUS_MIN_MM)
  {
    return 0;                       /* 绕内侧轮打转，内轮停 */
  }
  return (int16_t)((int32_t)outer * (r2 - d) / (r2 + d));
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
  int16_t v = (int16_t)Car_ToEff(speed);
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
  int16_t v = (int16_t)Car_ToEff(speed);
  Car_Drive((int16_t)(-v), (int16_t)(-v));
  Car_RunFor(time);
}

/**
  * @brief  变速直线：速度在 time 内从 speed_from 线性变到 speed_to
  * @param  speed_from: 起始速度百分比 0~100
  * @param  speed_to:   结束速度百分比 0~100，比起始小就是减速
  * @param  time:       全程时间（毫秒），结束后自动制动
  *
  * @note   插值在**有效速度**刻度上做。若按占空比插值，50% 的死区偏置也
  *         被算进插值区间，速度-时间曲线在低速段会被压扁，加速度不恒定。
  *
  * @note   每 CAR_RAMP_STEP_MS 更新一次占空比。time 不是步长整数倍时，
  *         余下的零头并在最后一步，保证总时间准确、末速正好是 speed_to。
  */
void Car_ForwardVary(uint8_t speed_from, uint8_t speed_to, uint16_t time)
{
  int16_t  v0    = (int16_t)Car_ToEff(speed_from);
  int16_t  v1    = (int16_t)Car_ToEff(speed_to);
  uint16_t steps = (uint16_t)(time / CAR_RAMP_STEP_MS);

  for (uint16_t i = 1U; i <= steps; i++)
  {
    /* v0 + (v1-v0)·i/steps，整数运算先乘后除保精度 */
    int16_t v = (int16_t)(v0 + (int32_t)(v1 - v0) * (int32_t)i / (int32_t)steps);

    Car_Drive(v, v);
    HAL_Delay(CAR_RAMP_STEP_MS);
  }

  /* 不足一个步长的零头（含 time < CAR_RAMP_STEP_MS 的情况）按末速补齐 */
  if ((uint16_t)(time % CAR_RAMP_STEP_MS) > 0U)
  {
    Car_Drive(v1, v1);
    HAL_Delay((uint16_t)(time % CAR_RAMP_STEP_MS));
  }

  Motor_BrakeAll();
}

/**
  * @brief  左转，转弯半径可调
  * @param  speed:     外侧（右侧）车轮速度百分比 0~100
  * @param  radius_mm: 转弯半径（车体中心线到圆心），单位 mm
  *                    ≤ D/2 时内侧轮停转，绕内侧轮打转；越大弯道越缓
  * @param  time:      持续时间（毫秒），结束后自动制动
  */
void Car_TurnLeft(uint8_t speed, uint16_t radius_mm, uint16_t time)
{
  uint8_t outer = Car_ToEff(speed);
  Car_Drive(Car_InnerSpeed(outer, radius_mm), (int16_t)outer);
  Car_RunFor(time);
}

/**
  * @brief  右转，转弯半径可调
  * @param  speed:     外侧（左侧）车轮速度百分比 0~100
  * @param  radius_mm: 转弯半径（车体中心线到圆心），单位 mm
  *                    ≤ D/2 时内侧轮停转，绕内侧轮打转；越大弯道越缓
  * @param  time:      持续时间（毫秒），结束后自动制动
  */
void Car_TurnRight(uint8_t speed, uint16_t radius_mm, uint16_t time)
{
  uint8_t outer = Car_ToEff(speed);
  Car_Drive((int16_t)outer, Car_InnerSpeed(outer, radius_mm));
  Car_RunFor(time);
}

/**
  * @brief  左旋转（原地左转，左轮后退右轮前进）
  * @param  speed: 速度百分比 0~100
  * @param  time:  持续时间（毫秒），结束后自动制动
  */
void Car_RotateLeft(uint8_t speed, uint16_t time)
{
  int16_t v = (int16_t)Car_ToEff(speed);
  Car_Drive((int16_t)(-v), v);
  Car_RunFor(time);
}

/**
  * @brief  右旋转（原地右转，左轮前进右轮后退）
  * @param  speed: 速度百分比 0~100
  * @param  time:  持续时间（毫秒），结束后自动制动
  */
void Car_RotateRight(uint8_t speed, uint16_t time)
{
  int16_t v = (int16_t)Car_ToEff(speed);
  Car_Drive(v, (int16_t)(-v));
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

/**
  * @brief  走一个三叶草轨迹
  * @param  speed:     叶片行驶速度 0~100
  * @param  radius_mm: 叶片转弯半径 mm，越小整个图形占地越小
  * @param  leaf_time: 走完一整片叶子（转一整圈）的时间（毫秒）
  *
  * @note   每片叶子是一整圈 360° 定半径圆弧，走完车回到出发点、朝向复原；
  *         再原地左旋 120° 换下一片叶子的方向。三片叶子的圆心按 120°
  *         均匀分布，合起来就是三叶草：
  *
  *                    ___
  *                   /   \        ← 叶 1
  *              ___  \___/  ___
  *             /   \  /|\  /   \
  *             \___/   |   \___/  ← 叶 2、叶 3
  *
  *         三次旋转累计 360°，一轮结束后车头方向与开始时一致，可无缝重复。
  *         整个图形的外径约 4R，R = 150mm 时占地约 60cm 见方。
  *
  * @note   目前圆弧和旋转角度靠时间估算，leaf_time 和 CLOVER_TURN_MS
  *         必须在实际场地上标定：
  *           1. 先只跑一片叶子，调 leaf_time 到车刚好回到出发点；
  *           2. 再调 CLOVER_TURN_MS 到原地转过的角度接近 120°。
  *         电池电压下降会让两个时间都变长。接上编码器后可改成按里程闭环。
  */
void Car_Clover(uint8_t speed, uint16_t radius_mm, uint16_t leaf_time)
{
  for (uint8_t leaf = 0U; leaf < 3U; leaf++)
  {
    /* 一整圈定半径圆弧，画出一片叶子 */
    Car_TurnLeft(speed, radius_mm, leaf_time);
    Car_Brake(150U);                 /* 短暂停顿，让车稳住再换向 */

    /* 原地左旋 120°，对准下一片叶子 */
    Car_RotateLeft(CLOVER_TURN_SPEED, CLOVER_TURN_MS);
    Car_Brake(150U);
  }
}
