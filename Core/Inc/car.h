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
  *          M1/M2 为左侧，M3/M4 为右侧。time 为持续毫秒数，动作结束后自动制动。
  *
  *          speed 为占空比刻度 0~100，但因为电机在 CAR_SPEED_BASE 以下堵转，
  *          实际可用区间是 (CAR_SPEED_BASE, 100]：
  *              ≤50 → 停       75 → 半速       100 → 满速
  *          内部统一先减 CAR_SPEED_BASE 得有效速度、按比例运算、再加回去。
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

/* 两轮间距 D，单位 mm */
#define CAR_WHEEL_TRACK_MM     128U

/* 转弯半径下限：R = D/2 时内侧轮速为 0，车绕内侧轮打转；
   再小内侧轮就要反转，属于原地旋转的范畴，用 Car_RotateLeft/Right 更直观。 */
#define CAR_RADIUS_MIN_MM      (CAR_WHEEL_TRACK_MM / 2U)

/* ==== 电机死区补偿 ==========================================================
   实测占空比不超过 CAR_SPEED_BASE 时电机堵转不动，这一段占空比对转速没有
   任何贡献。所以所有速度运算都先减去 CAR_SPEED_BASE 得到"有效速度"，
   在有效速度区间内成比例计算，最后再加回 CAR_SPEED_BASE 作为占空比输出。
   只有这样内外轮的速比才是真实的转速比 —— 直接按占空比取比例的话，
   死区那 50% 被算进了分子分母，速比全是错的，弯道半径也就不准。

   speed 参数的含义随之变成占空比刻度：
       CAR_SPEED_BASE(50) → 停          MOTOR_SPEED_MAX(100) → 满速
       75 → 半速                        0 → 停止（滑行）
   接上编码器改成按里程/转速闭环后，这一整套开环补偿可以删掉。
   ============================================================================ */
/* 死区上界。与电机层的启动下限是同一个实测值，所以只在 tim.h 里定义一次，
   要改就改 MOTOR_SPEED_MIN，避免两处不同步。 */
#define CAR_SPEED_BASE         MOTOR_SPEED_MIN
#define CAR_SPEED_SPAN         (MOTOR_SPEED_MAX - CAR_SPEED_BASE)

/* ==== 变速直线参数 ==========================================================
   变速直线把速度按时间线性插值，每 CAR_RAMP_STEP_MS 下发一次。步长越小越
   平滑，但 HAL_Delay 只有 1ms 分辨率，再小也换不来更细的台阶；20ms 时
   1.5 秒的行程有 75 级速度，肉眼已经看不出分段。
   ============================================================================ */
#define CAR_RAMP_STEP_MS       20U

/* 演示用的变速区间。下限取 60 而不是 CAR_SPEED_BASE：有效速度太低时
   电机刚出死区，转速对占空比的响应很不线性，起步会一顿一顿的。 */
#define RAMP_SPEED_LO          60U
#define RAMP_SPEED_HI          100U
#define RAMP_MS                1500U  /* 单程加速（或减速）的时间 */

/* ==== 三叶草轨迹参数 ========================================================
   下面两个时间必须在实际场地上标定，给的是起点值不是正确值。开环靠时间估算，
   电池电压、地面摩擦、载重都会改变结果。标定顺序：
     1. 先只跑一片叶子，调 CLOVER_LEAF_MS 到车刚好回到出发点、车头朝向复原；
     2. 再调 CLOVER_TURN_MS 到原地转过的角度接近 120°。
   注意两者互相独立，但都和对应的速度绑定 —— 改了速度就得重新标定时间。
   ============================================================================ */
#define CLOVER_SPEED           100U   /* 走叶片时的外侧轮速度 */
#define CLOVER_RADIUS_MM       150U   /* 叶片转弯半径，越小整个图形占地越小 */
#define CLOVER_LEAF_MS         2000U  /* 走完一整片叶子（转一整圈）的时间，待标定 */
#define CLOVER_TURN_SPEED      100U   /* 叶片之间原地转向的速度 */
#define CLOVER_TURN_MS         420U   /* 原地转 120° 所需时间，待标定 */

/* 按编码器距离运动的异常保护时间。编码器没有脉冲时到期自动制动，
   防止因编码器接线或参数错误导致小车一直运行。 */
#define CAR_DISTANCE_TIMEOUT_MS  60000U

/* Exported functions prototypes ---------------------------------------------*/
void Car_Init(void);

void Car_Forward(uint8_t speed, uint16_t time);
void Car_Backward(uint8_t speed, uint16_t time);
void Car_ForwardDistance(uint8_t speed, uint32_t distance_mm);
void Car_BackwardDistance(uint8_t speed, uint32_t distance_mm);
void Car_ForwardVary(uint8_t speed_from, uint8_t speed_to, uint16_t time);
void Car_TurnLeft(uint8_t speed, uint16_t radius_mm, uint16_t time);
void Car_TurnRight(uint8_t speed, uint16_t radius_mm, uint16_t time);
void Car_RotateLeft(uint8_t speed, uint16_t time);
void Car_RotateRight(uint8_t speed, uint16_t time);
void Car_Brake(uint16_t time);

void Car_Clover(uint8_t speed, uint16_t radius_mm, uint16_t leaf_time);

#ifdef __cplusplus
}
#endif

#endif /* __CAR_H__ */
