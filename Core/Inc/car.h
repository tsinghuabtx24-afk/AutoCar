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
   实测占空比 ≤ CAR_SPEED_BASE 时电机堵转，这段占空比对转速毫无贡献。所以所有
   速度运算都先减去它得到"有效速度"，按比例算完再加回去作为占空比输出。

   **必须这样算**：直接按占空比取比例的话，死区那 50% 进了分子分母，内外轮速比
   全是错的，弯道半径也就不准。

   接上编码器改成转速闭环后，这一整套开环补偿可以删掉。
   ============================================================================ */
/* 与电机层的启动下限是同一个实测值，只在 tim.h 定义一次，避免两处不同步。 */
#define CAR_SPEED_BASE         MOTOR_SPEED_MIN
#define CAR_SPEED_SPAN         (MOTOR_SPEED_MAX - CAR_SPEED_BASE)

/* 变速直线的速度按时间线性插值，每 CAR_RAMP_STEP_MS 下发一次。20ms 时 1.5 秒
   行程有 75 级速度，肉眼看不出分段；再小也换不来更细的台阶（tick 只有 1ms）。 */
#define CAR_RAMP_STEP_MS       20U

/* 演示用的变速区间。下限取 60 而非 CAR_SPEED_BASE：刚出死区时转速对占空比的
   响应很不线性，起步会一顿一顿的。 */
#define RAMP_SPEED_LO          60U
#define RAMP_SPEED_HI          100U
#define RAMP_MS                1500U  /* 单程加速（或减速）的时间 */

/* ==== 三叶草轨迹参数 ========================================================
   下面两个时间是**起点值不是正确值**，开环靠时间估算，电池电压、地面摩擦、
   载重都会改变结果。标定顺序：
     1. 只跑一片叶子，调 CLOVER_LEAF_MS 到车刚好回到出发点、车头朝向复原；
     2. 再调 CLOVER_TURN_MS 到原地转过的角度接近 120°。
   两者独立，但都与对应的速度绑定——改了速度就得重新标定时间。
   ============================================================================ */
#define CLOVER_SPEED           100U   /* 走叶片时的外侧轮速度 */
#define CLOVER_RADIUS_MM       150U   /* 叶片转弯半径，越小整个图形占地越小 */
#define CLOVER_LEAF_MS         2000U  /* 走完一整片叶子（转一整圈）的时间，待标定 */
#define CLOVER_TURN_SPEED      100U   /* 叶片之间原地转向的速度 */
#define CLOVER_TURN_MS         420U   /* 原地转 120° 所需时间，待标定 */

/* 按编码器运动的异常保护时间，距离和角度动作共用。编码器没有脉冲时到期自动
   制动，防止接线或参数错误导致小车一直跑。 */
#define CAR_DISTANCE_TIMEOUT_MS  60000U

/* 动作诊断输出周期。 */
#define CAR_ACTION_REPORT_MS     500U

/* 纯计时动作（定时保持、变速斜坡、制动保持）的超时余量。这类动作不依赖编码器、
   到点必然完成，超时只是防"调用方忘了推进 Step"，不需要 60 秒。 */
#define CAR_ACTION_TIMEOUT_MARGIN_MS  1000U

/* ==== 编码器角度滑移补偿 ====================================================
   原地旋转和差速转弯时轮胎横向滑移，编码器算出的角度明显大于车体真实转角，
   误差还随 PWM、电量、地面和轮胎变化。对外接口仍传真实目标角度，内部放大为
   编码器停止角度。系数为千分比，1000 = 1.000 倍。

   改用 IMU 航向闭环或换麦克纳姆轮时，CAR_ANGLE_COMP_ENABLE 置 0 即可整套关闭。
   ============================================================================ */
#define CAR_ANGLE_COMP_ENABLE       1U
#define CAR_ANGLE_MIN_SPEED         55U

/* 原地旋转实测标定点：指令 90° 时，70/80/100 分别约转 40°/41°/45°。 */
#define CAR_ROT_COMP_70_X1000       2250U
#define CAR_ROT_COMP_80_X1000       2195U
#define CAR_ROT_COMP_100_X1000      2000U

/* 半径转弯仅有 100 PWM 标定数据：左转约 45°，右转约 30°。 */
#define CAR_TURN_LEFT_COMP_X1000    2200U
#define CAR_TURN_RIGHT_COMP_X1000   3000U

/* ==== 非阻塞动作状态机 ======================================================
   长动作（按里程、按角度、变速斜坡、定时保持）统一走"发起 + 每轮推进"两步：

       Car_StartForwardDistance(70U, 200U);      // 发起一次
       ...
       switch (Car_ActionStep())                 // 每轮主循环推进一次
       {
         case CAR_ACTION_BUSY:    break;                     // 还在走
         case CAR_ACTION_DONE:    task_phase++;        break; // 完成
         case CAR_ACTION_TIMEOUT:
         case CAR_ACTION_FAILED:  task_state = FAILED; break; // 失败
       }

   同一时刻只允许一个动作。终态（DONE/TIMEOUT/FAILED）时内部已制动并转 IDLE，
   Step 返回终态**一次**，之后返回 IDLE。

   动作内部存编码器起始快照而不调 Encoder_ResetAll()，以免破坏其他模块的里程基准。
   ============================================================================ */
typedef enum
{
  CAR_ACTION_IDLE = 0,    /* 无动作 */
  CAR_ACTION_BUSY,        /* 进行中，需要继续 Step */
  CAR_ACTION_DONE,        /* 正常完成，已制动 */
  CAR_ACTION_TIMEOUT,     /* 超时退出，已制动 */
  CAR_ACTION_FAILED       /* 参数非法（已制动），或已有动作在跑（未干扰它） */
} Car_ActionStatus;

/* 发起动作，成功返回 CAR_ACTION_BUSY。两种失败要分清：
     - 参数非法（速度落在死区、距离/角度为 0）：制动后返回 FAILED；
     - 已有动作在跑：返回 FAILED 且**不打断**它，底盘不受影响。
   需要抢占的调用方应先显式 Car_ActionAbort()。 */
Car_ActionStatus Car_StartForwardDistance(uint8_t speed, uint32_t distance_mm);
Car_ActionStatus Car_StartBackwardDistance(uint8_t speed, uint32_t distance_mm);

/* 角度动作用带符号角度：正=左转，负=右转。radius_mm 为 0 表示原地旋转。 */
Car_ActionStatus Car_StartRotateAngle(uint8_t speed, int32_t angle_deg10);
Car_ActionStatus Car_StartTurnAngle(uint8_t speed, uint16_t radius_mm,
                                    int32_t angle_deg10);

/* 定时类动作：保持当前差速 time 毫秒后制动。left/right 为占空比刻度 -100~100，
   正数向前。 */
Car_ActionStatus Car_StartTimed(int16_t left, int16_t right, uint16_t time);

/* 变速直线：time 内速度从 speed_from 线性变到 speed_to，结束后制动。 */
Car_ActionStatus Car_StartForwardVary(uint8_t speed_from, uint8_t speed_to,
                                      uint16_t time);

/* 制动并保持 time 毫秒。time 为 0 时立即完成。 */
Car_ActionStatus Car_StartBrake(uint16_t time);

/* 每轮主循环推进一次。无动作时返回 CAR_ACTION_IDLE，不触碰底盘。 */
Car_ActionStatus Car_ActionStep(void);

/* 立即放弃当前动作并制动。无动作时无副作用。 */
void Car_ActionAbort(void);

/* 查询当前状态，不推进。 */
Car_ActionStatus Car_GetActionStatus(void);
uint8_t Car_IsActionBusy(void);

/* 瞬时接口：设 PWM 后立即返回，供循迹的差速连续修正、遥控手动转向等周期控制用。
   占空比刻度 -100~100，正数向前。 */
void Car_DriveSpeed(int16_t left, int16_t right);

/* Exported functions prototypes ---------------------------------------------*/
void Car_Init(void);

void Car_Forward(uint8_t speed, uint16_t time);
void Car_Backward(uint8_t speed, uint16_t time);
void Car_ForwardRun(uint8_t speed);
void Car_Stop(void);
void Car_ForwardDistance(uint8_t speed, uint32_t distance_mm);
void Car_BackwardDistance(uint8_t speed, uint32_t distance_mm);
void Car_ForwardVary(uint8_t speed_from, uint8_t speed_to, uint16_t time);
void Car_TurnLeft(uint8_t speed, uint16_t radius_mm, uint16_t time);
void Car_TurnRight(uint8_t speed, uint16_t radius_mm, uint16_t time);
void Car_RotateLeft(uint8_t speed, uint16_t time);
void Car_RotateRight(uint8_t speed, uint16_t time);
void Car_RotateLeftAngle(uint8_t speed, uint32_t angle_deg10);
void Car_RotateRightAngle(uint8_t speed, uint32_t angle_deg10);
void Car_TurnLeftAngle(uint8_t speed, uint16_t radius_mm, uint32_t angle_deg10);
void Car_TurnRightAngle(uint8_t speed, uint16_t radius_mm, uint32_t angle_deg10);
void Car_Brake(uint16_t time);

/* 上面这批阻塞接口是独立标定入口，内部就是"Start + 循环 Step 到终态"，与非阻塞
   路径共享同一份逻辑。正式主循环不要调用：动作期间调度器无法仲裁。 */

void Car_Clover(uint8_t speed, uint16_t radius_mm, uint16_t leaf_time);

#ifdef __cplusplus
}
#endif

#endif /* __CAR_H__ */
