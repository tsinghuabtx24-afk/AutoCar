/**
  ******************************************************************************
  * @file    encoder.h
  * @brief   四路电机霍尔编码器读取接口
  ******************************************************************************
  * @note    编码器接线（与电机编号一致）：
  *              M1 左前 → TIM4  (PD12/PD13)
  *              M2 左后 → TIM2  (PA15/PB3)
  *              M3 右前 → TIM5  (PA0/PA1)
  *              M4 右后 → TIM3  (PB4/PB5)
  *
  *          定时器工作在正交解码模式，CNT 由 A/B 相直接驱动，正反转自动
  *          加减，不占用 CPU。本模块只负责周期性把 CNT 的增量累加成
  *          32 位里程，并据此算转速。
  *
  *          用法：Encoder_Init() 之后周期调用 Encoder_Update()，
  *          再用 Encoder_GetDistance() / Encoder_GetSpeedRpm() 取值。
  ******************************************************************************
  */
#ifndef __ENCODER_H__
#define __ENCODER_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"

/* ==== 物理参数（名义值，全部待实测）=========================================
   这三个是**名义值**，不是标定结果。它们保留物理含义方便排查问题，实测
   误差统一由下面的 ENC_CAL_NUM/DEN 吸收 —— 不要直接改这三个来"凑准"，
   否则出了偏差分不清是哪一环错的。
   ============================================================================ */

/* 霍尔编码器单相每转脉冲数。MG310 常见 11 或 13，光电码盘才会是 500。
   实测法：轮子架空手转 10 圈，读 Encoder_GetCount() 除以 10，
   得到的是"轮子每圈计数"，即下面的 ENC_COUNTS_PER_REV。 */
#define ENC_PPR              13U

/* 正交倍频系数。取决于 CubeMX 里 Encoder Mode 的选择：
       Encoder Mode TI1 或 TI2        → 2（x2，只数一相边沿）
       Encoder Mode TI1 and TI2       → 4（x4，两相都数，分辨率最高）
   改了 CubeMX 的模式，这里必须同步改，否则里程差一倍。 */
#define ENC_QUAD_FACTOR      2U

/* 减速比。MG310 常见 20/30/45/100，一般印在减速箱侧面。 */
#define ENC_GEAR_RATIO       20U

/* 轮径 mm。拿卷尺量比查手册准，橡胶轮压扁后有效直径会略小。 */
#define ENC_WHEEL_DIA_MM     65U

/* 轮子转一圈的编码器计数。编码器装在电机轴上，所以要乘减速比。 */
#define ENC_COUNTS_PER_REV   (ENC_PPR * ENC_QUAD_FACTOR * ENC_GEAR_RATIO)

/* ==== 标定补偿系数 ==========================================================
   里程的最终换算是
       mm = counts × (π·D / COUNTS_PER_REV) × (CAL_NUM / CAL_DEN)
   标定方法：推着车直线走 1000mm（尺子量实际位移），读 Encoder_GetDistanceAvg()
   得到 measured，则填 ENC_CAL_NUM=1000、ENC_CAL_DEN=measured。
   用整数分数而不是 float，是因为 F103 没有硬件 FPU。
   ============================================================================ */
#define ENC_CAL_NUM          5L
#define ENC_CAL_DEN          7L

/* ==== 计数方向补偿 ==========================================================
   编码器计数的正方向由 A/B 相接线和电机安装朝向共同决定，和车体前进方向
   没有必然关系。左右两侧电机是镜像安装的（参见 car.c 的 CAR_LEFT_POLARITY），
   所以两侧编码器很可能也是反的。
   标定方法：让车前进，四个 Encoder_GetCount() 应该**全为正**。
   哪个是负的，就把对应的宏取反。
   ============================================================================ */
#define ENC_POLARITY_M1      (1)   /* 左前 */
#define ENC_POLARITY_M2      (1)   /* 左后 */
#define ENC_POLARITY_M3      (-1)    /* 右前 */
#define ENC_POLARITY_M4      (-1)    /* 右后 */

/* 转速计算的最小采样间隔 ms。间隔太短则一个周期内脉冲数太少，转速值会
   跳得很厉害；太长则响应迟钝。20ms 时 100rpm 下约有 17 个计数（x2 倍频），
   够用了。低于这个间隔的 Update 只累加里程、不更新转速。 */
#define ENC_SPEED_PERIOD_MS  20U

/* Exported functions prototypes ---------------------------------------------*/
void    Encoder_Init(void);
void    Encoder_Update(void);

int32_t Encoder_GetCount(Motor_ID motor);
int32_t Encoder_GetDistance(Motor_ID motor);
int16_t Encoder_GetSpeedRpm(Motor_ID motor);

int32_t Encoder_GetDistanceAvg(void);
int32_t Encoder_GetDistanceLeft(void);
int32_t Encoder_GetDistanceRight(void);

void    Encoder_Reset(Motor_ID motor);
void    Encoder_ResetAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */
