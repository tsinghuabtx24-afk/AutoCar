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
  *          定时器工作在正交解码模式，CNT 由 A/B 相直接驱动、正反转自动加减，
  *          不占 CPU。本模块只把 CNT 的增量周期性累加成 32 位里程并算转速。
  *
  *          用法：Encoder_Init() 之后周期调 Encoder_Update()，再用
  *          Encoder_GetDistance() / Encoder_GetSpeedRpm() 取值。
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

/* ==== 物理参数 ==============================================================
   下面四个是**名义值**，不是标定结果。它们只为保留物理含义方便排查，实测误差
   统一由 ENC_CAL_NUM/DEN 吸收——不要改这里来"凑准"，否则出偏差时分不清是哪
   一环错的。
   ============================================================================ */

/* 单相每转脉冲数。MG310 常见 11 或 13，光电码盘才是 500。实测法：轮子架空手转
   10 圈，Encoder_GetCount() 除以 10 得到的是 ENC_COUNTS_PER_REV。 */
#define ENC_PPR              13U

/* 正交倍频系数，必须与 CubeMX 的 Encoder Mode 一致，否则里程差一倍：
       TI1 或 TI2      → 2（只数一相边沿）
       TI1 and TI2     → 4（两相都数，分辨率最高） */
#define ENC_QUAD_FACTOR      2U

/* 减速比，一般印在减速箱侧面。MG310 常见 20/30/45/100。 */
#define ENC_GEAR_RATIO       20U

/* 轮径 mm。卷尺量比查手册准，橡胶轮压扁后有效直径会略小。 */
#define ENC_WHEEL_DIA_MM     65U

/* 实测车轮转一圈约 521 count，里程和转速统一使用该值。 */
#define ENC_COUNTS_PER_REV   521U

/* ==== 标定补偿系数 ==========================================================
   里程换算：mm = counts × (π·D / COUNTS_PER_REV) × (CAL_NUM / CAL_DEN)
   标定：推车直线走 1000mm（尺子量实际位移），读 Encoder_GetDistanceAvg() 得
   measured，则 ENC_CAL_NUM=1000、ENC_CAL_DEN=measured。
   用整数分数而非 float，因为 F103 没有硬件 FPU。
   ============================================================================ */
#define ENC_CAL_NUM          5L
#define ENC_CAL_DEN          7L

/* ==== 计数方向补偿 ==========================================================
   计数正方向由 A/B 相接线和电机安装朝向决定，与车体前进方向没有必然关系；左右
   电机镜像安装（见 car.c 的 CAR_LEFT_POLARITY），两侧编码器很可能也是反的。
   标定：让车前进，四个 Encoder_GetCount() 应**全为正**，哪个是负的就把对应宏取反。
   ============================================================================ */
#define ENC_POLARITY_M1      (1)   /* 左前 */
#define ENC_POLARITY_M2      (1)   /* 左后 */
#define ENC_POLARITY_M3      (-1)    /* 右前 */
#define ENC_POLARITY_M4      (-1)    /* 右后 */

/* 转速计算的最小采样间隔 ms：太短则一周期内脉冲太少、转速跳得厉害，太长则响应
   迟钝。20ms 在 100rpm 下约 17 个计数（x2 倍频），够用。间隔不足时 Update 只
   累加里程、不更新转速。 */
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

/* 按左右轮有符号里程差估算车体转角，单位 0.1 度，正=左转。
   wheel_track_mm 为左右轮中心距。 */
int32_t Encoder_GetRotationDeg10(uint16_t wheel_track_mm);

void    Encoder_Reset(Motor_ID motor);
void    Encoder_ResetAll(void);

#ifdef __cplusplus
}
#endif

#endif /* __ENCODER_H__ */
