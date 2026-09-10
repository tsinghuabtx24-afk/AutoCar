/**
  ******************************************************************************
  * @file    avoid_task.h
  * @brief   避障行为状态机
  ******************************************************************************
  * @note    避障的**行为**全在这里，传感器在 ir_avoid / ultrasonic_sense。
  *          只有拿到 CONTROL_IR_AVOID 控制权时才推进，动作全部非阻塞。
  *
  *          当前策略是**固定绕行序列**（AVOID_STRATEGY_DETOUR = 1）：不再"试一下
  *          看让开没让开"，而是一次走完一条绕过障碍并回到原线的路径。
  *
  *          旧的自适应策略（后退 → 按 ADC 择路 → VERIFY 确认 → 不行就重试）
  *          保留在 #else 分支里，把开关改成 0 即可切回，不用翻 git。
  ******************************************************************************
  */
#ifndef __AVOID_TASK_H__
#define __AVOID_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==== 策略开关 ==============================================================
   1 = 固定绕行序列（当前）：前方或左方受阻时走一条绕过去再回来的路径。
   0 = 旧的自适应策略：后退 → 按 ADC 择路 → VERIFY 确认 → 不行就重试。

   两套逻辑都编在文件里，切换只改这一处。
   ============================================================================ */
#define AVOID_STRATEGY_DETOUR     1U

/* ==== 绕行序列 ==============================================================
   五步开环 + 末段巡航，右绕：

       右转 90° → 前进 300 → 左转 80° → 前进 500 → 左转 80°
                → 持续前进直到见线（见下面的 AVOID_CRUISE_*）

   中间那段 500mm 是横跨障碍的宽度余量，障碍更宽就加大它。

   ⚠️ 开环五步只按编码器走，不看传感器，途中不判断障碍是否还在也不判断新障碍。
      安全兜底只有急停、手动接管和总超时。
   ============================================================================ */

/* 每一步的旋转角与前进距离。角度用 _DEG10（600 = 60.0°）。 */
#define AVOID_DETOUR_TURN_DEG10   900U
#define AVOID_DETOUR_LONG_MM      270U
#define AVOID_DETOUR_CROSS_MM     500U

/* 绕行用的速度。旋转速度必须 ≥ CAR_ANGLE_MIN_SPEED(55)，否则 car 拒绝动作
   （滑移补偿表在低速下不适用）。 */
#define AVOID_DETOUR_ROTATE_SPEED 80U
#define AVOID_DETOUR_FORWARD_SPEED 70U

/* ==== 避障动作参数（需按实车、电池、场地标定）========================== */

/* 后退：仅在正前方或双侧受阻时执行。 */
#define AVOID_REVERSE_SPEED       70U
#define AVOID_REVERSE_DIST_MM     80U

/* 本轮避障最多后退几次。重试次数不设上限，但后退要设——正对墙时每次尝试都会
   判"需要后退"，不限制的话 15 秒能累计倒退一米多，可能直接退出赛道。
   超过这个次数后只原地转，不再后退。 */
#define AVOID_MAX_REVERSES        3U

/* 转向：速度必须不低于 CAR_ANGLE_MIN_SPEED，否则滑移补偿表不适用。 */
#define AVOID_ROTATE_SPEED        80U
#define AVOID_ROTATE_DEG10        450U   /* 45.0 度 */

/* 单侧受阻时的转向角，比脱困转向小一些。 */
#define AVOID_NUDGE_DEG10         300U   /* 30.0 度 */

/* 重新采样确认的等待时间。要覆盖至少一轮完整红外采样周期。 */
#define AVOID_VERIFY_MS           120U

/* ==== 不设重试上限 ==========================================================
   避障失败**不是故障**。障碍不会四面八方同时存在，真挡住了也一定是暂时的
   （有人站着、放了个箱子），所以持续重试直到脱困，而不是判故障锁死。

   原实现连续 4 次未脱困就投 EVENT_FAULT 进急停，后果是：障碍拿开车也不会自己
   继续，必须按键；而障碍若仍在，按键换来的 2~3 秒后又会急停，形成死循环。

   总超时仍然保留，但**到期只是结束本轮避障并交还控制权**，不报故障。
   障碍还在的话传感器下一轮会重新触发，等于自动重来；这期间车是停着的。
   ============================================================================ */
#define AVOID_TOTAL_TIMEOUT_MS    15000U

/* 开环五步的总超时。三次旋转 + 800mm 直行，实测约 6~8 s，给到 12 s 留余量。
   到期只是结束本轮并交还控制权。末段巡航用自己的 AVOID_CRUISE_TIMEOUT_MS。 */
#define AVOID_DETOUR_TIMEOUT_MS   12000U

/* ==== 末段巡航 ==============================================================
   序列最后一步不走固定距离，而是持续前进直到循迹传感器见线，见线即交还控制权。
   固定距离要求"走完刚好落在线上"，而线的位置取决于障碍位置和前几步的累计误差。

   仍然有距离上限：线被绕丢时不至于一路开出赛场。到上限只是结束本轮并交还
   控制权，不报故障。
   ============================================================================ */
#define AVOID_CRUISE_ENABLE       1U

#define AVOID_CRUISE_SPEED        65U
#define AVOID_CRUISE_MAX_MM      600U

/* 巡航超时。car 对距离动作的兜底是 CAR_DISTANCE_TIMEOUT_MS(60s)，轮子被卡住时
   太长。600mm 在 PWM 65 下约 2~3s。 */
#define AVOID_CRUISE_TIMEOUT_MS  6000U

/* ==== 见线判定 ==============================================================
   全白(0x0F)以外的任何图案都算见到线，全黑(0x00)也算——它出现在路口和黑区。

   ⚠️ 必须防抖。单次毛刺就结束巡航的后果不轻：交还控制权后循迹读到全白，要数满
      LINE_LOST_COUNT（200 × 10ms = 2s）才判丢线急停，这 2 秒里车带着轮速在冲。
   ============================================================================ */
#define AVOID_CRUISE_SAMPLE_MS    2U
#define AVOID_CRUISE_CONFIRM      3U

typedef enum
{
  AVOID_IDLE = 0,
  AVOID_BACKING,   /* 后退让出空间（旧策略） */
  AVOID_TURNING,   /* 转向避开（旧策略） */
  AVOID_VERIFY,    /* 等新采样，确认是否脱困（旧策略） */
  AVOID_DETOUR,    /* 固定绕行序列进行中，具体第几步见 AvoidTask_GetStep() */
  AVOID_CRUISE,    /* 末段持续前进，见线即结束 */
  AVOID_DONE,
  AVOID_FAILED
} AvoidTask_State;

void AvoidTask_Init(void);

/* 调度器切入 CONTROL_IR_AVOID 时调用一次。 */
void AvoidTask_Begin(void);

/*
 * 每轮推进。返回 1=仍占用底盘，0=已结束（调度器应交还控制权）。
 */
uint8_t AvoidTask_Step(void);

/* 急停时放弃避障。 */
void AvoidTask_Cancel(void);

AvoidTask_State AvoidTask_GetState(void);
const char *AvoidTask_StateName(AvoidTask_State state);

/* 绕行序列当前第几步（0~6），供诊断查看。非绕行策略下恒为 0。 */
uint8_t AvoidTask_GetStep(void);


#ifdef __cplusplus
}
#endif

#endif /* __AVOID_TASK_H__ */
