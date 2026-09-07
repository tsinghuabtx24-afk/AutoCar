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

/* ==== 固定绕行序列 ==========================================================
   七步，右绕：

       右转 60° → 前进 400 → 左转 60° → 前进 100
                → 左转 60° → 前进 400 → 右转 60°

   四次旋转代数和 = -60 +60 +60 -60 = 0，**车头朝向回到原样**。
   横向位移 = 400·sin60° - 400·sin60° = 0，**横向也回到原线**，中途最大偏出
   约 400·sin60° ≈ 346mm。所以绕完直接交还控制权，循迹能接上原来那条线。

   中间那段 100mm 是横跨障碍的宽度余量：偏出 346mm 之后车头朝向已回正（与原
   方向平行），这一段是平行于原线走的，走多远就等于给障碍留多宽。障碍更宽就
   加大它，不影响回正几何。

   ⚠️ 序列是开环的：只按编码器走，不看传感器。绕行途中不判断障碍是否还在，
   也不判断新障碍——这是"固定序列"的定义。安全兜底只有急停、手动接管和总超时。
   ============================================================================ */

/* 每一步的旋转角与前进距离。角度用 _DEG10（600 = 60.0°）。 */
#define AVOID_DETOUR_TURN_DEG10   900U
#define AVOID_DETOUR_LONG_MM      300U
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

/* 绕行序列的总超时。七步的理论耗时：四次 60° 旋转 + 900mm 直行，按实测
   速度算大约 6~8 s。给到 12 s 留足余量，到期只是结束本轮并交还控制权。 */
#define AVOID_DETOUR_TIMEOUT_MS   12000U

typedef enum
{
  AVOID_IDLE = 0,
  AVOID_BACKING,   /* 后退让出空间（旧策略） */
  AVOID_TURNING,   /* 转向避开（旧策略） */
  AVOID_VERIFY,    /* 等新采样，确认是否脱困（旧策略） */
  AVOID_DETOUR,    /* 固定绕行序列进行中，具体第几步见 AvoidTask_GetStep() */
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
