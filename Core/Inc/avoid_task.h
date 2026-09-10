/**
  ******************************************************************************
  * @file    avoid_task.h
  * @brief   避障行为状态机
  ******************************************************************************
  * @note    避障的**行为**全在这里，传感器在 ir_avoid / ultrasonic_sense。
  *          只有拿到 CONTROL_IR_AVOID 控制权时才推进，动作全部非阻塞。
  *
  *          两套策略都编在文件里，切换只改 AVOID_STRATEGY_DETOUR 一处，不用翻 git：
  *            1 = 固定绕行序列（当前）：一次走完绕过障碍再回到原线的路径。
  *            0 = 旧的自适应策略：后退 → 按 ADC 择路 → VERIFY 确认 → 不行就重试。
  ******************************************************************************
  */
#ifndef __AVOID_TASK_H__
#define __AVOID_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define AVOID_STRATEGY_DETOUR     1U

/* ==== 固定绕行序列 ==========================================================
   七步右绕：右转 → 直行 LONG → 左转 → 直行 CROSS → 左转 → 直行 LONG → 右转。

   两个几何恒等式保证绕完能接回原来那条线，调参时别破坏：
     - 四次转角代数和 = -60 +60 +60 -60 = 0   → 车头朝向回正
     - 两段 LONG 等长，横向偏出与收回抵消     → 横向回到原线
   途中最大偏出 LONG·sin60°。中间 CROSS 段是平行于原线走的（此时朝向已回正），
   走多远就等于给障碍留多宽；障碍更宽只加大 CROSS，不影响上面两条。

   ⚠️ 开环：只按编码器走，途中不看传感器——这就是"固定序列"的定义，既不判断
   障碍是否还在，也不判断新障碍。兜底只有急停、手动接管和总超时。
   ============================================================================ */

/* 每一步的旋转角与前进距离。角度用 _DEG10（600 = 60.0°）。 */
#define AVOID_DETOUR_TURN_DEG10   600U
#define AVOID_DETOUR_LONG_MM      200U
#define AVOID_DETOUR_CROSS_MM     200U

/* 旋转速度必须 ≥ CAR_ANGLE_MIN_SPEED(55)，否则 car 拒绝动作（滑移补偿表在低速
   下不适用）。下面 AVOID_ROTATE_SPEED 同理。 */
#define AVOID_DETOUR_ROTATE_SPEED 80U
#define AVOID_DETOUR_FORWARD_SPEED 70U

/* ==== 避障动作参数（需按实车、电池、场地标定）========================== */

/* 后退：仅在正前方或双侧受阻时执行。 */
#define AVOID_REVERSE_SPEED       70U
#define AVOID_REVERSE_DIST_MM     80U

/* 后退次数要设上限（重试次数不设，见下）：正对墙时每次尝试都判"需要后退"，
   不限制的话 15 秒能累计倒退一米多，可能直接退出赛道。超过后只原地转。 */
#define AVOID_MAX_REVERSES        3U

#define AVOID_ROTATE_SPEED        80U
#define AVOID_ROTATE_DEG10        450U   /* 45.0°，脱困转向 */
#define AVOID_NUDGE_DEG10         300U   /* 30.0°，单侧受阻时轻推，比脱困小 */

/* 重新采样确认的等待时间，要覆盖至少一轮完整红外采样周期。 */
#define AVOID_VERIFY_MS           120U

/* ==== 超时：到期只结束本轮并交还控制权，不报故障 ============================
   避障失败**不是故障**。障碍不会四面八方同时存在，真挡住也一定是暂时的（有人
   站着、放了个箱子），所以重试次数不设上限，持续重试直到脱困。

   原实现连续 4 次未脱困就投 EVENT_FAULT 进急停，后果是：障碍拿开车也不会自己
   继续，必须按键；障碍仍在时按键换来的 2~3 秒后又急停，形成死循环。

   现在到期只是交还控制权，车停着；障碍若仍在，传感器下一轮会重新触发，等于
   自动重来。绕行序列用自己更短的超时——开环固定路径的理论耗时可算，卡住就该
   早点结束，不必等 15 秒。
   ============================================================================ */
#define AVOID_TOTAL_TIMEOUT_MS    15000U
#define AVOID_DETOUR_TIMEOUT_MS   12000U

typedef enum
{
  AVOID_IDLE = 0,
  AVOID_BACKING,   /* 后退让出空间（旧策略） */
  AVOID_TURNING,   /* 转向避开（旧策略） */
  AVOID_VERIFY,    /* 等新采样，确认是否脱困（旧策略） */
  AVOID_DETOUR,    /* 固定绕行序列进行中，第几步见 AvoidTask_GetStep() */
  AVOID_DONE,
  AVOID_FAILED
} AvoidTask_State;

void AvoidTask_Init(void);

/* 调度器切入 CONTROL_IR_AVOID 时调用一次。 */
void AvoidTask_Begin(void);

/* 每轮推进。返回 1=仍占用底盘，0=已结束（调度器应交还控制权）。 */
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
