/**
  ******************************************************************************
  * @file    avoid_task.h
  * @brief   避障行为状态机
  ******************************************************************************
  * @note    避障的**行为**全在这里，传感器在 ir_avoid / ultrasonic_sense。
  *          只有拿到 CONTROL_IR_AVOID 控制权时才推进，动作全部非阻塞。
  *
  *          相比原实现的三处改动：
  *
  *          1. **转向方向不再随机。** 原来双侧障碍时用伪随机选左右，同一处
  *             障碍反复试错、行为不可复现，调试时无法判断是策略错还是运气差。
  *             现在按两侧红外原始值择路：往回波更弱（更空旷）的一侧转。
  *          2. **加了 VERIFY 相位。** 转完先重新采样确认真的让开了，没让开就
  *             再试，而不是盲目认为一次旋转必然有效。
  *          3. **加了重试上限和故障出口。** 连续 AVOID_MAX_ATTEMPTS 次仍未脱困
  *             就投 EVENT_FAULT，由调度器进急停，而不是原地无限打转。
  ******************************************************************************
  */
#ifndef __AVOID_TASK_H__
#define __AVOID_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==== 避障动作参数（需按实车、电池、场地标定）========================== */

/* 后退：仅在正前方或双侧受阻时执行。 */
#define AVOID_REVERSE_SPEED       70U
#define AVOID_REVERSE_DIST_MM     80U

/* 转向：速度必须不低于 CAR_ANGLE_MIN_SPEED，否则滑移补偿表不适用。 */
#define AVOID_ROTATE_SPEED        80U
#define AVOID_ROTATE_DEG10        450U   /* 45.0 度 */

/* 单侧受阻时的转向角，比脱困转向小一些。 */
#define AVOID_NUDGE_DEG10         300U   /* 30.0 度 */

/* 重新采样确认的等待时间。要覆盖至少一轮完整红外采样周期。 */
#define AVOID_VERIFY_MS           120U

/* 脱困重试上限。超过就报故障。 */
#define AVOID_MAX_ATTEMPTS        4U

/* 整个避障过程的总超时，防止相位状态机卡死。 */
#define AVOID_TOTAL_TIMEOUT_MS    15000U

typedef enum
{
  AVOID_IDLE = 0,
  AVOID_BACKING,   /* 后退让出空间 */
  AVOID_TURNING,   /* 转向避开 */
  AVOID_VERIFY,    /* 等新采样，确认是否脱困 */
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

#ifdef __cplusplus
}
#endif

#endif /* __AVOID_TASK_H__ */
