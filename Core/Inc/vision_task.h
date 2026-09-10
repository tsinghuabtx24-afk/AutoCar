#ifndef __VISION_TASK_H__
#define __VISION_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "vision_uart.h"
#include "event.h"

/* ==== 速度配置 ==============================================================
   巡线默认速度与限速后的速度。速度刻度沿用 car 模块的 0~100 占空比刻度。
   ============================================================================ */
#define VISION_SPEED_NORMAL        72U
#define VISION_SPEED_LIMIT_DROP    12U
#define VISION_SPEED_FULL          100U

/* _Static_assert 见 vision_task.c：限速后的速度必须仍在死区之上。
   注意 VISION_SPEED_NORMAL 应与 LINE_BASE_SPEED 一致，否则"解除限速"会把
   循迹速度改成另一个值。 */

/* 转向：原地旋转 90.0°，单位 0.1 度。 */
#define VISION_TURN_SPEED          80U
#define VISION_TURN_ANGLE_DEG10    900U

/* 鸣笛：响 0.5s，间隔 0.2s，共两声。 */
#define VISION_HORN_ON_MS          500U
#define VISION_HORN_GAP_MS         200U
#define VISION_HORN_COUNT          2U

/* 入库：闪灯周期与总时长。 */
#define VISION_PARK_BLINK_MS       200U
#define VISION_PARK_BLINK_TOTAL_MS 2000U

/* 2 号库：闪灯后满速前进的时间。 */
#define VISION_PARK2_BOOST_MS      2000U

typedef enum
{
  VISION_TASK_IDLE = 0,
  VISION_TASK_TURN_LEFT,
  VISION_TASK_TURN_RIGHT,
  VISION_TASK_HORN,
  VISION_TASK_PARK_1,
  VISION_TASK_PARK_2,
  VISION_TASK_STOPPED
} VisionTask_State;

void VisionTask_Init(void);

/*
 * 按事件创建任务。由调度器调用。
 * 返回 1：确实起了一个占用底盘的任务，调度器应切到 CONTROL_TASK；
 * 返回 0：事件只改配置（限速）或被忽略（同类任务已在跑），模式不变。
 */
uint8_t VisionTask_Begin(Event_Type type);

/*
 * 推进当前任务一步。由调度器在 CONTROL_TASK 模式下每轮调用。
 * 返回 1：仍占用底盘；
 * 返回 0：已完成，调度器应交回循迹。
 */
uint8_t VisionTask_Step(void);

/* 放弃当前任务并制动。急停时由调度器调用。 */
void VisionTask_Cancel(void);

uint8_t VisionTask_IsBusy(void);

/* 当前巡线速度，受限速/解除限速事件影响。 */
uint8_t VisionTask_GetSpeed(void);

VisionTask_State VisionTask_GetState(void);
const char *VisionTask_StateName(VisionTask_State state);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_TASK_H__ */
