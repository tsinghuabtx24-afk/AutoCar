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
#define VISION_SPEED_NORMAL        85U
#define VISION_SPEED_LIMIT_DROP    20U
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

/* ==== 不占底盘的"信号类"动作 ================================================
   隧道 / 起伏路段 / 友军 都不接管底盘：车继续循迹，只叠加灯光、限速或声音。
   所以它们不进 VisionTask_Step() 的状态机，而由 VisionTask_Tick() 每轮推进，
   与当前控制模式无关（急停时由 VisionTask_Cancel() 一并清掉）。
   ============================================================================ */

/* 隧道：常亮白灯的时长。 */
#define VISION_TUNNEL_LIGHT_MS     4000U

/* 起伏路段：减速通过的时长，到点自动恢复原速。 */
#define VISION_ROUGH_SLOW_MS       5000U

/* 友军：RGB 短闪一次的亮灯时长。 */
#define VISION_FRIENDLY_FLASH_MS   200U

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

/*
 * 推进信号类动作（隧道白灯、起伏路段限速、友军闪灯+旋律）。
 * 必须每轮主循环调用一次，且不分控制模式——这些动作与底盘无关，
 * 只在 CONTROL_TASK 模式里推进会导致循迹时白灯永不灭、限速永不恢复。
 */
void VisionTask_Tick(void);

/* 当前巡线速度，受限速/解除限速事件影响。 */
uint8_t VisionTask_GetSpeed(void);

VisionTask_State VisionTask_GetState(void);
const char *VisionTask_StateName(VisionTask_State state);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_TASK_H__ */
