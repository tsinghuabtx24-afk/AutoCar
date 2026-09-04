#ifndef __VISION_TASK_H__
#define __VISION_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "vision_uart.h"

/* ==== 速度配置 ==============================================================
   巡线默认速度与限速后的速度。速度刻度沿用 car 模块的 0~100 占空比刻度。
   ============================================================================ */
#define VISION_SPEED_NORMAL        70U
#define VISION_SPEED_LIMIT_DROP    10U
#define VISION_SPEED_FULL          100U

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
 * 处理一次视觉事件与当前任务。
 * 返回 1：视觉任务占用底盘，调用方不得驱动电机；
 * 返回 0：没有任务占用，调用方可运行默认行为（循迹或直行）。
 */
uint8_t VisionTask_Handle(void);

/* 当前巡线速度，受限速/解除限速事件影响。 */
uint8_t VisionTask_GetSpeed(void);

VisionTask_State VisionTask_GetState(void);
const char *VisionTask_StateName(VisionTask_State state);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_TASK_H__ */
