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
#define VISION_SPEED_NORMAL        58U
#define VISION_SPEED_LIMIT_DROP    10U
#define VISION_SPEED_FULL          100U

/* 减速档。限速标志与**鸣笛标志**都用它，写成宏避免三处重复同一个表达式。 */
#define VISION_SPEED_LIMITED  \
  ((uint8_t)(VISION_SPEED_NORMAL - VISION_SPEED_LIMIT_DROP))

/* _Static_assert 见 vision_task.c：限速后的速度必须仍在死区之上。
   注意 VISION_SPEED_NORMAL 应与 LINE_BASE_SPEED 一致，否则"解除限速"会把
   循迹速度改成另一个值。 */

/* 转向：原地旋转 90.0°，单位 0.1 度。 */
#define VISION_TURN_SPEED          80U
#define VISION_TURN_ANGLE_DEG10    900U

/* ==== 鸣笛 ==================================================================
   响 0.5s，间隔 0.2s，共两声。

   鸣笛**不占用底盘、不停车**：它只是叫两声，车照常循迹。所以它不是一个"任务"。

   **但它会降速**：识别到鸣笛标志（线上 id 4 与 id 7，两者都映射到
   EVENT_VISION_HORN）之后按 VISION_SPEED_LIMITED 行驶，效果与限速标志相同。
   降速是持续的，不随鸣笛声结束而恢复——恢复只由"解除限速"或绿灯起步触发。
   ============================================================================ */
#define VISION_HORN_ON_MS          500U
#define VISION_HORN_GAP_MS         200U
#define VISION_HORN_COUNT          2U

/* 入库：闪灯周期与总时长。 */
#define VISION_PARK_BLINK_MS       200U
#define VISION_PARK_BLINK_TOTAL_MS 2000U

/* 2 号库：闪灯后前进的时间。 */
#define VISION_PARK2_BOOST_MS      2000U

/* ==== 1 号库（红灯）的最长停车时间 ==========================================
   红灯停稳后**最多等这么久**：期间等到绿灯（2 号库）就正常起步；一直没等到，
   到期自动起步，起步速度与绿灯**完全相同**（同一段代码，见
   VisionTask_StartRolling）。防的是赛场上绿灯没识别到、车在库里停到比赛结束。

   起算点是**进入停止态的时刻**，不含前面那 2s 闪灯——这个值就是纯粹的
   "停多久"，标定时只调一个量。

   0 = 关掉这个兜底，回到旧行为：无限等绿灯。 */
#define VISION_PARK1_MAX_STOP_MS   10000U

/* 超时自动起步后，这么长时间内忽略 1 号库信号。
   车起步时**仍停在那块红灯牌跟前**，摄像头还在反复报 1 号库；不压制的话下一帧
   就把车重新停回去，超时起步等于没生效。

   ⚠️ 必须 > VNAV_ACCEPT_COOLDOWN_MS(1500)：视觉导航层的去重只压 1.5s，到期后
      那块牌子会被重新放行。取 3000ms：够车驶离标志牌视野。*/
#define VISION_PARK1_RESTART_IGNORE_MS 3000U

/* ==== 1 号库整局只执行一次 ==================================================
   一次上电（复位）之内最多执行几次 1 号库。1 = **整局只入库一次**，之后再识别
   到 1 号库一律忽略，车照常循迹开过去；0 = 不限次数（旧行为）。

   急停打断入库也算用掉了这一次，只有 VisionTask_Init() 清零，也就是**只有复位/重新上电才重新给一次**。 */
#define VISION_PARK1_MAX_EXEC      1U

/* ==== 绿灯（2 号库）的起步速度 ==============================================
   2 号库标志就是**绿灯**：1 号库（红灯）把车停住，只有它能让车重新起步。

   0 = **正常速度** VISION_SPEED_NORMAL 起步（当前行为）。
       即使停之前正在减速档（限速标志或鸣笛标志造成的），绿灯起步也回到正常
       速度——红灯前的减速理由到绿灯就结束了，不该带着减速档跑完剩下的赛道。
   1 = VISION_SPEED_FULL(100) 满速冲。

   ⚠️ 早先的行为是"沿用停在 1 号库之前的那个速度"（速度快照）。那个逻辑已
      删除：鸣笛降速之后再遇红绿灯，会带着减速档起步，不是想要的结果。
   ============================================================================ */
#define VISION_PARK2_FULL_SPEED    0U

typedef enum
{
  VISION_TASK_IDLE = 0,
  VISION_TASK_TURN_LEFT,
  VISION_TASK_TURN_RIGHT,
  /* 鸣笛不再是任务（不占底盘），所以这里没有 VISION_TASK_HORN。 */
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

/*
 * 推进鸣笛节奏。**每轮主循环都要调**，与控制模式无关——鸣笛不占底盘，所以它
 * 不能挂在 VisionTask_Step() 里（那个只在 CONTROL_TASK 模式下被调用）。
 * 没有在鸣笛时是空操作。
 */
void VisionTask_StepHorn(void);

/* 放弃当前任务并制动。急停时由调度器调用。鸣笛也一并停掉。 */
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
