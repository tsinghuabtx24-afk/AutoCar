/**
  ******************************************************************************
  * @file    manual_task.h
  * @brief   红外遥控手动接管（调试用）
  ******************************************************************************
  * @note    定位：调试手段，优先级仅低于急停。
  *
  *          进入 / 退出只认 RED 键连按两次。不在手动模式时，方向键和 HORN
  *          全部忽略，自动驾驶不受干扰。
  *
  *          "松手停车"和"退出手动"是两件事：按键静默超时只是停车，仍留在
  *          手动模式；只有双击 RED 才交还控制权。这样遥控丢帧只会让车停下，
  *          不会把控制权莫名交还给自动驾驶。
  ******************************************************************************
  */
#ifndef __MANUAL_TASK_H__
#define __MANUAL_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "event.h"

/* RED 键双击判定窗口。两次 RED 间隔小于此值算一次双击。
   NEC 长按发的 repeat 帧不计入次数，否则按住 RED 会被误判成连击。 */
#define MANUAL_DOUBLE_MS        600U

/* 按键静默超时。超过此时长没有新按键或 repeat 帧就停车，但仍留在手动模式。
   300ms 约为一个 NEC repeat 间隔（约 108ms）的三倍，留足余量。 */
#define MANUAL_KEY_TIMEOUT_MS   300U

/* 手动模式的行驶速度与转向差速。 */
#define MANUAL_SPEED            70U
#define MANUAL_ROTATE_SPEED     70U

void ManualTask_Init(void);

/* 每轮主循环调用：读遥控帧，解码成事件投递到队列。
   注意：不在正式主循环调用 IrRemoteDebug_Update()，它会刷 OLED（阻塞 I2C）。
   无论当前是否手动模式都要调用——RED 双击是进入手动模式的唯一入口。 */
void ManualTask_Sense(void);

/* 调度器在进入 / 退出手动模式时调用。 */
void ManualTask_Enter(void);
void ManualTask_Exit(void);

/* 调度器把遥控动作事件转交给手动模块。 */
void ManualTask_OnEvent(Event_Type type);

/* 手动模式下每轮推进：按当前按键驱动底盘，静默超时则停车。 */
void ManualTask_Step(void);

#ifdef __cplusplus
}
#endif

#endif /* __MANUAL_TASK_H__ */
