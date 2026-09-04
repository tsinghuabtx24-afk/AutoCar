/**
  ******************************************************************************
  * @file    scheduler.h
  * @brief   整车控制模式调度
  ******************************************************************************
  * @note    调度器只做两件事：消费事件改变控制模式，按模式把控制权交给唯一
  *          一个行为模块。它不读传感器，也不直接写电机。
  *
  *          优先级（高→低）：
  *              STOP > MANUAL > IR_AVOID > TASK > LINE_TRACK > IDLE
  *
  *          MANUAL 高于 IR_AVOID 是有意的：遥控是调试手段，按下就该立刻拿到
  *          底盘。代价是手动模式下避障不再保护车体，急停是唯一兜底。
  ******************************************************************************
  */
#ifndef __SCHEDULER_H__
#define __SCHEDULER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "event.h"

typedef enum
{
  CONTROL_IDLE = 0,    /* 未启动，等待启动键 */
  CONTROL_LINE_TRACK,  /* 默认行为：循迹 */
  CONTROL_TASK,        /* 视觉任务占用底盘 */
  CONTROL_IR_AVOID,    /* 避障占用底盘 */
  CONTROL_MANUAL,      /* 红外遥控手动接管（调试用） */
  CONTROL_STOP         /* 急停 / 故障 */
} Control_Mode;

/* 上电后的初始模式。
   当前独立测试阶段沿用"上电即循迹"，与改造前行为一致。要改成"等启动键"
   就把这里换成 CONTROL_IDLE，届时需要按 key 或遥控发 EVENT_KEY_START。 */
#define SCHEDULER_START_MODE   CONTROL_LINE_TRACK

void Scheduler_Init(void);

/* 消费一个事件，可能改变控制模式。 */
void Scheduler_OnEvent(const Event *e);

/* 把队列里所有事件消费完。 */
void Scheduler_DrainEvents(void);

/* 按当前模式把控制权交给对应行为模块，本轮只有它能写电机。 */
void Scheduler_Dispatch(void);

Control_Mode Scheduler_GetMode(void);
Control_Mode Scheduler_GetPreemptedMode(void);

/* 供行为模块在任务完成时调用，交还控制权。 */
void Scheduler_TaskFinished(void);

/* 供避障模块在避障动作完成时调用，回到被抢占的模式。 */
void Scheduler_AvoidFinished(void);

const char *Scheduler_ModeName(Control_Mode mode);

#ifdef __cplusplus
}
#endif

#endif /* __SCHEDULER_H__ */
