#ifndef __IR_REMOTE_H__
#define __IR_REMOTE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ==== 帧队列 ================================================================
   原实现只有一个帧槽（data + ready 标志），连按时后一帧会覆盖前一帧，且无人
   知情。改成小环形队列：解码完成入队，主循环每轮取。

   深度 4 够用：主循环每轮几十微秒，而 NEC 一帧要 67ms，正常情况下队列里
   最多躺着一帧。深度 4 是给主循环被长动作占住时留的余量。
   ============================================================================ */
#define IR_REMOTE_QUEUE_LEN   4U

typedef struct
{
  uint8_t address;
  uint8_t command;
  uint8_t inverse_command;
} IrRemote_Frame;

void IrRemote_Init(void);
void IrRemote_EXTI_Callback(uint16_t GPIO_Pin);

/* 取一帧。队列空返回 0。校验失败的帧在入队时就被丢掉，取到的一定是好帧。 */
uint8_t IrRemote_Read(IrRemote_Frame *frame);

uint8_t IrRemote_IsRepeat(void);

/* 诊断：队列满而丢弃的帧数。正常应为 0。 */
uint32_t IrRemote_GetDroppedCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __IR_REMOTE_H__ */
