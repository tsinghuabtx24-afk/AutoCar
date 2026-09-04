#ifndef __IR_REMOTE_H__
#define __IR_REMOTE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef struct
{
  uint8_t address;
  uint8_t command;
  uint8_t inverse_command;
} IrRemote_Frame;

void IrRemote_Init(void);
void IrRemote_EXTI_Callback(uint16_t GPIO_Pin);
uint8_t IrRemote_Read(IrRemote_Frame *frame);
uint8_t IrRemote_IsRepeat(void);

#ifdef __cplusplus
}
#endif

#endif /* __IR_REMOTE_H__ */
