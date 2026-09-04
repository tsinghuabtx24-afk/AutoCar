#ifndef __IR_REMOTE_MAP_H__
#define __IR_REMOTE_MAP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum
{
  IR_REMOTE_ACTION_NONE = 0,
  IR_REMOTE_ACTION_FORWARD,
  IR_REMOTE_ACTION_ROTATE_LEFT,
  IR_REMOTE_ACTION_ROTATE_RIGHT,
  IR_REMOTE_ACTION_HORN
} IrRemote_Action;

const char *IrRemote_KeyName(uint8_t command);
IrRemote_Action IrRemote_GetAction(uint8_t command);
const char *IrRemote_ActionName(IrRemote_Action action);

#ifdef __cplusplus
}
#endif

#endif /* __IR_REMOTE_MAP_H__ */
