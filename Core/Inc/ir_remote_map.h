#ifndef __IR_REMOTE_MAP_H__
#define __IR_REMOTE_MAP_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
  IR_REMOTE_ACTION_NONE = 0,
  IR_REMOTE_ACTION_FORWARD,
  IR_REMOTE_ACTION_BACKWARD,
  IR_REMOTE_ACTION_ROTATE_LEFT,
  IR_REMOTE_ACTION_ROTATE_RIGHT,
  IR_REMOTE_ACTION_HORN,
  IR_REMOTE_ACTION_TURN_AROUND,   /* 原地旋转180° */
  IR_REMOTE_ACTION_CALL_ALLY     /* 呼唤友军：原地鸣笛闪灯8s */
} IrRemote_Action;

const char *IrRemote_KeyName(uint8_t command);
IrRemote_Action IrRemote_GetAction(uint8_t command);
const char *IrRemote_ActionName(IrRemote_Action action);

#ifdef __cplusplus
}
#endif

#endif /* __IR_REMOTE_MAP_H__ */