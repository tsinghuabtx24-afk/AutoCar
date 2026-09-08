#include "ir_remote_map.h"

const char *IrRemote_KeyName(uint8_t command)
{
  switch (command)
  {
    case 0x00U: return "RED";
    case 0x01U: return "UP";
    case 0x02U: return "BLUE";
    case 0x04U: return "LEFT";
    case 0x06U: return "RIGHT";
    case 0x05U: return "HORN";
    case 0x08U: return "BACK";
    case 0x09U: return "DOWN";
    case 0x0AU: return "RIGHTBACK";
    case 0x0CU: return "+";
    case 0x0DU: return "0";
    case 0x0EU: return "-";
    case 0x10U: return "1";
    case 0x11U: return "2";
    case 0x12U: return "3";
    case 0x14U: return "4";
    case 0x15U: return "5";
    case 0x16U: return "6";
    case 0x18U: return "7";
    case 0x19U: return "8";    /* 按键"8" → 原地旋转180° */
    case 0x1AU: return "9";    /* 按键"9" → 呼唤友军：原地鸣笛闪灯8s */
    default: return "UNKNOWN";
  }
}

IrRemote_Action IrRemote_GetAction(uint8_t command)
{
  switch (command)
  {
    case 0x01U: return IR_REMOTE_ACTION_FORWARD;
    case 0x04U: return IR_REMOTE_ACTION_ROTATE_LEFT;
    case 0x06U: return IR_REMOTE_ACTION_ROTATE_RIGHT;
    case 0x05U: return IR_REMOTE_ACTION_HORN;
    case 0x09U: return IR_REMOTE_ACTION_BACKWARD;    /* 新增：后退 */
    case 0x19U: return IR_REMOTE_ACTION_TURN_AROUND; /* 新增：原地旋转180° */
    case 0x1AU: return IR_REMOTE_ACTION_CALL_ALLY;  /* 新增：呼唤友军：原地鸣笛闪灯8s */
    default: return IR_REMOTE_ACTION_NONE;
  }
}

const char *IrRemote_ActionName(IrRemote_Action action)
{
  switch (action)
  {
    case IR_REMOTE_ACTION_FORWARD: return "FORWARD";
    case IR_REMOTE_ACTION_ROTATE_LEFT: return "ROTATE_LEFT";
    case IR_REMOTE_ACTION_ROTATE_RIGHT: return "ROTATE_RIGHT";
    case IR_REMOTE_ACTION_HORN: return "HORN";
    case IR_REMOTE_ACTION_BACKWARD: return "BACKWARD";
    case IR_REMOTE_ACTION_TURN_AROUND: return "TURN_AROUND";
    case IR_REMOTE_ACTION_CALL_ALLY: return "CALL_ALLY";
    default: return "NONE";
  }
}