#include "ir_remote_debug.h"
#include "ir_remote.h"
#include "ir_remote_map.h"
#include "oled.h"

#include <stdio.h>

static uint8_t debug_have_frame;
static IrRemote_Frame debug_frame;

void IrRemoteDebug_Init(void)
{
  debug_have_frame = 0U;
  Oled_Clear();
  Oled_SetCursor(0U, 0U);
  Oled_WriteString("REMOTE TEST");
  Oled_SetCursor(0U, 2U);
  Oled_WriteString("WAIT CODE");
  Oled_Update();
}

void IrRemoteDebug_Update(void)
{
  uint8_t received = IrRemote_Read(&debug_frame);

  if (received != 0U)
  {
    debug_have_frame = 1U;
    printf("[REMOTE] address=0x%02X command=0x%02X inverse=0x%02X key=%s action=%s\r\n",
           debug_frame.address,
           debug_frame.command,
           debug_frame.inverse_command,
           IrRemote_KeyName(debug_frame.command),
           IrRemote_ActionName(IrRemote_GetAction(debug_frame.command)));
  }

  if (IrRemote_IsRepeat() != 0U)
  {
    printf("[REMOTE] repeat\r\n");
  }

  if (received != 0U)
  {
    char line[22];
    Oled_Clear();
    Oled_SetCursor(0U, 0U);
    Oled_WriteString(IrRemote_KeyName(debug_frame.command));
    Oled_SetCursor(0U, 2U);
    snprintf(line, sizeof(line), "A:%02X", debug_frame.address);
    Oled_WriteString(line);
    Oled_SetCursor(0U, 4U);
    snprintf(line, sizeof(line), "C:%02X", debug_frame.command);
    Oled_WriteString(line);
    Oled_SetCursor(0U, 6U);
    snprintf(line, sizeof(line), "I:%02X", debug_frame.inverse_command);
    Oled_WriteString(line);
    Oled_Update();
  }
  else if (debug_have_frame == 0U)
  {
    /* 无新帧时保持等待提示，不反复刷新 I2C。 */
  }
}
