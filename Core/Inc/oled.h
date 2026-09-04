#ifndef __OLED_H__
#define __OLED_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define OLED_I2C_ADDRESS  (0x3CU << 1)
#define OLED_WIDTH        128U
#define OLED_HEIGHT       64U

/* 常见 0.96 寸模块实际为 SH1106；若芯片确认为 SSD1306 改为 0。 */
#define OLED_CONTROLLER_SH1106  1U
#if OLED_CONTROLLER_SH1106
#define OLED_COLUMN_OFFSET      2U
#else
#define OLED_COLUMN_OFFSET      0U
#endif

HAL_StatusTypeDef Oled_Init(void);
void Oled_Clear(void);
void Oled_SetCursor(uint8_t column, uint8_t page);
void Oled_WriteChar(char ch);
void Oled_WriteString(const char *text);
HAL_StatusTypeDef Oled_Update(void);
uint8_t Oled_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* __OLED_H__ */
