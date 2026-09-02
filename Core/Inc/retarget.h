/**
  ******************************************************************************
  * @file    retarget.h
  * @brief   printf 重定向到 USART1
  ******************************************************************************
  * @note    USART1 = PA9(TX)/PA10(RX)，115200 8N1。这对引脚同时也是 F103
  *          ISP bootloader 的下载口，所以烧录线插着就能看输出，不用额外接线。
  *
  *          用法：MX_USART1_UART_Init() 之后调一次 Retarget_Init()，
  *          之后 printf() 直接可用。
  *
  * @warning 不要用 %f —— newlib-nano 默认不链接浮点格式化代码，会打出乱码。
  *          需要小数就自己拆成整数部分和小数部分打，例如
  *              printf("%ld.%03ld\n", mm / 1000, mm %% 1000);
  ******************************************************************************
  */
#ifndef __RETARGET_H__
#define __RETARGET_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported functions prototypes ---------------------------------------------*/
void Retarget_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __RETARGET_H__ */
