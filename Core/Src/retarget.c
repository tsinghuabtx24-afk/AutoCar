/**
  ******************************************************************************
  * @file    retarget.c
  * @brief   printf 重定向到 USART1 实现
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "retarget.h"
#include "usart.h"

#include <stdio.h>

/* Exported functions --------------------------------------------------------*/

/**
  * @brief  关掉 stdout 缓冲
  * @note   newlib 判断 stdout 不是终端设备时会用**全缓冲**，printf 的内容
  *         要攒够 1024 字节才真正调用 _write。标定时每 500ms 打一行，
  *         那就是几十秒才蹦出一大坨，看着像程序卡死。改成无缓冲后
  *         每个字符立刻发出去。
  *         代价是每字符一次 _write 调用，115200 下够快，不用在意。
  */
void Retarget_Init(void)
{
  setvbuf(stdout, NULL, _IONBF, 0);
}

/**
  * @brief  newlib 的底层字符输出钩子
  * @param  file: 文件描述符，1=stdout 2=stderr，这里不区分
  * @param  ptr:  待发送数据
  * @param  len:  字节数
  * @retval 已发送字节数，失败返回 -1
  *
  * @note   覆盖 syscalls.c 里的 weak 版本（那个直接丢弃所有输出）。
  *         链接器优先取强符号，所以不用改 syscalls.c。
  *
  * @note   阻塞发送。115200 下一字节约 87us，标定输出一行几十字节即
  *         几毫秒，对主循环没影响。但**不要在中断里 printf** ——
  *         比如 SysTick 每 1ms 触发一次，一行都发不完就会被下一次
  *         中断追上，串口输出会乱，还可能拖垮整个 tick 时基。
  */
int _write(int file, char *ptr, int len)
{
  (void)file;

  if (HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len,
                        HAL_MAX_DELAY) != HAL_OK)
  {
    return -1;
  }

  return len;
}
