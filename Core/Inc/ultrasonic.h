/**
  ******************************************************************************
  * @file    ultrasonic.h
  * @brief   HC-SR04 超声波测距
  ******************************************************************************
  * @note    测距改为**中断式非阻塞**：发 TRIG 后立即返回，ECHO 的上升沿和
  *          下降沿各触发一次 EXTI，在中断里用 DWT 打时间戳，主循环只查状态。
  *
  *          为什么不用定时器输入捕获（架构文档阶段 6 原计划）：
  *          **ECHO 接在 PF12，F103 没有任何定时器通道映射到 GPIOF**。输入捕获
  *          要求引脚本身是 TIMx_CHy，改用输入捕获必须先改硬件接线。EXTI 双边沿
  *          + DWT 时间戳达到同样的目的——消掉忙等——且不动硬件。
  *
  *          代价：中断响应有抖动（几百 ns 量级，主要来自 EXTI15_10 与 IR_IN
  *          共享向量时的排队）。1mm 距离对应 5.8us 回波时间，这点抖动不影响
  *          毫米级精度。
  *
  *          残留阻塞只有 TRIG 那 10us 触发脉冲（DWT 忙等），这是 HC-SR04 的
  *          时序要求，无法省掉，量级可忽略。
  ******************************************************************************
  */
#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 单次测距的总超时。ECHO 迟迟不来（空旷无回波）或回波异常长时到期作废。
   HC-SR04 量程 4m 对应约 23ms 回波，取 35ms 留足余量。 */
#define ULTRASONIC_TIMEOUT_MS  35U

/* 两次测距之间的最小间隔。HC-SR04 手册要求 ≥60ms，否则上一次的余波会被当成
   这一次的回波（鬼影）。调用方的采样周期若小于这个值，Start 会被拒绝。 */
#define ULTRASONIC_MIN_GAP_MS  60U

/* 量程上限。超过这个值一律视为无效回波，避免鬼影读出一个荒谬的近距离。 */
#define ULTRASONIC_MAX_MM      4000U

typedef enum
{
  ULTRASONIC_IDLE = 0,    /* 空闲，可以发起新测距 */
  ULTRASONIC_BUSY,        /* 已发 TRIG，等 ECHO */
  ULTRASONIC_DONE,        /* 本次测距成功，结果可读 */
  ULTRASONIC_TIMEOUT      /* 超时作废 */
} Ultrasonic_Status;

void Ultrasonic_Init(void);

/* ==== 非阻塞接口 ============================================================
   用法与 car 的动作状态机一致：Start 一次，之后每轮 Step 推进。

       if (Ultrasonic_GetStatus() == ULTRASONIC_IDLE) Ultrasonic_Start();
       switch (Ultrasonic_Step())
       {
         case ULTRASONIC_BUSY:    break;
         case ULTRASONIC_DONE:    use(Ultrasonic_GetLastMm()); break;
         case ULTRASONIC_TIMEOUT: no_echo(); break;
         default: break;
       }
   ============================================================================ */

/* 发起一次测距。成功返回 ULTRASONIC_BUSY；距上次测距不足
   ULTRASONIC_MIN_GAP_MS、或已有测距在跑时返回当前状态且不重发。 */
Ultrasonic_Status Ultrasonic_Start(void);

/* 每轮推进一次。返回终态（DONE/TIMEOUT）时内部已转 IDLE，终态只返回一次。 */
Ultrasonic_Status Ultrasonic_Step(void);

/* 查询状态，无副作用。只会看到 IDLE 或 BUSY——终态由 Step 直接消费掉。 */
Ultrasonic_Status Ultrasonic_GetStatus(void);

/* ECHO 双边沿中断入口，由 HAL_GPIO_EXTI_Callback 转发。 */
void Ultrasonic_EXTI_Callback(uint16_t GPIO_Pin);

uint16_t Ultrasonic_GetLastMm(void);
uint8_t Ultrasonic_IsValid(void);

/* ==== 阻塞接口（仅标定用）===================================================
   内部实现为"Start + 循环 Step 直到终态"，与非阻塞路径共享同一份换算逻辑，
   标定值不会两处不同步。正式主循环不要调用。
   ============================================================================ */
HAL_StatusTypeDef Ultrasonic_ReadMm(uint16_t *distance_mm);

#ifdef __cplusplus
}
#endif

#endif /* __ULTRASONIC_H__ */
