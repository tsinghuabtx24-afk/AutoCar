/**
  ******************************************************************************
  * @file    ir_avoid.h
  * @brief   双路红外避障传感器
  ******************************************************************************
  * @note    **本模块只是传感器。** 它采样、判阈值、判迟滞、产生障碍事件，
  *          不驱动底盘、不控制蜂鸣和 RGB——避障动作在 avoid_task。
  *
  *          等发射管稳定不用 HAL_Delay，而是拆成相位状态机由 IrAvoid_Sense()
  *          每轮推进一相，全程非阻塞。
  ******************************************************************************
  */
#ifndef __IR_AVOID_H__
#define __IR_AVOID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum
{
  IR_AVOID_CLEAR = 0,
  IR_AVOID_LEFT,
  IR_AVOID_RIGHT,
  IR_AVOID_BOTH
} IrAvoid_State;

/* 发射使能引脚：左 PE5，右 PE6。 */
#define IR_AVOID_LEFT_ENABLE_PORT     Left_Switch_Iravoid_GPIO_Port
#define IR_AVOID_LEFT_ENABLE_PIN      Left_Switch_Iravoid_Pin
#define IR_AVOID_RIGHT_ENABLE_PORT    Right_Switch_Iravoid_GPIO_Port
#define IR_AVOID_RIGHT_ENABLE_PIN     Right_Switch_Iravoid_Pin
#define IR_AVOID_ENABLE_LEVEL         GPIO_PIN_RESET
#define IR_AVOID_DISABLE_LEVEL        GPIO_PIN_SET

/* 接收 ADC：PF9=ADC3_IN7（左），PF10=ADC3_IN8（右）。 */
#define IR_AVOID_LEFT_ADC_CHANNEL     ADC_CHANNEL_7
#define IR_AVOID_RIGHT_ADC_CHANNEL    ADC_CHANNEL_8

/* 0：ADC 越小表示障碍越近；1：ADC 越大表示障碍越近。 */
#define IR_AVOID_OBSTACLE_WHEN_HIGH   0U

/* 当前实测 ADC 约 60~2000，先以 1500 作为左右两侧判断阈值。 */
#define IR_AVOID_LEFT_THRESHOLD       1500U
#define IR_AVOID_RIGHT_THRESHOLD      1500U
#define IR_AVOID_HYSTERESIS           100U

#define IR_AVOID_SAMPLE_COUNT         8U
#define IR_AVOID_EMITTER_SETTLE_MS    1U
#define IR_AVOID_UPDATE_PERIOD_MS     20U
#define IR_AVOID_REPORT_PERIOD_MS     500U

void IrAvoid_Init(void);

/* 每轮主循环调用，推进采样状态机一步（非阻塞）。状态变化时投 EVENT_OBSTACLE_*。 */
void IrAvoid_Sense(void);

/* 丢弃当前采样进度重新开始。避障动作结束后调用，确保拿到的是动作后的新数据。 */
void IrAvoid_Restart(void);

IrAvoid_State IrAvoid_GetState(void);
uint8_t  IrAvoid_IsLeftBlocked(void);
uint8_t  IrAvoid_IsRightBlocked(void);
uint8_t  IrAvoid_IsAnyBlocked(void);
uint16_t IrAvoid_GetLeftRaw(void);
uint16_t IrAvoid_GetRightRaw(void);

/* 一轮完整采样是否已完成过至少一次（数据是否可信）。 */
uint8_t IrAvoid_HasSample(void);

const char *IrAvoid_StateName(IrAvoid_State state);

/* 阻塞式单次采样，仅供独立标定使用，不要在正式主循环调用。 */
void IrAvoid_UpdateBlocking(void);

#ifdef __cplusplus
}
#endif

#endif /* __IR_AVOID_H__ */
