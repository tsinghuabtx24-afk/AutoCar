/**
  ******************************************************************************
  * @file    indicator.h
  * @brief   蜂鸣器与 RGB 指示层
  ******************************************************************************
  * @note    蜂鸣器和 RGB 原先有四个写入方（ir_avoid / ultrasonic_avoid /
  *          vision_task / 按键中断），互相覆盖且无优先级：入库闪灯期间红外
  *          判无障碍就会把灯灭掉。
  *
  *          改成"申请 + 裁决"：模块只申请，不直接写 GPIO。Indicator_Step()
  *          每轮取最高优先级的有效申请落到硬件。闪烁相位由本模块统一维护，
  *          申请方只给周期。
  *
  *          顺带修掉 RGB_SetColor() 的坑：那个函数的 r 参数实际控制右侧红灯、
  *          g 控制左侧，交叉映射极易用错。本层接口直接用 left / right 命名。
  ******************************************************************************
  */
#ifndef __INDICATOR_H__
#define __INDICATOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 优先级。数值越大越优先，同一时刻只有最高优先级的申请生效。 */
typedef enum
{
  INDICATOR_PRIO_NONE = 0,
  INDICATOR_PRIO_TASK,     /* 入库闪灯、鸣笛 */
  INDICATOR_PRIO_MANUAL,   /* 遥控手动接管提示 */
  INDICATOR_PRIO_AVOID,    /* 避障告警 */
  INDICATOR_PRIO_FAULT,    /* 急停 / 故障 */
  INDICATOR_PRIO_COUNT
} Indicator_Prio;

/* RGB 颜色。当前硬件每侧三个引脚，按位组合。 */
typedef enum
{
  INDICATOR_OFF    = 0,
  INDICATOR_RED    = 1,
  INDICATOR_GREEN  = 2,
  INDICATOR_BLUE   = 4,
  INDICATOR_YELLOW = 3,   /* R|G */
  INDICATOR_CYAN   = 6,   /* G|B */
  INDICATOR_WHITE  = 7
} Indicator_Color;

/* 常闪周期。传 0 表示常亮不闪。 */
#define INDICATOR_BLINK_FAST_MS   150U
#define INDICATOR_BLINK_SLOW_MS   400U

void Indicator_Init(void);

/*
 * 提交一个指示申请。同一优先级重复申请会覆盖上一次。
 * buzzer:   1=鸣，0=静
 * left/right: 两侧 RGB 颜色
 * blink_ms: 闪烁半周期，0=常亮
 */
void Indicator_Request(Indicator_Prio prio, uint8_t buzzer,
                       Indicator_Color left, Indicator_Color right,
                       uint16_t blink_ms);

/* 撤销某优先级的申请。 */
void Indicator_Release(Indicator_Prio prio);

/* 撤销全部申请并关闭硬件。 */
void Indicator_ReleaseAll(void);

/* 每轮主循环调用一次：裁决并写 GPIO。 */
void Indicator_Step(void);

#ifdef __cplusplus
}
#endif

#endif /* __INDICATOR_H__ */
