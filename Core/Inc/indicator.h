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
  *          RGB_SetColor() 函数的 r 参数实际控制右侧红灯、g 控制左侧，交叉映射极易用错。本层接口直接用 left / right 命名。
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
  INDICATOR_PRIO_SIGNAL,   /* 视觉信号灯效：隧道白灯、友军短闪。短时且不占底盘，
                              压在 TASK 之上，否则会被同时在跑的任务灯覆盖 */
  INDICATOR_PRIO_MANUAL,   /* 遥控手动接管提示 */
  INDICATOR_PRIO_CALL_ALLY,    /* 呼唤友军（遥控"9"）：两短闪循环 + 鸣笛。
                              压在 MANUAL 之上，否则手动接管的蓝灯常亮会把
                              呼唤闪灯整个盖掉，看不出车在传讯 */
  INDICATOR_PRIO_AVOID,    /* 避障呼唤 */
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

/* 点鸣模式的一声时长与间隔。 */
#define INDICATOR_BEEP_ON_MS      120U
#define INDICATOR_BEEP_GAP_MS     100U

/* buzzer 参数的取值。持续鸣叫用 INDICATOR_BUZZER_ON，
   叫固定几声后自动静音用 INDICATOR_BEEPS(n)。 */
#define INDICATOR_BUZZER_OFF      0U
#define INDICATOR_BUZZER_ON       0xFFU
#define INDICATOR_BEEPS(n)        ((uint8_t)(n))

void Indicator_Init(void);

/*
 * 提交一个指示申请。同一优先级重复申请会覆盖上一次。
 *
 * buzzer:   INDICATOR_BUZZER_OFF     静音
 *           INDICATOR_BUZZER_ON      持续鸣叫直到 Release
 *           INDICATOR_BEEPS(n)       叫 n 声后自动静音，灯继续按 blink_ms 闪
 * left/right: 两侧 RGB 颜色
 * blink_ms: 闪烁半周期，0=常亮
 *
 * 注意：重复提交同一优先级的**相同**参数不会重启点鸣计数，否则每轮主循环
 * 都重新申请的话蜂鸣器会一直响。参数变化才重新起算。
 */
void Indicator_Request(Indicator_Prio prio, uint8_t buzzer,
                       Indicator_Color left, Indicator_Color right,
                       uint16_t blink_ms);

/* 撤销某优先级的申请。 */
void Indicator_Release(Indicator_Prio prio);

/* 撤销全部申请并关闭硬件。 */
void Indicator_ReleaseAll(void);

/*
 * 暂时交出蜂鸣器引脚。buzzer_tone 播放旋律时需要按音高翻转引脚，
 * 本层每轮写 GPIO 会把方波压平，所以播放期间置 1，结束置 0。
 * 只影响蜂鸣器，灯光仲裁照常。
 */
void Indicator_SuspendBuzzer(uint8_t suspend);

/* 每轮主循环调用一次：裁决并写 GPIO。 */
void Indicator_Step(void);

#ifdef __cplusplus
}
#endif

#endif /* __INDICATOR_H__ */
