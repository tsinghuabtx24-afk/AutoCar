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
  INDICATOR_PRIO_VISION,   /* 识别到视觉标志的绿灯提示，1s 后自动撤销 */
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

/* 点鸣模式的一声时长与间隔。 */
#define INDICATOR_BEEP_ON_MS      120U
#define INDICATOR_BEEP_GAP_MS     100U

/* ==== 视觉识别提示 ==========================================================
   识别到**任何**视觉标志就亮这个颜色这么久，到期自动撤销，不鸣笛。

   优先级取最低（INDICATOR_PRIO_VISION）：入库闪灯、避障告警、急停都比它重要，
   同时亮的时候让那些盖住它。所以这个提示是"没别的事时才看得到"。

   ⚠️ 左侧 RGB 的 R/G 引脚在 main.h 里命名与实物相反，indicator.c 的
      Indicator_WriteLeft() 已按引脚名做了补偿。若实车看到左右颜色不一致，
      改那个函数，不要改这里的颜色宏。
   ============================================================================ */
#define INDICATOR_VISION_COLOR    INDICATOR_GREEN
#define INDICATOR_VISION_HOLD_MS  1000U

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

/*
 * 同上，但 hold_ms 毫秒后**自动撤销**，调用方不必自己记时间去 Release。
 * hold_ms 为 0 时等价于 Indicator_Request()（一直有效直到显式 Release）。
 *
 * 与 Indicator_Request() 的另一个区别：重复提交相同参数会**刷新计时**而不是被
 * 当作"续订"忽略。这正是"每次识别到就重新亮 1s"想要的行为；点鸣计数不受影响，
 * 所以不会把蜂鸣器重置成第一声。
 */
void Indicator_RequestTimed(Indicator_Prio prio, uint8_t buzzer,
                            Indicator_Color left, Indicator_Color right,
                            uint16_t blink_ms, uint16_t hold_ms);

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
