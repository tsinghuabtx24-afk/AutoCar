/**
  ******************************************************************************
  * @file    line_tracker.h
  * @brief   四路红外黑线循迹
  ******************************************************************************
  * @note    控制律为**差速连续修正 + 大偏差原地转**的混合策略：
  *
  *            - 小偏差（1、2 档）：车不停，靠左右轮速差纠正航向，每周期算一次
  *              轮速就返回。
  *            - 大偏差（极左/极右）：前进中转过接近直角要内侧轮反转，本质已是
  *              原地旋转，纯差速压不住。此时切原地转，但**不是**转固定角度，
  *              而是边转边读传感器、一压回线就退出。
  *
  *          全程非阻塞是关键：阻塞转固定角度期间读不到传感器，反馈回路的死时间
  *          正是循迹振荡的主要来源。
  ******************************************************************************
  */
#ifndef __LINE_TRACKER_H__
#define __LINE_TRACKER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 传感器从左到右为 S1~S4。实物左侧两个的顺序与端口标号相反，故 S1/S2 交换映射。 */
#define LINE_SENSOR_S1_PIN       X2_Pin
#define LINE_SENSOR_S1_PORT      X2_GPIO_Port
#define LINE_SENSOR_S2_PIN       X1_Pin
#define LINE_SENSOR_S2_PORT      X1_GPIO_Port
#define LINE_SENSOR_S3_PIN       X3_Pin
#define LINE_SENSOR_S3_PORT      X3_GPIO_Port
#define LINE_SENSOR_S4_PIN       X4_Pin
#define LINE_SENSOR_S4_PORT      X4_GPIO_Port

/* 数字量：黑线=0，白色/反射面=1。 */
#define LINE_BLACK_LEVEL         GPIO_PIN_RESET
#define LINE_WHITE_LEVEL         GPIO_PIN_SET

/* ==== 控制周期 ==============================================================
   固定周期，不是"采样后 delay 这么久"：主循环每轮都调 LineTracker_Step()，
   未到周期就返回、维持上次轮速。周期必须由本模块自己把住而不能跟随主循环节拍
   （主循环每轮只几十微秒），因为差速增益的标定值只在固定周期下才有意义。

   10ms 够用：电机从改占空比到转速跟上是几十毫秒量级，比它更短不是瓶颈，只是
   重复算同一个结果。
   ============================================================================ */
#define LINE_CONTROL_PERIOD_MS   10U

/* ==== 速度与差速参数（必须实车标定）=========================================
   速度沿用 car 的占空比刻度 0~100。

   **重要约束**：占空比不超过 CAR_SPEED_BASE(50) 电机根本不转，所以基础速度要
   留够余量，让最大档差速减下来仍在死区之上：

       LINE_BASE_SPEED - 2 × LINE_DIFF_STEP > CAR_SPEED_BASE

   当前 60 和 4：最紧的 2 档是 60-8=52 > 50。余量不足时内侧轮会在 2 档修正时
   直接停转，那是急转不是修正。

   标定顺序（别一上来就提速）：
     1. 先调 LINE_DIFF_STEP 到直线上不画蛇；
     2. 再看缓弯是否切内道，不够就加大；
     3. 最后才逐步提 LINE_BASE_SPEED。
   ============================================================================ */
#define LINE_BASE_SPEED          60U

/* 一档修正的左右轮速度差。2 档用 ×2，3 档改走原地转。 */
#define LINE_DIFF_STEP           4U

/* 大偏差原地转，极左/极右图案时启用。不受 CAR_ANGLE_MIN_SPEED 限制：这里不按
   角度停而是边转边看传感器，与 car 的滑移补偿无关。 */
#define LINE_PIVOT_SPEED         75U

/* 原地转的超时保护。转这么久还没压回线就判丢线并制动。 */
#define LINE_PIVOT_TIMEOUT_MS    2500U

/* 连续这么多个控制周期读到全白判定丢线，制动并投 EVENT_FAULT。 */
#define LINE_LOST_COUNT          200U

/* 未定义图案（0x02/0x04/0x05/0x06/0x0A）是否立即停车。
   ⚠️ 全黑 0x00 **不走这条路径**，见下。 */
#define LINE_STOP_ON_UNKNOWN     0U

/* ==== 全黑图案（0x00）=======================================================
   全黑**不停车**，维持上一周期的轮速继续走，同时把图案告知 vision_nav 作为开启
   视觉的条件。因为全黑出现在路口和黑区，是有信息量的图案而不是传感器噪声；
   停在路口上视觉即使识别到了也没法继续走。
   ============================================================================ */
#define LINE_PATTERN_ALL_BLACK   0x00U

/* 全白（四路都离线）。丢线判定与下面的边缘观察窗都要认它。 */
#define LINE_PATTERN_ALL_WHITE   0x0FU

/* ==== 偏离图案（0001 / 1000 / 0011 / 1100）的两段判定 =======================
   这四个图案有歧义，同一个图案对应两种完全不同的路况：

     a) 车身偏了，线还压在外侧那一两路下面 → 该按档位差速修正；
     b) 直角/锐角弯道入口 → 该原地转，且下一瞬间四路就会全部离线。

   当场下注两边都会错：按 a 处理会在急弯冲出去，按 b 处理会在直线上被传感器
   抖动骗去原地转。所以**不当场判**——先按 a 走差速，同时开一个短观察窗盯住接
   下来几个周期：窗口内全白够多说明确实要离线，翻案改判 b；不够多说明只是普通
   偏离，窗口到期作废继续跟线。

   代价是急弯响应晚 LINE_EDGE_WHITE_COUNT 个周期，比一次误判原地转的姿态偏差
   便宜得多。1011/1101 **不开窗**：中间那路还压着线，没有歧义。
   ============================================================================ */

/* 观察窗长度，单位控制周期。 */
#define LINE_EDGE_WATCH_CYCLES   8U

/* 窗口内全白次数达到这么多就改判原地转。必须 ≤ LINE_EDGE_WATCH_CYCLES。
   调大 = 更保守（更晚认弯），调小 = 更激进（更容易被白缝骗）。 */
#define LINE_EDGE_WHITE_COUNT    1U

/* 要开观察窗的四个图案。EDGE_* 的值与 vision_nav 的 VNAV_PATTERN_*_EDGE 相同，
   但两者用途无关，各自定义以免耦合。 */
#define LINE_PATTERN_EDGE_R      0x01U   /* 0001：仅最右一路离线，1 档 */
#define LINE_PATTERN_EDGE_L      0x08U   /* 1000：仅最左一路离线，1 档 */
#define LINE_PATTERN_END_R       0x03U   /* 0011：轨迹右端，2 档 */
#define LINE_PATTERN_END_L       0x0CU   /* 1100：轨迹左端，2 档 */

/* 置 1 时每次图案变化打印一行。**实车跑循迹时置 0**：printf 走 USART1 阻塞发送，
   一行约 30 字节在 115200 下要 2.6ms，占掉 10ms 控制周期的四分之一，而图案变化
   很频繁。置 0 仍保留 pivot / lost / reacquire 等低频事件输出，不影响排查。 */
#define LINE_DEBUG_PATTERN       0U

/* ==== 运行时基础速度 ========================================================
   视觉的限速/恢复要生效，基础速度必须运行时可变，所以 LINE_BASE_SPEED 只是
   **默认值**，实际用的是内部变量。

   ⚠️ 上面的死区约束随之从编译期检查变成运行时钳位，差速档位按当前速度现算：
       eff_diff = min(LINE_DIFF_STEP, (base - CAR_SPEED_BASE - 1) / 2)
   不这样做的话限速后 2 档修正会把内侧轮压进死区、直接停转。
   ============================================================================ */

/* 设置基础速度。低于 CAR_SPEED_BASE+3 会被钳到该值——再低就没有差速空间了。 */
void LineTracker_SetBaseSpeed(uint8_t speed);
uint8_t LineTracker_GetBaseSpeed(void);

void LineTracker_Init(void);
uint8_t LineTracker_ReadPattern(void);

/* 推进一次循迹控制。非阻塞：未到控制周期直接返回，维持上次轮速。
   由调度器在 CONTROL_LINE_TRACK 模式下每轮调用。 */
void LineTracker_Step(void);

/* 被抢占后恢复循迹时调用，清掉原地转和丢线计数等内部状态。 */
void LineTracker_Reset(void);

/* 独立循迹调试入口，内部无限循环。正式主循环不要调用。 */
void LineTracker_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* __LINE_TRACKER_H__ */
