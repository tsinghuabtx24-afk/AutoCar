/**
  ******************************************************************************
  * @file    speed_ctrl.h
  * @brief   四路电机转速闭环（PI + 前馈）
  ******************************************************************************
  * @note    解决的问题：原先上层直接下发占空比，转速取决于电池电压、地面
  *          摩擦和载重。电量从满到半，同一个占空比的实际车速能差 20% 以上，
  *          所有靠时间标定的量（三叶草时间、避障横移）随之漂移。
  *
  *          本模块把控制量从"占空比"改成"目标转速 rpm"：
  *              上层给 rpm  →  PI 比较编码器实测 rpm  →  算出占空比
  *          电压掉了 PI 自动补占空比，车速不变。CAR_SPEED_BASE 那套死区
  *          开环补偿在闭环下不再需要（前馈里已包含）。
  *
  *          坐标系约定（容易搞错，务必注意）：
  *            - 目标与实测 rpm 都在**车体系**，前进为正。编码器的
  *              ENC_POLARITY_Mx 已把四轮统一成前进为正，所以可以直接比。
  *            - 输出占空比要转到**电机系**才能给 Motor_SetSpeedRaw()。
  *              极性表由 Car_Init() 通过 SpeedCtrl_Init() 传进来，
  *              car.c 的 CAR_*_POLARITY 仍是唯一真源，本模块不自己定义。
  *
  *          调用位置：SpeedCtrl_Step() 放在 SysTick 里紧跟 Encoder_Update()，
  *          见 stm32f1xx_it.c。放中断而不是主循环，是为了保证固定周期，
  *          且 HAL_Delay 期间（阻塞标定接口）闭环照样工作。
  ******************************************************************************
  */
#ifndef __SPEED_CTRL_H__
#define __SPEED_CTRL_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "encoder.h"        /* ENC_MAX_RPM：斜坡步长和前馈量程都要用 */

/* ==== 总开关 ================================================================
   置 0 则完全退回开环占空比模式（Car_Drive 直写 PWM），本模块不参与。
   换底盘、编码器故障、或要对比闭环前后差异时用得上。
   ============================================================================ */
#define SPEEDCTRL_ENABLE        1U

/* PI 执行周期 ms。必须 >= ENC_SPEED_PERIOD_MS：转速值本身每 20ms 才更新
   一次，跑得比它快只是在同一个测量值上重复积分，白增加超调。 */
#define SPEEDCTRL_PERIOD_MS     20U

/* ==== PI 参数（分数形式，F103 无硬件 FPU）===================================
   输出量纲是占空比 0~100，误差量纲是 rpm。
     KP: 误差 100rpm 时贡献 100*KP_NUM/KP_DEN = 25 的占空比
     KI: 每周期把误差累加进积分，积分每 100rpm·周期 贡献 100*KI_NUM/KI_DEN
   调参顺序：先只留 KP（KI_NUM=0）调到响应快且不振荡，再加 KI 消除稳态误差。
   ============================================================================ */
#define SPEEDCTRL_KP_NUM        1L
#define SPEEDCTRL_KP_DEN        4L
#define SPEEDCTRL_KI_NUM        1L
#define SPEEDCTRL_KI_DEN        32L

/* 积分限幅（rpm·周期）。防止起步或堵转时积分饱和，解除后车猛冲。
   取值让 KI 项最大贡献约 ±40 占空比：40*KI_DEN/KI_NUM = 1280。 */
#define SPEEDCTRL_I_LIMIT       1280L

/* 输出占空比下限。PI 算出的占空比低于此值时电机堵转不动，反而让积分继续
   累积。给一个最小输出让轮子能动起来，PI 才有实测值可用。
   注意这不是 MOTOR_SPEED_MIN 那种"死区补偿" —— 闭环下前馈已经跨过死区，
   这里只兜底极小目标转速的情形。 */
#define SPEEDCTRL_OUT_MIN       (MOTOR_SPEED_MIN + 2U)

/* 目标转速死区 rpm。目标绝对值小于此值一律当作停止，避免在 0 附近
   来回抖动、积分乱走。 */
#define SPEEDCTRL_TARGET_DEAD   3

/* 起步斜坡时间 ms：目标转速从 0 爬到 ENC_MAX_RPM 所需的时间。0 = 关闭斜坡。
 *
 * 为什么需要：四个电机同时从静止全压启动，浪涌电流把电池电压拉垮，加上左右
 * 静摩擦不相等，谁先挣脱静摩擦谁先转 —— 表现为左右轮起步差几百毫秒。斜坡把
 * 起步电流摊开，四轮就能一起动起来。顺带压住起步打滑（轮子空转的计数会被
 * 编码器当成里程，让判停提前满足，实际距离偏短）。
 *
 * 只限制"增大"：减速和换向立即生效。否则制动要等斜坡降完才生效，循迹的急转
 * 也会被拖慢 —— 那是安全和响应问题，不能为了平顺牺牲。
 *
 * 怎么标：从 150 起。左右仍不同时就加大（200/250）；循迹入弯变迟钝、
 * 原地旋转起步发飘就减小（120/100）。当前 ENC_MAX_RPM=150 下每周期步长
 * 20rpm，爬到循迹基速 60rpm 约 60ms，爬到 90rpm 约 90ms。
 *
 * 注意这个时间是"0 → ENC_MAX_RPM 满量程"的时间，所以重新标定 ENC_MAX_RPM 后
 * 步长会跟着变（量程翻倍则步长翻倍），但满量程耗时仍是 RAMP_MS。 */
#define SPEEDCTRL_RAMP_MS       150U

/* 每个控制周期允许的目标转速增量 rpm。由 RAMP_MS 换算，不要直接改这个。
   RAMP_MS 为 0 时取 0，表示不限速率（由 StepMotor 里的判断短路掉）。 */
#define SPEEDCTRL_RAMP_STEP     ((SPEEDCTRL_RAMP_MS == 0U) ? 0 \
                                 : ((int32_t)ENC_MAX_RPM * (int32_t)SPEEDCTRL_PERIOD_MS \
                                    / (int32_t)SPEEDCTRL_RAMP_MS))

/* Exported functions prototypes ---------------------------------------------*/

/**
  * @brief  初始化闭环
  * @param  out_polarity: 长度 MOTOR_NUM 的极性表，元素为 +1/-1，
  *                       表示"车体前进"对应该电机 Motor_SetSpeedRaw 的符号。
  *                       内部只保存指针，调用方须传静态数组。
  */
void SpeedCtrl_Init(const int8_t *out_polarity);

/* 设定左右两侧目标转速，车体系、前进为正。任一侧非零即视为闭环接管。 */
void SpeedCtrl_SetTargetRpm(int16_t left_rpm, int16_t right_rpm);

/* 交还控制权：清零目标与积分，并让四路电机滑行。
   调用方随后若要制动，自行调 Motor_BrakeAll()（顺序：先 Disable 再 Brake，
   否则 SysTick 可能在 Brake 之后又写一次 PWM）。 */
void SpeedCtrl_Disable(void);

/* 推进一步。内部按 SPEEDCTRL_PERIOD_MS 限流，可以每 1ms 调。 */
void SpeedCtrl_Step(void);

uint8_t SpeedCtrl_IsEnabled(void);
int16_t SpeedCtrl_GetTargetRpm(Motor_ID motor);
int16_t SpeedCtrl_GetDuty(Motor_ID motor);

#ifdef __cplusplus
}
#endif

#endif /* __SPEED_CTRL_H__ */
