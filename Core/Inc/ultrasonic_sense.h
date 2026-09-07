/**
  ******************************************************************************
  * @file    ultrasonic_sense.h
  * @brief   超声波前方障碍传感器
  ******************************************************************************
  * @note    只测距、判阈值、发事件，不驱动底盘。避障动作在 avoid_task。
  *
  *          超声波只能报正前方，与红外的左/右语义不重叠。两者同时报障时
  *          **前方优先**：有前方障碍就按前方处理（后退再选向），否则按红外
  *          的左右状态处理。
  *
  *          测距已改为非阻塞（阶段 6）：Ultrasonic_Start() 发 TRIG 后立即返回，
  *          ECHO 双边沿在中断里打时间戳，本模块每轮 Ultrasonic_Step() 推进。
  *          空旷无回波时不再卡 35ms，循迹的 10ms 控制周期不受影响。
  ******************************************************************************
  */
#ifndef __ULTRASONIC_SENSE_H__
#define __ULTRASONIC_SENSE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 前方障碍判定距离。 */
#define ULTRASONIC_SENSE_THRESHOLD_MM   300U

/* 迟滞：解除判定要比触发判定远这么多，避免边界抖动。 */
#define ULTRASONIC_SENSE_HYSTERESIS_MM  50U

/* 测距周期。测距本身已不阻塞主循环，但仍要 ≥ ULTRASONIC_MIN_GAP_MS(60)，
   否则上一次的余波会被当成这一次的回波。 */
#define ULTRASONIC_SENSE_PERIOD_MS      100U

void UltrasonicSense_Init(void);

/* 推进一次测距判定，状态变化时投递 EVENT_OBSTACLE_FRONT / CLEAR。 */
void UltrasonicSense_Sense(void);

/* 丢弃当前判定，下一轮重新测。避障动作结束后调用。 */
void UltrasonicSense_Restart(void);

uint8_t  UltrasonicSense_IsBlocked(void);
uint16_t UltrasonicSense_GetLastMm(void);

#ifdef __cplusplus
}
#endif

#endif /* __ULTRASONIC_SENSE_H__ */
