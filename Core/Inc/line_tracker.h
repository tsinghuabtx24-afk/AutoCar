#ifndef __LINE_TRACKER_H__
#define __LINE_TRACKER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 传感器从左到右定义为 S1、S2、S3、S4。
   当前实物左侧两个传感器顺序与端口标号相反，因此 S1/S2 交换映射。 */
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

/* 运动参数，均可直接修改。速度为现有 Car 模块的 0~100 刻度。 */
#define LINE_FORWARD_SPEED       70U
#define LINE_CORRECT_ANGLE_DEG10 50U   /* 5.0 度，单位 0.1 度 */
#define LINE_SAMPLE_PERIOD_MS    10U

/* 未定义状态的安全策略：停车并输出一次状态。 */
#define LINE_STOP_ON_UNKNOWN     1U

void LineTracker_Init(void);
uint8_t LineTracker_ReadPattern(void);
void LineTracker_Step(void);
void LineTracker_Run(void);

#ifdef __cplusplus
}
#endif

#endif /* __LINE_TRACKER_H__ */
