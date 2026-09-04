#ifndef __ULTRASONIC_AVOID_H__
#define __ULTRASONIC_AVOID_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define ULTRASONIC_AVOID_SPEED          70U
#define ULTRASONIC_AVOID_REVERSE_MM     50U
#define ULTRASONIC_AVOID_ROTATE_SPEED   80U
#define ULTRASONIC_AVOID_ROTATE_DEG10   800U
#define ULTRASONIC_AVOID_THRESHOLD_MM   200U
#define ULTRASONIC_AVOID_SAMPLE_MS      100U

void UltrasonicAvoid_Init(void);
uint8_t UltrasonicAvoid_Handle(void);

#ifdef __cplusplus
}
#endif

#endif /* __ULTRASONIC_AVOID_H__ */
