#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define ULTRASONIC_TIMEOUT_MS  35U

void Ultrasonic_Init(void);
HAL_StatusTypeDef Ultrasonic_ReadMm(uint16_t *distance_mm);
uint16_t Ultrasonic_GetLastMm(void);
uint8_t Ultrasonic_IsValid(void);

#ifdef __cplusplus
}
#endif

#endif /* __ULTRASONIC_H__ */
