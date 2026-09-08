/**
  ******************************************************************************
  * @file    buzzer_tone.h
  * @brief   蜂鸣器音调 / 旋律播放
  ******************************************************************************
  * @note    蜂鸣器接在普通 GPIO（PG12）上，没有 PWM 通道，所以音调靠 TIM6
  *          更新中断翻转引脚生成方波。本模块**独占 TIM6**（CubeMX 未使用）。
  *
  *          播放期间由本模块直接写 Buzzer 引脚，因此会调
  *          Indicator_SuspendBuzzer(1) 让指示层暂时不碰蜂鸣器，结束时恢复。
  *          灯光仍由指示层管，互不干扰。
  *
  *          **硬件前提**：只有无源蜂鸣器才能出音调。若板上是有源蜂鸣器，
  *          方波只会让它按固定频率叫，旋律退化为节奏（仍可听出长短）。
  ******************************************************************************
  */
#ifndef __BUZZER_TONE_H__
#define __BUZZER_TONE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* 音符。freq_hz 为 0 表示休止符（静音但占时长）。 */
typedef struct
{
  uint16_t freq_hz;
  uint16_t ms;
} Buzzer_Note;

/* 常用音高（十二平均律，A4=440）。 */
#define BUZZER_NOTE_G4    392U
#define BUZZER_NOTE_C5    523U
#define BUZZER_NOTE_E5    659U
#define BUZZER_NOTE_G5    784U
#define BUZZER_NOTE_C6   1047U
#define BUZZER_NOTE_REST     0U

void BuzzerTone_Init(void);

/* 播放一段旋律。notes 必须是常驻内存（static / const），本模块只存指针。
   正在播放时再次调用会从头改播新旋律。 */
void BuzzerTone_Play(const Buzzer_Note *notes, uint8_t count);

/* 立即静音并交还蜂鸣器。 */
void BuzzerTone_Stop(void);

/* 每轮主循环调用一次：推进音符时序。 */
void BuzzerTone_Step(void);

uint8_t BuzzerTone_IsPlaying(void);

/* 友军信号用的旋律（军号式短促上行）。 */
extern const Buzzer_Note BUZZER_MELODY_FRIENDLY[];
extern const uint8_t     BUZZER_MELODY_FRIENDLY_LEN;

#ifdef __cplusplus
}
#endif

#endif /* __BUZZER_TONE_H__ */
