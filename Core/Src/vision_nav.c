/**
  ******************************************************************************
  * @file    vision_nav.c
  * @brief   视觉导航门控与两段式转向编排实现
  ******************************************************************************
  */

#include "vision_nav.h"
#include "car.h"

#include <stdio.h>

static VisionNav_State vnav_state;
static uint32_t vnav_phase_tick;      /* 当前状态的进入时刻 */
static uint32_t vnav_maneuver_tick;   /* 本次机动的起始时刻 */
static int8_t   vnav_turn_dir;        /* 视觉给出的方向：+1 左，-1 右 */
static uint32_t vnav_rejected;
/* 本控制周期的图案，由 OnPattern() 存入、TakeTurnRequest() 取用。
   两者都在同一个循迹周期内被调用，所以不存在过期问题。 */
static uint8_t  vnav_last_pattern;

const char *VisionNav_StateName(VisionNav_State state)
{
  switch (state)
  {
    case VNAV_OPEN:        return "OPEN";
    case VNAV_TURN_1_WAIT: return "TURN1_WAIT";
    case VNAV_TURN_1_RUN:  return "TURN1_RUN";
    case VNAV_LOCKOUT_1:   return "LOCKOUT1";
    case VNAV_TURN_2_WAIT: return "TURN2_WAIT";
    case VNAV_TURN_2_RUN:  return "TURN2_RUN";
    case VNAV_LOCKOUT_2:   return "LOCKOUT2";
    default:               return "ARMED";
  }
}

VisionNav_State VisionNav_GetState(void) { return vnav_state; }
uint32_t VisionNav_GetRejectedCount(void) { return vnav_rejected; }

static void VisionNav_Enter(VisionNav_State state)
{
  if (vnav_state == state)
  {
    return;
  }
  printf("[VNAV] %s -> %s\r\n",
         VisionNav_StateName(vnav_state), VisionNav_StateName(state));
  vnav_state      = state;
  vnav_phase_tick = HAL_GetTick();
}

void VisionNav_Init(void)
{
  vnav_state         = VNAV_ARMED;
  vnav_phase_tick    = HAL_GetTick();
  vnav_maneuver_tick = 0U;
  vnav_turn_dir      = 0;
  vnav_rejected      = 0U;
  vnav_last_pattern  = 0xFFU;
  printf("[VNAV] init: gate %s, turn cmd %u.%u deg\r\n",
#if VNAV_ENABLE
         "ENABLED",
#else
         "DISABLED (vision always open)",
#endif
         (unsigned)(VNAV_TURN_DEG10 / 10U), (unsigned)(VNAV_TURN_DEG10 % 10U));
}

void VisionNav_Reset(void)
{
  if (vnav_state != VNAV_ARMED)
  {
    printf("[VNAV] reset from %s\r\n", VisionNav_StateName(vnav_state));
  }
  vnav_state      = VNAV_ARMED;
  vnav_phase_tick = HAL_GetTick();
  vnav_turn_dir   = 0;
}

uint8_t VisionNav_IsVisionOpen(void)
{
#if VNAV_ENABLE
  return (uint8_t)((vnav_state == VNAV_OPEN) ? 1U : 0U);
#else
  return 1U;   /* 关闭门控时视觉常开 */
#endif
}

void VisionNav_Step(void)
{
#if VNAV_ENABLE
  uint32_t now = HAL_GetTick();

  switch (vnav_state)
  {
    case VNAV_LOCKOUT_1:
      /* 锁定期内正常循迹，只是不响应触发图案。 */
      if ((uint32_t)(now - vnav_phase_tick) >= VNAV_LOCKOUT_1_MS)
      {
        VisionNav_Enter(VNAV_TURN_2_WAIT);
      }
      break;

    case VNAV_LOCKOUT_2:
      if ((uint32_t)(now - vnav_phase_tick) >= VNAV_LOCKOUT_2_MS)
      {
        /* 机动结束，重新允许开视觉。 */
        vnav_turn_dir = 0;
        VisionNav_Enter(VNAV_ARMED);
      }
      break;

    case VNAV_TURN_1_WAIT:
    case VNAV_TURN_2_WAIT:
      /* 等触发图案。等太久说明岔路没找到或图案判错，放弃本次机动。
         不报故障：下次识别到转向信号可以重来。 */
      if ((uint32_t)(now - vnav_maneuver_tick) >= VNAV_MANEUVER_TIMEOUT_MS)
      {
        printf("[VNAV] maneuver timeout at %s, give up\r\n",
               VisionNav_StateName(vnav_state));
        vnav_turn_dir = 0;
        VisionNav_Enter(VNAV_ARMED);
      }
      break;

    default:
      break;
  }
#endif
}

uint8_t VisionNav_OnVisionEvent(Event_Type type)
{
#if VNAV_ENABLE
  /* ---- 门没开：一律丢弃 ---- */
  if (vnav_state != VNAV_OPEN)
  {
    vnav_rejected++;
    printf("[VNAV] reject %s: gate closed at %s\r\n",
           Event_TypeName(type), VisionNav_StateName(vnav_state));
    return 0U;
  }

  /* ---- 转向信号：不直接执行，转为内部两段式机动 ---- */
  if ((type == EVENT_VISION_TURN_LEFT) || (type == EVENT_VISION_TURN_RIGHT))
  {
    vnav_turn_dir      = (type == EVENT_VISION_TURN_LEFT) ? 1 : -1;
    vnav_maneuver_tick = HAL_GetTick();
    printf("[VNAV] turn signal %s -> maneuver armed\r\n",
           (vnav_turn_dir > 0) ? "LEFT" : "RIGHT");
    VisionNav_Enter(VNAV_TURN_1_WAIT);
    return 0U;   /* 不交给 vision_task 执行 */
  }

  /* ---- 其余有效信号（鸣笛/减速/恢复/入库）：直接执行，门关回 ARMED ---- */
  printf("[VNAV] accept %s, gate closes\r\n", Event_TypeName(type));
  VisionNav_Enter(VNAV_ARMED);
  return 1U;
#else
  (void)type;
  return 1U;   /* 关闭门控时全部照原样执行 */
#endif
}

void VisionNav_OnPattern(uint8_t pattern)
{
#if VNAV_ENABLE
  vnav_last_pattern = pattern;

  switch (vnav_state)
  {
    case VNAV_ARMED:
      /* 全黑是开启视觉的唯一条件（另一个条件 s==1 即本状态本身）。 */
      if (pattern == VNAV_PATTERN_ALL_BLACK)
      {
        printf("[VNAV] all-black seen, vision OPEN\r\n");
        VisionNav_Enter(VNAV_OPEN);
      }
      break;

    case VNAV_TURN_1_WAIT:
    case VNAV_TURN_2_WAIT:
      /* 触发图案到了，但发起旋转要等循迹来取——本模块不写电机。
         这里什么都不做，判断在 VisionNav_TakeTurnRequest() 里做，
         那样"看到图案"和"发起动作"发生在同一个控制周期，不会错开。 */
      break;

    default:
      break;
  }
#else
  (void)pattern;
#endif
}

/**
  * @brief  当前图案是否构成旋转触发
  */
static uint8_t VisionNav_IsTriggerPattern(uint8_t pattern)
{
  return (uint8_t)((pattern == VNAV_PATTERN_ALL_BLACK) ||
                   (pattern == VNAV_PATTERN_RIGHT_EDGE) ||
                   (pattern == VNAV_PATTERN_LEFT_EDGE));
}

uint8_t VisionNav_TakeTurnRequest(int32_t *deg10)
{
#if VNAV_ENABLE
  uint8_t pattern;

  if (deg10 == NULL)
  {
    return 0U;
  }

  /* 只有两个 WAIT 状态会发起旋转。RUN 中不重复发起，LOCKOUT 中不响应。 */
  if ((vnav_state != VNAV_TURN_1_WAIT) && (vnav_state != VNAV_TURN_2_WAIT))
  {
    return 0U;
  }

  /* 图案由 line_tracker 在同一周期内先调 OnPattern() 存进来。 */
  pattern = vnav_last_pattern;
  if (VisionNav_IsTriggerPattern(pattern) == 0U)
  {
    return 0U;
  }

  if (vnav_state == VNAV_TURN_1_WAIT)
  {
    /* 第一次：向视觉给出的方向转。 */
    *deg10 = (int32_t)vnav_turn_dir * (int32_t)VNAV_TURN_DEG10;
    VisionNav_Enter(VNAV_TURN_1_RUN);
  }
  else
  {
    /* 第二次：正向，在绕弯半圈后把车头转回原朝向。 */
    *deg10 = (int32_t)vnav_turn_dir * (int32_t)VNAV_TURN_DEG10;
    VisionNav_Enter(VNAV_TURN_2_RUN);
  }

  printf("[VNAV] turn request %ld deg10 (pattern 0x%02X)\r\n",
         (long)*deg10, (unsigned)pattern);
  return 1U;
#else
  (void)deg10;
  return 0U;
#endif
}

void VisionNav_OnTurnDone(int32_t enc_deg10)
{
#if VNAV_ENABLE
  printf("[VNAV] turn done at %s, cmd=%u enc=%ld\r\n",
         VisionNav_StateName(vnav_state),
         (unsigned)VNAV_TURN_DEG10, (long)enc_deg10);

  if (vnav_state == VNAV_TURN_1_RUN)
  {
    /* 锁定期从旋转完成时刻起算：这个值就是纯粹的"沿岔路走多久"。 */
    VisionNav_Enter(VNAV_LOCKOUT_1);
  }
  else if (vnav_state == VNAV_TURN_2_RUN)
  {
    VisionNav_Enter(VNAV_LOCKOUT_2);
  }
#else
  (void)enc_deg10;
#endif
}
