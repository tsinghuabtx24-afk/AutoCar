#include "vision_task.h"
#include "car.h"

#include <stdio.h>

static VisionTask_State task_state;
static uint32_t task_start_tick;
static uint32_t task_phase_tick;
static uint8_t task_phase;
static uint8_t task_speed;

static void VisionTask_Alert(uint8_t on)
{
  /* 入库提示：两侧 RGB 同时点亮（RGB_SetColor 的 r/g 分别对应右/左侧红色）。 */
  RGB_SetColor(on, on, 0U);
}

static void VisionTask_Buzzer(uint8_t on)
{
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin,
                    (on != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void VisionTask_Begin(VisionTask_State state)
{
  task_state = state;
  task_start_tick = HAL_GetTick();
  task_phase_tick = task_start_tick;
  task_phase = 0U;
  printf("[VTASK] start %s\r\n", VisionTask_StateName(state));
}

static void VisionTask_Finish(void)
{
  VisionTask_Alert(0U);
  VisionTask_Buzzer(0U);
  task_state = VISION_TASK_IDLE;
  printf("[VTASK] done, speed=%u\r\n", (unsigned)task_speed);
}

void VisionTask_Init(void)
{
  task_state = VISION_TASK_IDLE;
  task_start_tick = 0U;
  task_phase_tick = 0U;
  task_phase = 0U;
  task_speed = VISION_SPEED_NORMAL;
  VisionTask_Alert(0U);
  VisionTask_Buzzer(0U);
}

uint8_t VisionTask_GetSpeed(void) { return task_speed; }
VisionTask_State VisionTask_GetState(void) { return task_state; }

const char *VisionTask_StateName(VisionTask_State state)
{
  switch (state)
  {
    case VISION_TASK_TURN_LEFT: return "TURN_LEFT";
    case VISION_TASK_TURN_RIGHT: return "TURN_RIGHT";
    case VISION_TASK_HORN: return "HORN";
    case VISION_TASK_PARK_1: return "PARK_1";
    case VISION_TASK_PARK_2: return "PARK_2";
    case VISION_TASK_STOPPED: return "STOPPED";
    default: return "IDLE";
  }
}

/**
  * @brief  接收新的视觉事件
  * @note   限速/解除限速只改巡线速度，不创建任务，也不抢占底盘。
  *         其余事件创建任务；任务运行中不接收同类新事件，避免反复重启动作。
  */
static void VisionTask_AcceptEvent(void)
{
  Vision_Event event;

  if (VisionUart_ReadEvent(&event) == 0U)
  {
    return;
  }

  printf("[VISION] %s (id=%u)\r\n",
         VisionUart_TargetName(event.target), event.id);

  switch (event.target)
  {
    case VISION_TARGET_SPEED_NORMAL:
      /* 限速：在正常速度基础上降低固定占空比，并保护下限。 */
      if (VISION_SPEED_NORMAL > VISION_SPEED_LIMIT_DROP)
      {
        task_speed = (uint8_t)(VISION_SPEED_NORMAL - VISION_SPEED_LIMIT_DROP);
      }
      else
      {
        task_speed = 0U;
      }
      printf("[VTASK] speed limited to %u\r\n", (unsigned)task_speed);
      break;

    case VISION_TARGET_SPEED_RELEASE:
      /* 解除限速：恢复正常巡线速度。 */
      task_speed = VISION_SPEED_NORMAL;
      printf("[VTASK] speed restored to %u\r\n", (unsigned)task_speed);
      break;

    case VISION_TARGET_TURN_LEFT:
      if ((task_state == VISION_TASK_IDLE) ||
          (task_state == VISION_TASK_STOPPED))
      {
        VisionTask_Begin(VISION_TASK_TURN_LEFT);
      }
      break;

    case VISION_TARGET_TURN_RIGHT:
      if ((task_state == VISION_TASK_IDLE) ||
          (task_state == VISION_TASK_STOPPED))
      {
        VisionTask_Begin(VISION_TASK_TURN_RIGHT);
      }
      break;

    case VISION_TARGET_HORN:
      if ((task_state == VISION_TASK_IDLE) ||
          (task_state == VISION_TASK_STOPPED))
      {
        VisionTask_Begin(VISION_TASK_HORN);
      }
      break;

    case VISION_TARGET_PARK_1:
      if ((task_state == VISION_TASK_IDLE) ||
          (task_state == VISION_TASK_STOPPED))
      {
        VisionTask_Begin(VISION_TASK_PARK_1);
      }
      break;

    case VISION_TARGET_PARK_2:
      if ((task_state == VISION_TASK_IDLE) ||
          (task_state == VISION_TASK_STOPPED))
      {
        VisionTask_Begin(VISION_TASK_PARK_2);
      }
      break;
    default:
      break;
  }
}

/**
  * @brief  鸣笛任务：响 0.5s、停 0.2s，共两声，期间停车
  */
static void VisionTask_RunHorn(void)
{
  uint32_t now = HAL_GetTick();
  uint32_t elapsed = (uint32_t)(now - task_phase_tick);
  uint8_t beeping = (uint8_t)((task_phase % 2U) == 0U);
  uint32_t limit = (beeping != 0U) ? VISION_HORN_ON_MS : VISION_HORN_GAP_MS;

  Car_Stop();
  VisionTask_Buzzer(beeping);

  if (elapsed < limit)
  {
    return;
  }

  task_phase++;
  task_phase_tick = now;

  /* 每声占用「响 + 停」两个阶段。 */
  if (task_phase >= (uint8_t)(VISION_HORN_COUNT * 2U))
  {
    VisionTask_Finish();
  }
}

/**
  * @brief  闪灯，返回 1 表示闪灯阶段仍在进行
  */
static uint8_t VisionTask_RunBlink(void)
{
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - task_start_tick) >= VISION_PARK_BLINK_TOTAL_MS)
  {
    VisionTask_Alert(0U);
    return 0U;
  }
  if ((uint32_t)(now - task_phase_tick) >= VISION_PARK_BLINK_MS)
  {
    task_phase_tick = now;
    task_phase ^= 1U;
  }
  VisionTask_Alert(task_phase);
  return 1U;
}

/**
  * @brief  1 号库：闪灯后停车并保持停止
  */
static void VisionTask_RunPark1(void)
{
  Car_Stop();
  if (VisionTask_RunBlink() != 0U)
  {
    return;
  }
  VisionTask_Buzzer(0U);
  task_state = VISION_TASK_STOPPED;
  printf("[VTASK] park 1 finished, stay stopped\r\n");
}

/**
  * @brief  2 号库：闪灯后满速前进 2s
  */
static void VisionTask_RunPark2(void)
{
  uint32_t now;

  if (task_phase < 2U)
  {
    Car_Stop();

    if (VisionTask_RunBlink() != 0U)
    {
      return;
    }

    /* 闪灯结束时重新取时间，避免把本次处理耗时算入前进阶段。 */
    now = HAL_GetTick();
    task_phase = 2U;
    task_phase_tick = now;
  }
  else
  {
    now = HAL_GetTick();
  }

  Car_ForwardRun(VISION_SPEED_FULL);

  if ((uint32_t)(now - task_phase_tick) >= VISION_PARK2_BOOST_MS)
  {
    VisionTask_Finish();
  }
}

uint8_t VisionTask_Handle(void)
{
  VisionTask_AcceptEvent();

  switch (task_state)
  {
    case VISION_TASK_TURN_LEFT:
      /* car 的角度旋转当前是阻塞式，动作完成后立即结束任务。 */
      Car_RotateLeftAngle(VISION_TURN_SPEED, VISION_TURN_ANGLE_DEG10);
      VisionTask_Finish();
      return 1U;

    case VISION_TASK_TURN_RIGHT:
      Car_RotateRightAngle(VISION_TURN_SPEED, VISION_TURN_ANGLE_DEG10);
      VisionTask_Finish();
      return 1U;

    case VISION_TASK_HORN:
      VisionTask_RunHorn();
      return 1U;

    case VISION_TASK_PARK_1:
      VisionTask_RunPark1();
      return 1U;

    case VISION_TASK_PARK_2:
      VisionTask_RunPark2();
      return 1U;

    case VISION_TASK_STOPPED:
      /* 1 号库结束后保持停止；新的运动类视觉事件可在
         VisionTask_AcceptEvent() 中重新唤醒任务。 */
      Car_Stop();
      return 1U;

    default:
      return 0U;
  }
}

