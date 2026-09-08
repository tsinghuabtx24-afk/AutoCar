#include "vision_task.h"
#include "buzzer_tone.h"
#include "car.h"
#include "indicator.h"
#include "line_tracker.h"

#include <stdio.h>

_Static_assert(VISION_SPEED_NORMAL > VISION_SPEED_LIMIT_DROP,
               "VISION_SPEED_LIMIT_DROP exceeds VISION_SPEED_NORMAL");
_Static_assert((VISION_SPEED_NORMAL - VISION_SPEED_LIMIT_DROP) >
               (CAR_SPEED_BASE + 2U),
               "limited speed falls into the motor dead zone");
_Static_assert(VISION_SPEED_NORMAL == LINE_BASE_SPEED,
               "VISION_SPEED_NORMAL must match LINE_BASE_SPEED");

static VisionTask_State task_state;
static uint32_t task_start_tick;
static uint32_t task_phase_tick;
static uint8_t task_phase;
static uint8_t task_speed;

static uint8_t task_alert_on;
static uint8_t task_buzzer_on;

/* 信号类动作的独立状态 */
static uint32_t signal_tunnel_until;
static uint32_t signal_slow_until;
static uint32_t signal_flash_until;
static uint32_t signal_narrow_until;   /* 狭窄街道黄闪的截止时刻 */

/* 友军旋律两遍之间的静音间隔起点。0 = 不在间隔中。 */
static uint32_t friendly_gap_tick;
static uint8_t  signal_light_shown;

/* 仓库任务专用状态 */
static uint8_t  warehouse_zero_count;
static uint8_t  warehouse_phase;
static uint32_t warehouse_sample_tick;   /* 上次全白采样时刻 */
static uint32_t warehouse_fwd_ms;        /* 直行段实测时长 x，回程照此走 */

/* 呼唤友军状态。call_ally_used 是一次性闩锁，只有 Init() 清零，
   所以 Cancel()/急停都不会让它恢复可用——符合"除 reset 不再收第二次"。 */
static uint8_t  call_ally_active;
static uint8_t  call_ally_used;
static uint32_t call_ally_start_tick;

static void VisionTask_Apply(void)
{
  if ((task_alert_on == 0U) && (task_buzzer_on == 0U))
  {
    Indicator_Release(INDICATOR_PRIO_TASK);
    return;
  }

  Indicator_Request(INDICATOR_PRIO_TASK,
                    (task_buzzer_on != 0U) ? INDICATOR_BUZZER_ON
                                           : INDICATOR_BUZZER_OFF,
                    (task_alert_on != 0U) ? INDICATOR_YELLOW : INDICATOR_OFF,
                    (task_alert_on != 0U) ? INDICATOR_YELLOW : INDICATOR_OFF,
                    0U);
}

static void VisionTask_Alert(uint8_t on)
{
  task_alert_on = on;
  VisionTask_Apply();
}

static void VisionTask_Buzzer(uint8_t on)
{
  task_buzzer_on = on;
  VisionTask_Apply();
}

/**
  * @brief  呼唤友军的灯笛输出。走 PRIO_CALL_ALLY，压得住手动模式的蓝灯。
  * @param  on 1=亮灯并鸣笛，0=熄灭并静音
  */
static void VisionTask_CallAllyApply(uint8_t on)
{
  Indicator_Request(INDICATOR_PRIO_CALL_ALLY,
                    (on != 0U) ? INDICATOR_BUZZER_ON : INDICATOR_BUZZER_OFF,
                    (on != 0U) ? INDICATOR_YELLOW : INDICATOR_OFF,
                    (on != 0U) ? INDICATOR_YELLOW : INDICATOR_OFF,
                    0U);
}

/**
  * @brief  呼唤图案：两次短闪 + 短暂熄灭，循环
  * @param  elapsed 距呼唤开始的毫秒数
  * @retval 1=此刻应亮，0=此刻应灭
  */
static uint8_t VisionTask_CallAllyPhaseOn(uint32_t elapsed)
{
  uint32_t t = elapsed % VISION_CALL_ALLY_CYCLE_MS;

  /* 第一闪 */
  if (t < VISION_CALL_ALLY_ON_MS)
  {
    return 1U;
  }
  t -= VISION_CALL_ALLY_ON_MS;

  /* 两闪之间的熄灭 */
  if (t < VISION_CALL_ALLY_GAP_MS)
  {
    return 0U;
  }
  t -= VISION_CALL_ALLY_GAP_MS;

  /* 第二闪 */
  if (t < VISION_CALL_ALLY_ON_MS)
  {
    return 1U;
  }

  /* 剩下的就是一组之后的短暂熄灭 */
  return 0U;
}

static void VisionTask_SignalLightApply(void)
{
  Indicator_Color color;
  uint16_t blink_ms;

  /* 三个信号灯效共用 PRIO_SIGNAL 这一个槽，同时成立时按下面的顺序取一个。
     友军闪白排在最前：它伴随停车，是当下最该看见的状态。 */
  if (signal_flash_until != 0U)
  {
    color    = INDICATOR_WHITE;
    blink_ms = 0U;
  }
  else if (signal_narrow_until != 0U)
  {
    color    = INDICATOR_YELLOW;
    blink_ms = INDICATOR_BLINK_FAST_MS;
  }
  else if (signal_tunnel_until != 0U)
  {
    color    = INDICATOR_WHITE;
    blink_ms = 0U;
  }
  else
  {
    if (signal_light_shown != 0U)
    {
      Indicator_Release(INDICATOR_PRIO_SIGNAL);
      signal_light_shown = 0U;
    }
    return;
  }

  Indicator_Request(INDICATOR_PRIO_SIGNAL, INDICATOR_BUZZER_OFF,
                    color, color, blink_ms);
  signal_light_shown = 1U;
}

static uint8_t VisionTask_SpeedForSignals(void)
{
  if (signal_slow_until != 0U)
  {
    return (uint8_t)(VISION_SPEED_NORMAL - VISION_SPEED_LIMIT_DROP);
  }
  return VISION_SPEED_NORMAL;
}

void VisionTask_Tick(void)
{
  uint32_t now = HAL_GetTick();

  if ((signal_tunnel_until != 0U) &&
      ((int32_t)(now - signal_tunnel_until) >= 0))
  {
    signal_tunnel_until = 0U;
    printf("[VTASK] tunnel light off\r\n");
  }

  if ((signal_flash_until != 0U) &&
      ((int32_t)(now - signal_flash_until) >= 0))
  {
    signal_flash_until = 0U;
  }

  if ((signal_narrow_until != 0U) &&
      ((int32_t)(now - signal_narrow_until) >= 0))
  {
    signal_narrow_until = 0U;
    printf("[VTASK] narrow street blink off\r\n");
  }

  if ((signal_slow_until != 0U) &&
      ((int32_t)(now - signal_slow_until) >= 0))
  {
    signal_slow_until = 0U;
    task_speed = VISION_SPEED_NORMAL;
    LineTracker_SetBaseSpeed(task_speed);
    printf("[VTASK] slow-ahead passed, speed restored to %u\r\n",
           (unsigned)task_speed);
  }

  VisionTask_SignalLightApply();
  BuzzerTone_Step();

  /* 呼唤友军：两短闪循环，8s 到点自动收尾。
     注意 call_ally_used 不在这里清——用过就是用过。 */
  if (call_ally_active != 0U)
  {
    uint32_t elapsed = (uint32_t)(now - call_ally_start_tick);

    if (elapsed >= VISION_CALL_ALLY_DURATION_MS)
    {
      call_ally_active = 0U;
      Indicator_Release(INDICATOR_PRIO_CALL_ALLY);
      if (task_state == VISION_TASK_CALL_ALLY)
      {
        task_state = VISION_TASK_IDLE;
        printf("[VTASK] horn alarm finished\r\n");
      }
    }
    else
    {
      VisionTask_CallAllyApply(VisionTask_CallAllyPhaseOn(elapsed));
    }
  }
}

static void VisionTask_Start(VisionTask_State state)
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
  task_phase = 0U;
  warehouse_zero_count = 0U;
  warehouse_phase = 0U;
  printf("[VTASK] done, speed=%u\r\n", (unsigned)task_speed);
}

void VisionTask_Init(void)
{
  task_state = VISION_TASK_IDLE;
  task_start_tick = 0U;
  task_phase_tick = 0U;
  task_phase = 0U;
  task_speed = VISION_SPEED_NORMAL;
  signal_tunnel_until = 0U;
  signal_slow_until = 0U;
  signal_flash_until = 0U;
  signal_narrow_until = 0U;
  friendly_gap_tick = 0U;
  signal_light_shown = 0U;
  warehouse_zero_count = 0U;
  warehouse_phase = 0U;
  warehouse_sample_tick = 0U;
  warehouse_fwd_ms = 0U;
  call_ally_active = 0U;
  call_ally_used = 0U;      /* 唯一的闩锁清零点：仅上电/复位 */
  VisionTask_Alert(0U);
  VisionTask_Buzzer(0U);
  Indicator_Release(INDICATOR_PRIO_SIGNAL);
  Indicator_Release(INDICATOR_PRIO_CALL_ALLY);
}

uint8_t VisionTask_GetSpeed(void) { return task_speed; }
VisionTask_State VisionTask_GetState(void) { return task_state; }

const char *VisionTask_StateName(VisionTask_State state)
{
  switch (state)
  {
    case VISION_TASK_STOPPED:           return "STOPPED";
    case VISION_TASK_FRIENDLY:          return "FRIENDLY";
    case VISION_TASK_NO_ENTRY:          return "NO_ENTRY";
    case VISION_TASK_WAREHOUSE:         return "WAREHOUSE";
    case VISION_TASK_CALL_ALLY:        return "CALL_ALLY";
    case VISION_TASK_TURN_AROUND:      return "TURN_AROUND";
    default:                            return "IDLE";
  }
}

uint8_t VisionTask_Begin(Event_Type type)
{
  uint8_t acceptable = (uint8_t)((task_state == VISION_TASK_IDLE) ||
                                 (task_state == VISION_TASK_STOPPED));

  switch (type)
  {
    /* ==== 信号类（不占底盘） ==== */
    case EVENT_VISION_TUNNEL:
      signal_tunnel_until = HAL_GetTick() + VISION_TUNNEL_LIGHT_MS;
      if (signal_tunnel_until == 0U) signal_tunnel_until = 1U;
      VisionTask_SignalLightApply();
      printf("[VTASK] tunnel: white light %u ms\r\n",
             (unsigned)VISION_TUNNEL_LIGHT_MS);
      return 0U;

    case EVENT_VISION_SLOW_AHEAD:
      signal_slow_until = HAL_GetTick() + VISION_SLOW_AHEAD_MS;
      if (signal_slow_until == 0U) signal_slow_until = 1U;
      task_speed = VisionTask_SpeedForSignals();
      printf("[VTASK] slow ahead: speed %u for %u ms\r\n",
             (unsigned)task_speed, (unsigned)VISION_SLOW_AHEAD_MS);
      return 0U;

    /* ==== 狭窄街道：黄灯闪烁 2s（不占底盘，仅指示） ==== */
    case EVENT_VISION_NARROW_STREET:
      signal_narrow_until = HAL_GetTick() + VISION_NARROW_BLINK_MS;
      if (signal_narrow_until == 0U) signal_narrow_until = 1U;
      VisionTask_SignalLightApply();
      printf("[VTASK] narrow street: yellow blink %u ms\r\n",
             (unsigned)VISION_NARROW_BLINK_MS);
      return 0U;

    /* ==== 友军信号：停车鸣笛，放完原速启动（占底盘） ==== */
    case EVENT_VISION_FRIENDLY:
      if (acceptable == 0U) return 0U;
      VisionTask_Start(VISION_TASK_FRIENDLY);
      friendly_gap_tick = 0U;   /* task_phase 由 Start() 清零，用作遍数计数 */
      /* 闪白灯覆盖整个停车段，配合旋律；由 RunFriendly 结束时清掉。 */
      signal_flash_until = HAL_GetTick() + VISION_FRIENDLY_TIMEOUT_MS;
      if (signal_flash_until == 0U) signal_flash_until = 1U;
      VisionTask_SignalLightApply();
      BuzzerTone_Play(BUZZER_MELODY_FRIENDLY, BUZZER_MELODY_FRIENDLY_LEN);
      printf("[VTASK] friendly: stop + melody\r\n");
      return 1U;

    /* ==== 倒塌房屋：走避障绕行 ====
       这里不做事，也不投 EVENT_OBSTACLE_FRONT。原来投假的传感器事件走不通：
       AvoidTask_Begin() 要靠传感器读数选绕行方向，读到"无障碍"就当场放弃，
       车一步都不动。改由 scheduler 直接调 AvoidTask_BeginDetour()。 */
    case EVENT_VISION_COLLAPSED_HOUSE:
      return 0U;

    /* ==== 禁止通行：倒车→右转→前进 ==== */
    case EVENT_VISION_NO_ENTRY:
      if (acceptable == 0U) return 0U;
      VisionTask_Start(VISION_TASK_NO_ENTRY);
      return 1U;

    /* ==== 仓库 ==== */
    case EVENT_VISION_WAREHOUSE:
      if (acceptable == 0U) return 0U;
      warehouse_zero_count = 0U;
      warehouse_phase = 0U;
      warehouse_fwd_ms = 0U;
      warehouse_sample_tick = HAL_GetTick();
      VisionTask_Start(VISION_TASK_WAREHOUSE);
      return 1U;

    default:
      return 0U;
  }
}

/**
  * @brief  友军信号：停住把旋律放四遍，然后原速启动
  *
  * @note   "原速"不需要在这里恢复：本任务全程不碰 task_speed / 基础速度，
  *         结束后调度器切回 CONTROL_LINE_TRACK，循迹沿用原来的基础速度。
  *         若此前正处于前方慢行的限速窗内，限速也照样保持——那是另一条
  *         独立的时间窗，不该被友军信号顺手清掉。
  *
  * @note   task_phase 用作已放遍数计数，friendly_gap_tick 记间隔起点。
  */
static void VisionTask_RunFriendly(void)
{
  uint32_t now     = HAL_GetTick();
  uint32_t elapsed = (uint32_t)(now - task_start_tick);

  Car_Stop();

  /* 兜底：蜂鸣器被 suspend 或旋律异常时别把车永远停在这。 */
  if (elapsed >= VISION_FRIENDLY_TIMEOUT_MS)
  {
    printf("[VTASK] friendly: melody timeout at loop %u/%u\r\n",
           (unsigned)task_phase, (unsigned)VISION_FRIENDLY_LOOPS);
    BuzzerTone_Stop();
    signal_flash_until = 0U;
    VisionTask_SignalLightApply();
    VisionTask_Finish();
    return;
  }

  /* 本遍还在响：等它放完。BuzzerTone_Step() 由 VisionTask_Tick() 推进。 */
  if (BuzzerTone_IsPlaying() != 0U)
  {
    friendly_gap_tick = 0U;
    return;
  }

  /* 本遍刚放完，开始记间隔。 */
  if (friendly_gap_tick == 0U)
  {
    friendly_gap_tick = now;
    return;
  }

  /* 间隔没走完，继续静音等着。 */
  if ((uint32_t)(now - friendly_gap_tick) < VISION_FRIENDLY_LOOP_GAP_MS)
  {
    return;
  }

  /* 起下一遍，或者四遍放满就收工。 */
  if (task_phase < (uint8_t)(VISION_FRIENDLY_LOOPS - 1U))
  {
    task_phase++;
    friendly_gap_tick = 0U;
    BuzzerTone_Play(BUZZER_MELODY_FRIENDLY, BUZZER_MELODY_FRIENDLY_LEN);
    printf("[VTASK] friendly: melody loop %u/%u\r\n",
           (unsigned)(task_phase + 1U), (unsigned)VISION_FRIENDLY_LOOPS);
    return;
  }

  /* 白灯窗口跟着一起收，别让它挂到 TIMEOUT 才灭。 */
  signal_flash_until = 0U;
  VisionTask_SignalLightApply();

  printf("[VTASK] friendly: %u loops done, resume at speed %u\r\n",
         (unsigned)VISION_FRIENDLY_LOOPS, (unsigned)task_speed);
  VisionTask_Finish();
}

static void VisionTask_RunNoEntry(void)
{
  uint32_t now = HAL_GetTick();

  switch (task_phase)
  {
    case 0U:  /* 倒车 */
      Car_Backward(VISION_SPEED_NORMAL, VISION_NO_ENTRY_BACK_MS);
      if ((uint32_t)(now - task_start_tick) >= VISION_NO_ENTRY_BACK_MS)
      {
        task_phase = 1U;
        task_phase_tick = now;
        Car_Stop();
        printf("[VTASK] no_entry: back done, turning right\r\n");
      }
      break;

    case 1U:  /* 右转90° */
      if (Car_StartRotateAngle(VISION_TURN_SPEED, -(int32_t)VISION_TURN_ANGLE_DEG10) 
          != CAR_ACTION_BUSY)
      {
        printf("[VTASK] no_entry: turn rejected\r\n");
        VisionTask_Finish();
        return;
      }
      task_phase = 2U;
      break;

    case 2U:  /* 等待右转完成 */
      switch (Car_ActionStep())
      {
        case CAR_ACTION_BUSY:
          break;
        case CAR_ACTION_DONE:
          task_phase = 3U;
          task_phase_tick = now;
          printf("[VTASK] no_entry: turn done, moving forward\r\n");
          break;
        default:
          printf("[VTASK] no_entry: turn failed\r\n");
          VisionTask_Finish();
          break;
      }
      break;

    case 3U:  /* 前进 */
      Car_ForwardRun(VISION_SPEED_NORMAL);
      if ((uint32_t)(now - task_phase_tick) >= VISION_NO_ENTRY_FWD_MS)
      {
        VisionTask_Finish();
        printf("[VTASK] no_entry: all done\r\n");
      }
      break;

    default:
      VisionTask_Finish();
      break;
  }
}

static void VisionTask_RunWarehouse(void)
{
  uint32_t now = HAL_GetTick();
  uint8_t pattern;

  switch (warehouse_phase)
  {
    case 0U:  /* 左转90° */
      if (Car_StartRotateAngle(VISION_TURN_SPEED, (int32_t)VISION_TURN_ANGLE_DEG10)
          != CAR_ACTION_BUSY)
      {
        printf("[VTASK] warehouse: turn left rejected\r\n");
        VisionTask_Finish();
        return;
      }
      warehouse_phase = 1U;
      warehouse_zero_count = 0U;
      break;

    case 1U:  /* 等待左转完成 */
      switch (Car_ActionStep())
      {
        case CAR_ACTION_BUSY:
          break;
        case CAR_ACTION_DONE:
          warehouse_phase = 2U;
          warehouse_zero_count = 0U;
          /* 直行段从这一刻起计时，这段时长就是 x。 */
          task_phase_tick = now;
          warehouse_sample_tick = now;
          printf("[VTASK] warehouse: left turn done, going straight\r\n");
          break;
        default:
          printf("[VTASK] warehouse: left turn failed\r\n");
          VisionTask_Finish();
          break;
      }
      break;

    case 2U:  /* 直行，直到连续 8 个采样周期读到 0000 */
      Car_ForwardRun(VISION_SPEED_NORMAL);

      /* 按固定节拍采样。Step() 每轮都进来，不限速的话 8 次只有零点几毫秒。 */
      if ((uint32_t)(now - warehouse_sample_tick) >= VISION_WAREHOUSE_SAMPLE_MS)
      {
        warehouse_sample_tick = now;
        pattern = LineTracker_ReadPattern();

        if (pattern == 0x00U)
        {
          warehouse_zero_count++;
        }
        else
        {
          warehouse_zero_count = 0U;
        }
      }

      /* 到位，或者走太久了兜底收手。两种情况都把已走时长记为 x。 */
      if ((warehouse_zero_count >= VISION_WAREHOUSE_ZERO_COUNT) ||
          ((uint32_t)(now - task_phase_tick) >= VISION_WAREHOUSE_FWD_MAX_MS))
      {
        warehouse_fwd_ms = (uint32_t)(now - task_phase_tick);
        if (warehouse_fwd_ms > VISION_WAREHOUSE_FWD_MAX_MS)
        {
          warehouse_fwd_ms = VISION_WAREHOUSE_FWD_MAX_MS;
        }

        if (warehouse_zero_count < VISION_WAREHOUSE_ZERO_COUNT)
        {
          printf("[VTASK] warehouse: straight timeout, x=%u ms\r\n",
                 (unsigned)warehouse_fwd_ms);
        }
        else
        {
          printf("[VTASK] warehouse: %u zero samples, x=%u ms, stopping\r\n",
                 (unsigned)VISION_WAREHOUSE_ZERO_COUNT,
                 (unsigned)warehouse_fwd_ms);
        }

        warehouse_phase = 3U;
        warehouse_zero_count = 0U;
        Car_Stop();
        task_phase_tick = now;
      }
      break;

    case 3U:  /* 停止3s */
      Car_Stop();
      if ((uint32_t)(now - task_phase_tick) >= VISION_WAREHOUSE_STOP_MS)
      {
        warehouse_phase = 4U;
        printf("[VTASK] warehouse: stop done, rotating 180\r\n");
      }
      break;

    case 4U:  /* 原地旋转180° */
      if (Car_StartRotateAngle(VISION_TURN_SPEED, 1800) != CAR_ACTION_BUSY)
      {
        printf("[VTASK] warehouse: 180 turn rejected\r\n");
        VisionTask_Finish();
        return;
      }
      warehouse_phase = 5U;
      break;

    case 5U:  /* 等待180°完成 */
      switch (Car_ActionStep())
      {
        case CAR_ACTION_BUSY:
          break;
        case CAR_ACTION_DONE:
          warehouse_phase = 6U;
          task_phase_tick = now;
          printf("[VTASK] warehouse: 180 done, going forward\r\n");
          break;
        default:
          printf("[VTASK] warehouse: 180 failed\r\n");
          VisionTask_Finish();
          break;
      }
      break;

    case 6U:  /* 前进距离 x：与进库直行段等时长，回到进库前的位置 */
      Car_ForwardRun(VISION_SPEED_NORMAL);
      if ((uint32_t)(now - task_phase_tick) >= warehouse_fwd_ms)
      {
        warehouse_phase = 7U;
        Car_Stop();
        printf("[VTASK] warehouse: forward x=%u ms done, turning right\r\n",
               (unsigned)warehouse_fwd_ms);
      }
      break;

    case 7U:  /* 右转90° */
      if (Car_StartRotateAngle(VISION_TURN_SPEED, -(int32_t)VISION_TURN_ANGLE_DEG10)
          != CAR_ACTION_BUSY)
      {
        printf("[VTASK] warehouse: right turn rejected\r\n");
        VisionTask_Finish();
        return;
      }
      warehouse_phase = 8U;
      break;

    case 8U:  /* 等待右转完成 */
      switch (Car_ActionStep())
      {
        case CAR_ACTION_BUSY:
          break;
        case CAR_ACTION_DONE:
          printf("[VTASK] warehouse: all done\r\n");
          VisionTask_Finish();
          break;
        default:
          printf("[VTASK] warehouse: right turn failed\r\n");
          VisionTask_Finish();
          break;
      }
      break;

    default:
      VisionTask_Finish();
      break;
  }
}

uint8_t VisionTask_StartCallAlly(void)
{
  /* 一次性：用过就不再受理，避免反复按键把 8s 无限续期。 */
  if (call_ally_used != 0U)
  {
    printf("[VTASK] horn alarm ignored: already used (reset to re-arm)\r\n");
    return 0U;
  }

  call_ally_used       = 1U;
  call_ally_active     = 1U;
  call_ally_start_tick = HAL_GetTick();
  task_state            = VISION_TASK_CALL_ALLY;

  /* 原地呼唤：先把底盘停住，Step() 里每轮维持。 */
  Car_ActionAbort();
  Car_Stop();
  VisionTask_CallAllyApply(1U);

  printf("[VTASK] horn alarm started (%u ms)\r\n",
         (unsigned)VISION_CALL_ALLY_DURATION_MS);
  return 1U;
}

uint8_t VisionTask_StartTurnAround(void)
{
  Car_ActionAbort();

  if (Car_StartRotateAngle(VISION_TURN_SPEED,
                           (int32_t)VISION_TURN_AROUND_DEG10) != CAR_ACTION_BUSY)
  {
    printf("[VTASK] turn around rejected\r\n");
    return 0U;
  }

  VisionTask_Start(VISION_TASK_TURN_AROUND);
  task_phase = 1U;   /* 旋转已发起，直接进等待相位 */
  return 1U;
}

/**
  * @brief  补给掉头：等旋转完成，超时兜底后交还循迹
  */
static void VisionTask_RunTurnAround(void)
{
  if ((uint32_t)(HAL_GetTick() - task_start_tick) >=
      VISION_TURN_AROUND_TIMEOUT_MS)
  {
    printf("[VTASK] turn around timeout\r\n");
    Car_ActionAbort();
    Car_Stop();
    VisionTask_Finish();
    return;
  }

  switch (Car_ActionStep())
  {
    case CAR_ACTION_BUSY:
      break;

    case CAR_ACTION_DONE:
      Car_Stop();
      /* 掉头后沿原路返回会遇到来时的岔道，武装"全黑→全白→左转"规则。 */
      LineTracker_ArmForkLeft();
      printf("[VTASK] turn around done, fork-left armed, back to line track\r\n");
      VisionTask_Finish();
      break;

    default:
      printf("[VTASK] turn around failed\r\n");
      Car_Stop();
      VisionTask_Finish();
      break;
  }
}

uint8_t VisionTask_Step(void)
{
  switch (task_state)
  {
    case VISION_TASK_STOPPED:
      Car_Stop();
      return 1U;

    case VISION_TASK_FRIENDLY:
      VisionTask_RunFriendly();
      return 1U;

    case VISION_TASK_NO_ENTRY:
      VisionTask_RunNoEntry();
      return 1U;

    case VISION_TASK_WAREHOUSE:
      VisionTask_RunWarehouse();
      return 1U;

    case VISION_TASK_CALL_ALLY:
      /* 原地呼唤：底盘全程停住。灯笛节拍由 VisionTask_Tick() 驱动。 */
      Car_Stop();
      return 1U;

    case VISION_TASK_TURN_AROUND:
      VisionTask_RunTurnAround();
      return 1U;

    default:
      return 0U;
  }
}

void VisionTask_Cancel(void)
{
  if (task_state != VISION_TASK_IDLE)
  {
    printf("[VTASK] cancelled at %s\r\n", VisionTask_StateName(task_state));
  }
  Car_ActionAbort();
  VisionTask_Alert(0U);
  VisionTask_Buzzer(0U);
  task_state = VISION_TASK_IDLE;
  task_phase = 0U;
  warehouse_zero_count = 0U;
  warehouse_phase = 0U;
  warehouse_fwd_ms = 0U;
  call_ally_active = 0U;
  Indicator_Release(INDICATOR_PRIO_CALL_ALLY);
  /* call_ally_used 有意不清：呼唤全程只允许一次，急停也不该让它复活。 */

  signal_tunnel_until = 0U;
  signal_slow_until = 0U;
  signal_flash_until = 0U;
  signal_narrow_until = 0U;
  friendly_gap_tick = 0U;
  task_speed = VISION_SPEED_NORMAL;
  BuzzerTone_Stop();
  VisionTask_SignalLightApply();
}

uint8_t VisionTask_IsBusy(void)
{
  return (task_state != VISION_TASK_IDLE) ? 1U : 0U;
}