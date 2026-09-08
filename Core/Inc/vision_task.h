#ifndef __VISION_TASK_H__
#define __VISION_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "vision_uart.h"
#include "event.h"

/* ==== 速度配置 ============================================================== */
#define VISION_SPEED_NORMAL        64U
#define VISION_SPEED_LIMIT_DROP    4U

/* 转向：原地旋转90.0°，单位0.1度。 */
#define VISION_TURN_SPEED          80U
#define VISION_TURN_ANGLE_DEG10    900U

/* ==== 信号类动作 ============================================================ */
#define VISION_TUNNEL_LIGHT_MS     4000U
#define VISION_SLOW_AHEAD_MS       5000U

/* 狭窄街道：黄灯闪烁时长。到点自动释放指示层，不会一直挂着。 */
#define VISION_NARROW_BLINK_MS     2000U

/* ==== 友军信号 ==============================================================
   停车 + 鸣旋律 → 放完即原速启动。

   旋律本身只有 540ms（BUZZER_MELODY_FRIENDLY: 110+110+110+30+180），单靠
   "边走边响"太短、听不出来，所以改成占底盘：停下来响完再走。

   每遍不写死时长，而是等 BuzzerTone_IsPlaying() 变 0 再起下一遍，这样以后
   改旋律表不用同步改这里。TIMEOUT 只是兜底，防止蜂鸣器被 suspend 时卡住。
   ============================================================================ */

/* 旋律循环遍数。单遍 540ms 太短听不清，放四遍约 2.4s。 */
#define VISION_FRIENDLY_LOOPS       4U

/* 两遍之间的静音间隔，让人耳分得出是四遍而不是一长串。 */
#define VISION_FRIENDLY_LOOP_GAP_MS 120U

/* 兜底总时长。四遍 ×(540+120) ≈ 2.6s，留足余量。 */
#define VISION_FRIENDLY_TIMEOUT_MS  5000U

/* ==== 禁止通行 ============================================================== */
#define VISION_NO_ENTRY_BACK_MS    500U    /* 倒车时间 */
#define VISION_NO_ENTRY_FWD_MS     100U    /* 前进时间 */

/* ==== 仓库动作 ==============================================================
   流程：左转90° → 直行（时长记为 x）→ 连续 8 个采样周期全白(0000) → 停 3s
         → 原地180° → 前进 x → 右转90° → 交还循迹

   x 是实测量，不是常数：直行段走多久，回程就走多久，这样才能回到进库前的
   位置。VISION_WAREHOUSE_FWD_MAX_MS 只是回程的兜底上限，防止 x 因为异常
   （比如一直没读到全白）大到让车冲出赛道。
   ============================================================================ */
#define VISION_WAREHOUSE_STOP_MS       3000U   /* 到位后停止时间 */
#define VISION_WAREHOUSE_ZERO_COUNT    8U      /* 判定到位的连续全白次数 */

/* 全白计数的采样周期。VisionTask_Step() 由主循环每轮调用（几十微秒一轮），
   不加这个节拍，"8 个周期"只有零点几毫秒，等于一读到全白立刻就判到位，
   传感器抖一下就误触发。取 10ms 与 LINE_CONTROL_PERIOD_MS 对齐，
   8 × 10ms = 80ms 才算真的到位。 */
#define VISION_WAREHOUSE_SAMPLE_MS     10U

/* 直行段计时上限，同时也是回程 x 的上限。 */
#define VISION_WAREHOUSE_FWD_MAX_MS    6000U

/* ==== 补给掉头（遥控"8"）====================================================
   收到信号后原地掉头 180°，完成即交还循迹。

   做成任务状态而不是在 scheduler 里阻塞等待：原来那段 while + HAL_Delay(10)
   在转完之前（最坏 5s）不出 Scheduler_OnEvent，期间事件队列不消费、循迹不
   推进、避障不响应，连急停键都进不来。
   ============================================================================ */
#define VISION_TURN_AROUND_DEG10       1800U   /* 掉头角度，单位0.1° */
#define VISION_TURN_AROUND_TIMEOUT_MS  5000U   /* 编码器不到位时的兜底 */

/* ==== 呼唤友军（遥控"9"）====================================================
   图案：连续两次短闪 → 短暂熄灭 → 重复，直到 8s 结束。灯与笛同步。

   一个完整循环 = 闪 + 灭 + 闪 + 暗，即
       ON + GAP + ON + DARK = 150+150+150+550 = 1000ms
   8s 正好跑 8 个循环。改这几个值时留意别让 DARK 短到跟 GAP 分不出来，
   否则看着就是等周期单闪，失去"两短闪"的辨识度。
   ============================================================================ */
#define VISION_CALL_ALLY_DURATION_MS  8000U   /* 呼唤总时长8s */
#define VISION_CALL_ALLY_ON_MS        150U    /* 单次短闪亮的时长 */
#define VISION_CALL_ALLY_GAP_MS       150U    /* 两次短闪之间的熄灭 */
#define VISION_CALL_ALLY_DARK_MS      550U    /* 一组之后的短暂熄灭 */

/* 单个循环周期，由上面四段推出，不要单独改。 */
#define VISION_CALL_ALLY_CYCLE_MS     (VISION_CALL_ALLY_ON_MS * 2U + \
                                        VISION_CALL_ALLY_GAP_MS +     \
                                        VISION_CALL_ALLY_DARK_MS)

typedef enum
{
  VISION_TASK_IDLE = 0,
  VISION_TASK_STOPPED,

  /* 新增任务 */
  VISION_TASK_FRIENDLY,            /* 友军信号：停车鸣笛，放完原速启动 */
  VISION_TASK_NO_ENTRY,            /* 禁止通行 */
  VISION_TASK_WAREHOUSE,           /* 仓库 */
  VISION_TASK_CALL_ALLY,          /* 呼唤友军：原地鸣笛闪灯8s */
  VISION_TASK_TURN_AROUND         /* 补给：原地掉头180°，完成后交还循迹 */
} VisionTask_State;

void VisionTask_Init(void);

uint8_t VisionTask_Begin(Event_Type type);
uint8_t VisionTask_Step(void);
void VisionTask_Cancel(void);
uint8_t VisionTask_IsBusy(void);
void VisionTask_Tick(void);
uint8_t VisionTask_GetSpeed(void);
VisionTask_State VisionTask_GetState(void);
const char *VisionTask_StateName(VisionTask_State state);

/* 呼唤友军：原地鸣笛闪灯 8s。
   全程只接受一次——收到并成功启动后，除 MCU 复位外不再响应第二次按键
   （VisionTask_Init() 是唯一的清零点）。
   返回 1=本次已启动，0=已经用过或正在呼唤中，被忽略。 */
uint8_t VisionTask_StartCallAlly(void);

/* 补给掉头：原地 180°，非阻塞。返回 1=已启动，0=底盘正忙未受理。 */
uint8_t VisionTask_StartTurnAround(void);

#ifdef __cplusplus
}
#endif

#endif /* __VISION_TASK_H__ */