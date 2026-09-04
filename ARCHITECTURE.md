# AutoCar 控制架构

STM32F103 智能小车。本文描述目标架构：**主循环调度 + 各模块独立状态机 + 非阻塞动作**，
以及从当前实现迁移过去的步骤。

本文是设计文档，不是现状说明。凡是尚未落地的部分，均在 §2 的差距清单和 §10 的
改造阶段中明确标注。

---

## 1. 设计目标

1. **任何模块都不得独占 CPU。** 主循环每轮都要能重新仲裁优先级，否则高优先级事件
   （急停、避障）只能等低优先级动作做完才被看到。
2. **同一周期只有一个模块写电机。** 控制权由调度器授予，不靠模块之间自觉。
3. **共享外设有唯一属主。** 蜂鸣器和 RGB 当前被四处直写，互相覆盖。
4. **每个动作都有超时和失败出口。** 失败一律制动停车，并向上报告。
5. **调试入口保留。** 阻塞式调试函数不删除，只是不出现在正式主循环里。

---

## 2. 当前实现与目标的差距

### 2.1 阻塞点清单

改造的实际工作量集中在这里。`√` 表示已符合目标架构。

| 位置 | 阻塞形式 | 最坏阻塞时长 |
| --- | --- | --- |
| `car.c:210` `Car_RunDistance` | `while(1)` + `HAL_Delay(1)` | `CAR_DISTANCE_TIMEOUT_MS` = 60 s |
| `car.c:335` `Car_RunAngle` | `while(1)` + `HAL_Delay(1)` | 60 s |
| `car.c:177` `Car_Forward/Backward` | `HAL_Delay(time)` | 参数 time |
| `car.c:470` `Car_ForwardVary` | 循环 `HAL_Delay(20)` | 参数 time |
| `car.c:578` `Car_Brake` | `HAL_Delay(time)` | 参数 time |
| `ir_avoid.c:95,102` `IrAvoid_Update` | 每次采样两段发射管稳定延时 | 2 × settle |
| `ultrasonic.c:46,56` `Ultrasonic_ReadMm` | 忙等 ECHO 电平 | 量程上限对应回波时间 |
| `line_tracker.c:89` `LineTracker_Run` | `while(1)` | 无限（调试入口，允许） |
| `oled.c:49` `Oled_Update` | I2C 阻塞发送整屏 | 数 ms（诊断用，见 §9.1） |
| `ultrasonic_avoid.c` | — | √ 已是 tick 判周期 |
| `vision_task.c` 鸣笛/闪灯 | — | √ 已是相位状态机 |

两个连带后果：

- **`LineTracker_Step()` 名义上是单步接口，实际是阻塞的。** 它在
  `line_tracker.c:54` 起的各分支调用 `Car_RotateLeftAngle()` / `Car_RotateRightAngle()`，
  每次修正都要等编码器转够角度才返回。文档此前把它描述为"可被抢占"，与实现不符。
- **`vision_task.c:224,229` 的转弯任务同理。** 注释已说明这是阻塞的，属于已知待改造项。

### 2.2 共享资源无属主（阶段 3 已解决）

蜂鸣器和 RGB 目前有四个写入方，互相覆盖且无优先级：

| 模块 | 写入点 |
| --- | --- |
| `ir_avoid` | `IrAvoid_UpdateAlert()`，无障碍时无条件 `RGB_SetColor(0,0,0)` |
| `ultrasonic_avoid` | `UltrasonicAvoid_Alert()` |
| `vision_task` | `VisionTask_Alert()` / `VisionTask_Buzzer()` |
| `main.c:108` | 按键 EXTI 回调直接写 GPIO |

后果：入库闪灯期间若红外判无障碍，`IrAvoid_UpdateAlert(CLEAR)` 会把灯灭掉；
按键中断也能随时改写。

### 2.3 编码器基准是全局的

`Car_RunDistance` 和 `Car_RunAngle` 都调 `Encoder_ResetAll()`（`car.c:201,319`）。
一旦改成非阻塞、允许"避障动作打断任务动作"，两个动作各自 reset 会互相破坏基准，
而且任何想独立读里程的模块都会被清零。非阻塞化时必须改为**动作内部保存起始快照**，
不再清零全局计数。

### 2.4 已知逻辑缺陷

- **`VISION_TASK_STOPPED` 出不去**（`vision_task.c:245`）。设计意图是"停在库里，
  直到识别到新的视觉任务再判定状态"，但 `VisionTask_AcceptEvent()` 建任务的条件
  全是 `task_state == IDLE`，落到 `STOPPED` 后所有任务类事件被静默丢弃，
  `Handle()` 恒返回 1。**意图本身保留**，只需放开 `STOPPED` 也可接受新任务事件。
- 事件槽只有一个（`vision_uart.c` 的 `vision_event` + `ready` 标志）。任务执行
  期间到达的第二个事件会覆盖第一个，且无人知情。
- 帧同步无超时重整。丢字节后若 id 或校验位恰为 `0xAA`，要等本帧走完才能重新对齐。
- 红外遥控解出的 action 无人消费，见 §8.2。

---

## 3. 分层结构

```
传感器 / 识别模块      ir_avoid  ultrasonic  line_sensor  vision_uart  key
        ↓ 只产生状态或事件，不写电机
事件队列 + 调度器      main 循环：取事件 → 改控制权 → 按模式分派
        ↓ 每周期只授予一个模块控制权
行为状态机层           line_tracker  vision_task  avoid_task  emergency
        ↓ 只发起 / 推进底盘动作
底盘动作层             car：非阻塞动作状态机（Start / Step / Abort）
        ↓
电机 / 编码器          tim  encoder
```

旁路一条：**指示层 `indicator`**（蜂鸣器 + RGB）由调度器统一裁决，见 §7。

分层的硬约束：

- 下层不认识上层。`car` 不知道任务优先级，`encoder` 不知道谁在用它。
- 识别模块不调 `car`。`ir_avoid` 当前既采样又直接旋转，要拆开（§8）。
- 只有获得控制权的行为模块能调 `car` 的动作接口。

---

## 4. 底盘非阻塞动作模型

这是整个改造的基石：**上层状态机再漂亮，只要 `car` 是阻塞的，主循环就还是卡住。**

### 4.1 接口形态

每个长动作从"一个阻塞函数"变成"发起 + 推进"两个接口：

```c
typedef enum
{
  CAR_ACTION_IDLE = 0,    /* 无动作 */
  CAR_ACTION_BUSY,        /* 进行中，需要继续 Step */
  CAR_ACTION_DONE,        /* 正常完成，已制动 */
  CAR_ACTION_TIMEOUT,     /* 超时退出，已制动 */
  CAR_ACTION_FAILED       /* 参数非法或编码器无脉冲，已制动 */
} Car_ActionStatus;

/* 发起动作。同一时刻只允许一个动作，重复发起返回 FAILED。 */
Car_ActionStatus Car_StartForwardDistance(uint8_t speed, uint32_t distance_mm);
Car_ActionStatus Car_StartRotateAngle(uint8_t speed, int32_t angle_deg10);
Car_ActionStatus Car_StartTurnAngle(uint8_t speed, uint16_t radius_mm,
                                    int32_t angle_deg10);

/* 每轮主循环推进一次。返回当前状态；DONE/TIMEOUT/FAILED 时内部已制动并转 IDLE。 */
Car_ActionStatus Car_ActionStep(void);

/* 立即放弃当前动作并制动。高优先级抢占时调用。 */
void Car_ActionAbort(void);

Car_ActionStatus Car_GetActionStatus(void);
```

旋转和转弯用**带符号角度**（正左负右）合并成一个接口，替掉现在
`Car_RotateLeftAngle` / `Car_RotateRightAngle` 两套几乎重复的实现。

### 4.2 内部状态

动作状态机保存：目标量、起始编码器快照、起始 tick、超时上限、当前阶段。
**不再调用 `Encoder_ResetAll()`**，改为记录起点、每次 Step 算增量，解决 §2.3。

`Car_ActionStep()` 的职责：读编码器算增量 → 判是否达标 → 判是否超时 →
未完成则维持 PWM 并返回 BUSY，完成则制动并返回终态。原来 `while(1)` 里
每 500 ms 的 printf 诊断保留，改为在 Step 里按 tick 判周期输出。

### 4.3 瞬时动作保持原样

`Car_ForwardRun()`、`Car_Stop()`、`Car_Drive()` 本来就是"设置 PWM 后立即返回"，
不需要改。它们是行为模块每周期直接调用的稳态接口。

`Car_Brake(time)` 拆成 `Car_Brake()`（立即制动、立即返回）和调用方自己的计时，
或提供 `Car_StartBrake(time)` 走同一套动作状态机。

### 4.4 调试接口保留

现有阻塞版 `Car_ForwardDistance()` 等**不删除**，改为在内部实现为
"Start + 循环 Step 直到终态"的薄包装，供独立标定使用。这样两套接口共享同一份
逻辑，不会出现标定值在两处不同步。

---

## 5. 事件与调度

### 5.1 事件队列

单事件槽改成环形队列，容量足够容纳一次任务执行期间可能到达的事件：

```c
typedef enum
{
  EVENT_NONE = 0,
  EVENT_VISION_SPEED_LIMIT,   EVENT_VISION_SPEED_RELEASE,
  EVENT_VISION_TURN_LEFT,     EVENT_VISION_TURN_RIGHT,
  EVENT_VISION_HORN,
  EVENT_VISION_PARK_1,        EVENT_VISION_PARK_2,
  EVENT_OBSTACLE_LEFT,        EVENT_OBSTACLE_RIGHT,
  EVENT_OBSTACLE_BOTH,        EVENT_OBSTACLE_CLEAR,
  EVENT_KEY_START,            EVENT_KEY_STOP,
  EVENT_REMOTE_FORWARD,       EVENT_REMOTE_ROTATE_LEFT,
  EVENT_REMOTE_ROTATE_RIGHT,  EVENT_REMOTE_HORN
} Event_Type;

typedef struct { Event_Type type; uint8_t param; uint32_t tick; } Event;

uint8_t Event_Post(Event_Type type, uint8_t param);  /* 中断安全，满则丢弃并计数 */
uint8_t Event_Pop(Event *out);
```

`Event_Post()` 要能从中断调用，入队只做指针搬移，**不 printf**（沿用
`vision_uart.c` 现在的正确做法）。队列满时丢弃并累加计数器，供诊断查看。

### 5.2 主循环骨架

```c
while (1)
{
  /* 1. 采样层：各识别模块推进自己的采样状态机，只产生事件 */
  IrAvoid_Sense();          /* 非阻塞采样，状态变化时 Event_Post */
  Ultrasonic_Sense();
  VisionUart_Drain();       /* 把中断收到的帧转成事件 */

  /* 2. 调度层：消费事件，决定控制模式 */
  Event e;
  while (Event_Pop(&e))
  {
    Scheduler_OnEvent(&e);
  }

  /* 3. 分派：只有一个模块获得控制权 */
  switch (Scheduler_GetMode())
  {
    case CONTROL_STOP:       Emergency_Step();    break;
    case CONTROL_MANUAL:     ManualTask_Step();   break;
    case CONTROL_IR_AVOID:   AvoidTask_Step();    break;
    case CONTROL_TASK:       VisionTask_Step();   break;
    case CONTROL_LINE_TRACK: LineTracker_Step();  break;
    case CONTROL_IDLE:       Car_Stop();          break;
  }

  /* 4. 指示层：按当前模式和任务裁决蜂鸣与 RGB */
  Indicator_Step();

  /* 5. 诊断输出，按 tick 判周期 */
  Diag_Step();
}
```

没有 `HAL_Delay`。每轮耗时由最慢的 Step 决定，都是"读几个 GPIO/ADC + 算一次"，
量级在几十微秒。是否需要固定周期（例如 5 ms 一轮）取决于循迹控制律的需要，
可以在循环末尾加一个"等到下一个 tick 边界"的让步，而不是无条件 delay。

---

## 6. 控制模式状态机

### 6.1 模式定义

```c
typedef enum
{
  CONTROL_IDLE = 0,    /* 未启动，等待启动键 */
  CONTROL_LINE_TRACK,  /* 默认行为：循迹 */
  CONTROL_TASK,        /* 视觉任务占用底盘 */
  CONTROL_IR_AVOID,    /* 避障占用底盘 */
  CONTROL_MANUAL,      /* 红外遥控手动接管（调试用） */
  CONTROL_STOP         /* 急停 / 故障 */
} Control_Mode;
```

优先级固定：`CONTROL_STOP` > `CONTROL_MANUAL` > `CONTROL_IR_AVOID` >
`CONTROL_TASK` > `CONTROL_LINE_TRACK` > `CONTROL_IDLE`。

`CONTROL_MANUAL` 高于避障是有意的：遥控是调试手段，按下遥控就该立刻拿到底盘，
不能被避障顶回去。代价是**手动模式下避障不再保护车体**，撞不撞由操作者负责；
急停仍然高于手动，是最后的兜底。

### 6.2 转移表

| 当前模式 | 事件 | 新模式 | 附带动作 |
| --- | --- | --- | --- |
| 任意 | `KEY_STOP` / 故障 | `STOP` | `Car_ActionAbort()`，清除全部任务 |
| 非 `STOP`、非 `MANUAL` | RED 双击 | `MANUAL` | `Car_ActionAbort()`，记录被抢占者 |
| `MANUAL` | RED 双击 | 被抢占者 | `Car_Stop()` |
| 非 `MANUAL` | 其他 `REMOTE_*` | 不变 | 忽略，遥控不介入 |
| `IDLE` | `KEY_START` | `LINE_TRACK` | — |
| `LINE_TRACK` | `OBSTACLE_*` | `IR_AVOID` | 记录被抢占者 = `LINE_TRACK` |
| `LINE_TRACK` | 任务类视觉事件 | `TASK` | `VisionTask_Begin()` |
| `LINE_TRACK` | 限速类视觉事件 | 不变 | 只改巡线速度上限 |
| `TASK` | `OBSTACLE_*` | `IR_AVOID` | `VisionTask_Suspend()`，记录被抢占者 = `TASK` |
| `TASK` | 任务完成 | `LINE_TRACK` | — |
| `TASK` | 同类任务事件 | 不变 | 丢弃，避免反复重启动作 |
| `IR_AVOID` | 避障完成 | 见 §6.3 | — |
| `STOP` | `KEY_START` | `LINE_TRACK` | 重新初始化各模块 |

限速是**持续配置**，不是任务，不参与模式切换——这一点现有实现已经做对了。

### 6.3 避障结束后返回哪里

实际赛道上转弯任务与避障几乎不会同时出现，因此**先用最简策略**：避障完成后
回到 `preempted_mode`，任务从被打断的相位继续。

不做多档恢复策略。真要区分时，加一个"任务是否位姿相关"的查询即可——转弯、入库
位姿相关（避障动过车身，应作废回循迹），鸣笛无关（接着响完）。留作后续需要时再补，
不在本轮引入。

### 6.4 被抢占者只需记一层

避障可以打断任务，但避障自身不会被同级打断，急停会清空一切。所以只需要一个
`preempted_mode` 变量，不需要模式栈。

---

## 7. 指示层

蜂鸣器和 RGB 改为"申请 + 裁决"，解决 §2.2：

```c
typedef enum
{
  INDICATOR_PRIO_NONE = 0,
  INDICATOR_PRIO_TASK,      /* 入库闪灯、鸣笛 */
  INDICATOR_PRIO_AVOID,     /* 避障告警 */
  INDICATOR_PRIO_FAULT      /* 急停 */
} Indicator_Prio;

void Indicator_Request(Indicator_Prio prio, uint8_t buzzer,
                       uint8_t rgb_left, uint8_t rgb_right, uint16_t blink_ms);
void Indicator_Release(Indicator_Prio prio);
void Indicator_Step(void);   /* 取最高优先级的有效申请，实际写 GPIO */
```

各模块只申请，不直接写 GPIO。闪烁相位由 `Indicator_Step()` 统一维护，模块只给
周期。`main.c` 的按键回调也改成 `Event_Post()` + 申请，不再直接写引脚。

顺带修掉 `RGB_SetColor()` 里 `r/g` 与左右侧交叉映射那个坑（`main.c:98`：`r` 参数
实际控制右侧红灯、`g` 控制左侧），指示层接口直接用 `rgb_left` / `rgb_right` 命名。

---

## 8. 识别模块的拆分

**`ir_avoid` 现在既是传感器又是执行器**（`IrAvoid_Handle()` 里直接
`Car_BackwardDistance()` + `Car_RotateRightAngle()`），违反分层。拆成两个：

- **`ir_avoid`（传感器）**：只保留采样、阈值、迟滞、状态判断，状态变化时
  `Event_Post(EVENT_OBSTACLE_*)`。对外接口 `IrAvoid_Sense()` + 若干 getter。
  发射管稳定延时改为采样状态机的相位（使能 → 等 settle → 读 ADC → 关闭 → 换边），
  消掉 `HAL_Delay`。
- **`avoid_task`（行为）**：拥有避障动作状态机，只在获得 `CONTROL_IR_AVOID`
  控制权时推进。阶段：`BACKING`（双侧障碍才有）→ `RESAMPLE` → `ROTATING` → `DONE`，
  每阶段通过 `Car_Start*` + `Car_ActionStep()` 驱动。避障参数留在 `avoid_task`。

超声波同理：`ultrasonic` 只测距，`ultrasonic_avoid` 的动作逻辑并入 `avoid_task`，
两种传感器发同一族 `EVENT_OBSTACLE_*` 事件，避障行为只写一份。

`ultrasonic.c` 忙等 ECHO 那段（§2.1）最终应改成定时器输入捕获 + 中断，测距完成
时回调。这是独立的一步，可以放到后面做；在它改完之前，忙等时长受量程上限约束，
是本方案里唯一可接受的残留阻塞。

**两路传感器冲突规则**：超声波只能报正前方，红外报左 / 右 / 双侧，语义不重叠。
同时报障时**前方优先**——超声波报障就按前方障碍处理（后退再选向），否则按红外的
左右状态处理。避障行为逻辑本身后续还要改，这条规则先按最简实现。

### 8.1 循迹改为连续修正

现在的 `LineTracker_Step()` 偏离时调 `Car_RotateLeftAngle(90, 50)`：**停下、原地
转 5°、再继续**。这个"停—转—走"步态有两处硬伤：

- **转动期间完全不读传感器**（阻塞在 `Car_RunAngle` 的 `while` 里）。反馈回路中
  插了一段死时间，这是循迹振荡的主要来源。
- 修正量只有 5° / 10° / 15° 三档，且要靠 `CAR_ROT_COMP_*` 滑移补偿表把真实角度
  换算成编码器角度，而那张表随电量、地面、轮胎变化。

改为差速连续修正：每个控制周期读四路传感器算偏差，直接 `Car_Drive(left, right)`
写一次轮速就返回。车**一直在前进**，靠左右轮速差纠正航向。

`LINE_CORRECT_ANGLE_DEG10`（角度）随之换成速度差参数；循迹不再依赖
`CAR_ROT_COMP_*` 滑移补偿表。原 `LineTracker_Run()` 保留为调试入口。

#### 控制周期

`LINE_SAMPLE_PERIOD_MS` 沿用 10 ms，但语义从"采样后 delay 10 ms"变成
**固定控制周期**：主循环每轮都会调 `LineTracker_Step()`，未到 10 ms 就直接返回、
维持上次轮速。建议改名为 `LINE_CONTROL_PERIOD_MS` 以免误解。

10 ms 够用的理由：电机从改占空比到转速跟上是几十毫秒量级，控制周期比它短就不是
瓶颈；周期再短只是重复算同一个结果。**关键是周期固定**——差速增益的标定值只在
固定周期下才有意义，这也是必须由循迹自己判周期、不能跟随主循环节拍的原因。

#### 会不会更容易冲出轨道

这是改造的主要风险点，答案分两面。

**控制品质本身是变好的**，两个原因：死时间消失（原来原地转 5° 的整段时间里传感器
是瞎的），修正量从 3 档离散变成连续可调。死时间是反馈系统失稳的首要因素，去掉它
等于提高稳定裕度。

**但确实存在一个真实风险，来源不是"非阻塞"而是"车真的跑起来了"。** 现在的步态
每次修正都要停车原地转，平均对地速度远低于 `LINE_FORWARD_SPEED = 60` 所暗示的
速度。改成连续修正后车不再停顿，实际速度显著提高，**原来够用的修正力度在新速度
下可能不够**，表现就是弯道压不住、冲出轨道。

对策：
- 阶段 5 上车调试时**先把 `LINE_FORWARD_SPEED` 降下来**（例如 55，刚出死区），
  把差速增益调到能稳定跟线，再逐步提速。不要一开始就用 60 试。
- 增益标定顺序：直线不摆动 → 缓弯不切内道 → 再提速。
- 加"丢线保护"：连续若干周期读到全白（`0x0F`）判定丢线，立即制动并报事件，
  而不是带着上次轮速继续冲。现在的 `LINE_STOP_ON_UNKNOWN` 只覆盖单次未定义
  图案，没有持续判定。

#### 直角弯需要单独处理

纯差速修正在**直角弯**上会力不从心：前进中要转过 90°，需要内侧轮反转，本质已是
原地旋转。极端图案（`0x07` / `0x0E` 极左极右，以及全黑 `0x00`）应保留原地转向，
但改用非阻塞的 `Car_StartRotateAngle()` + `Car_ActionStep()`，这样转向期间仍能被
急停和手动模式抢占，也仍在每轮读传感器判断是否已经压线。

即：**小偏差走差速、大偏差走非阻塞原地转**的混合策略，而不是全盘替换。两者的
切换阈值是需要实车标定的参数。

### 8.2 红外遥控

`ir_remote` 已用 EXTI 完成 NEC 解码，`ir_remote_map` 已把按键映射为
`FORWARD` / `ROTATE_LEFT` / `ROTATE_RIGHT` / `HORN`，但**当前没有任何模块消费这些
action**——只有 `ir_remote_debug` 打印，而它在 `main.c:301` 处于注释状态。
遥控实际是"解码链路已通、控制链路未接"。

接入方式与其他识别模块一致：解码完成时 `Event_Post(EVENT_REMOTE_*)`。

**定位：调试用手动接管，优先级仅低于急停**（§6.1）。

**进入 / 退出只认「RED 键连按两次」**（RED = command `0x00`，见
`ir_remote_map.c:7`）。这是一个开关：不在手动模式时双击 RED 接管，
在手动模式时双击 RED 交还。**其他任何遥控键都不会让遥控介入**——
不在手动模式时，方向键、HORN 全部忽略，自动驾驶不受干扰。

双击判定：两次 RED 的间隔小于 `MANUAL_DOUBLE_MS`（建议 600 ms）算一次双击。
NEC 长按发的 repeat 帧**不计入**次数，否则按住 RED 会被误判成连击。

`ManualTask_Step()` 的行为：

- 接管瞬间 `Car_ActionAbort()` 打断当前动作，并记录被抢占的模式。
- `FORWARD` → `Car_ForwardRun()`；`ROTATE_LEFT/RIGHT` → `Car_Drive()` 反向差速。
  都用瞬时接口。按键静默超过 `MANUAL_KEY_TIMEOUT_MS`（建议 300 ms，约一个 NEC
  repeat 间隔）就 `Car_Stop()`，**但仍留在手动模式**——松手停车，不退出。
- `HORN` → 通过指示层申请蜂鸣（§7），不占底盘。
- 交还时 `Car_Stop()`，回到被抢占的模式。

"松手停车"和"退出手动"是两件事：前者靠按键静默超时，后者只靠双击 RED。这样
遥控丢帧只会让车停下，不会把控制权莫名交还给自动驾驶。

`ir_remote.c` 的帧槽是单槽（`ir_remote_repeat` + 帧缓冲），连按会覆盖；接入事件
队列后这个问题自然消失。遥控的 OLED 显示属于诊断，归 §9.1。

---

## 9. 故障与超时

- 每个 `Car_Start*` 动作带超时上限，`Car_ActionStep()` 到期返回 `TIMEOUT` 并制动。
- 行为状态机收到 `TIMEOUT` / `FAILED`：任务标记 `FAILED`，交回控制权；
  连续失败达阈值则 `Event_Post` 故障事件，调度器进 `CONTROL_STOP`。
- 每个任务另有整体超时，防止相位状态机卡在某一相。
- `CONTROL_STOP` 下 `Emergency_Step()` 持续制动 + 故障指示，只有启动键能退出。

### 9.1 诊断输出不能拖慢主循环

`printf` 走 USART1 阻塞发送，`Oled_Update()` 走 I2C 阻塞发送整屏。两者都不能
每轮无条件调用。规则：

- 全部诊断集中在 `Diag_Step()`，按 tick 判周期（现有代码已是这个做法）。
- 车辆运动期间不刷 OLED；OLED 只在 `CONTROL_IDLE` / `CONTROL_STOP` 或调试模式下更新。
- 中断里绝不 `printf`，只累加计数器（`vision_uart.c` 已正确处理，保持）。

---

## 10. 改造阶段

每阶段结束后工程可编译、可上车验证，不留中间断裂状态。

| 阶段 | 内容 | 验证方式 |
| --- | --- | --- |
| ~~1~~ | **已完成** `car` 非阻塞动作状态机（§4），阻塞版改为薄包装 | 用现有阻塞接口跑标定，结果应与改造前一致 |
| ~~2~~ | **已完成** 事件队列 + 调度器 + 模式机 + 手动接管（§5、§6、§8.2） | 视觉事件仍能触发任务，主循环不再有 `HAL_Delay` |
| ~~3~~ | **已完成** 指示层（§7） | 入库闪灯不再被红外清零 |
| ~~4~~ | **已完成** 拆 `ir_avoid` 为传感器 + `avoid_task`（§8） | 避障动作期间仍能响应急停 |
| 5 | 循迹改差速连续修正（§8.1）**从低速起调** | 直线不摆动 → 缓弯不切内道 → 再提速 |
| 6 | 超声波输入捕获（§8 末） | 消除最后一处忙等 |
| 7 | 修 §2.4 缺陷；按 §8.2 接入遥控 | 1 号库后仍能接受新事件；连发事件不丢 |

阶段 1 是硬前提：`car` 不非阻塞化，上面的调度器都是空架子。

### 阶段 2 落地情况

新增 `event`（环形队列）、`scheduler`（模式机）、`manual_task`（遥控接管）三个模块。
`vision_task` 从 `Handle()`（自己读事件 + 跑状态机）拆成 `Begin(Event_Type)` +
`Step()`，任务创建权交给调度器。转弯任务已改为非阻塞
（`Car_StartRotateAngle` + `Car_ActionStep`）。

与阶段 2 计划的两处偏差，都是刻意的：

- **避障事件暂不切模式。** `EVENT_OBSTACLE_*` 已定义，调度器也留了
  `CONTROL_IR_AVOID` 分支，但当前没有生产者——`ir_avoid` 仍是"传感器 + 阻塞动作"
  合一，且本来就在 `main.c` 里处于注释状态。阶段 4 拆出 `avoid_task` 后再接。
  在那之前 `CONTROL_IR_AVOID` 不会被进入。
- **初始模式沿用"上电即循迹"**（`SCHEDULER_START_MODE`），与改造前行为一致。
  要改成"等启动键"把它换成 `CONTROL_IDLE` 即可，key2 已投 `EVENT_KEY_START`。

一个已知的良性行为：任务运行中双击 RED 接管，退出后模式回到 `CONTROL_TASK`，
但该任务的动作已被 `Car_ActionAbort()` 打断。转弯任务会在下一次 `Step()` 看到
`Car_ActionStep()` 返回 `IDLE`，走失败出口结束任务并交回循迹。失败安全，不卡死。

### 阶段 3～4 落地情况

新增 `indicator`（指示层）、`avoid_task`（避障行为）、`ultrasonic_sense`
（超声波传感器）。`ir_avoid` 重写为纯传感器。

**避障按"重新设计"而非"改造"处理**，相比原实现三处实质变化：

1. **转向方向不再随机。** 原来双侧受阻时用伪随机选左右，同一处障碍反复试错、
   行为不可复现，调试时分不清是策略错还是运气差。现在按两侧红外原始值择路，
   往回波更弱（更空旷）的一侧转；读数接近时固定偏右，保证可复现。
2. **新增 `VERIFY` 相位。** 转完先等一轮新采样确认真的让开了，没让开就带着
   新的方向判断再试。原实现认为一次旋转必然有效，转完直接结束。
3. **新增重试上限与故障出口。** 连续 `AVOID_MAX_ATTEMPTS`（4）次未脱困就投
   `EVENT_FAULT`，调度器进急停；另有 `AVOID_TOTAL_TIMEOUT_MS` 总超时兜底。
   原实现可能原地无限打转。

相位：`BACKING`（仅前方/双侧受阻）→ `TURNING` → `VERIFY` → 结束。全部走
`Car_Start*` + `Car_ActionStep()`，避障期间主循环照常仲裁，急停随时生效。

其他要点：

- `IrAvoid_Sense()` 把发射管稳定延时拆成相位状态机
  （`IDLE` → `LEFT_SETTLE` → `RIGHT_SETTLE`），消掉两处 `HAL_Delay`。
  阻塞版留作 `IrAvoid_UpdateBlocking()`，仅供标定阈值。
- 传感器只在**状态变化时**发事件，避免每 20 ms 灌满队列。
- `EVENT_OBSTACLE_CLEAR` 不撤销避障：是否脱困由 `VERIFY` 判定，否则障碍刚
  离开视野动作就做半截。
- 避障结束调 `IrAvoid_Restart()` / `UltrasonicSense_Restart()`，丢掉动作前的
  残留判定。急停恢复时同样重新起判。
- `ultrasonic_avoid.c` 保留为独立调试入口但已被取代，文件头标注了它直写 RGB、
  绕过指示层、内部阻塞三个问题。

指示层优先级：`FAULT` > `AVOID` > `MANUAL` > `TASK`。手动接管用蓝灯常亮区分，
避障告警按受阻侧亮红灯快闪，急停红灯慢闪。蜂鸣不跟随闪烁相位——告警要持续响。

---

## 11. 编程规则

- 正式模块不得使用无法退出的 `while (1)`；无限循环只允许在独立调试入口。
- 正式主循环不得出现 `HAL_Delay`。
- 识别与执行分层：识别只发事件，行为模块管生命周期，`car` 只执行动作。
- 同一周期只有一个模块写电机，控制权由调度器授予。
- 共享外设（蜂鸣器、RGB）只能通过指示层申请。
- 编码器不做全局清零，动作内部保存起始快照。
- 每个动作必须有超时或失败出口，故障时制动停车。
- 参数用带单位的宏名（`_MS`、`_MM`、`_DEG10`），不在主循环里散落裸数。
- 不删除已有调试代码；调试入口与正式入口并存。

---

## 12. 已定与待定

已定：

- **1 号库**：维持"识别到新视觉任务才离开停止态"的现有意图，只修 `STOPPED`
  出不去的缺陷（§2.4）。
- **转弯 + 避障并发**：实际不会出现，恢复策略取最简（§6.3）。
- **循迹周期**：固定 10 ms，由循迹自己判周期，建议改名
  `LINE_CONTROL_PERIOD_MS`（§8.1）。
- **传感器冲突**：超声波（前方）优先于红外（左右）（§8 末）。
- **红外遥控**：调试用手动接管，优先级仅低于急停。**RED 键连按两次**切换
  接管/退出，其他键不介入（§8.2）。

待定：

1. **循迹增益与差速/原地转的切换阈值**：必须实车标定，见 §8.1。阶段 5 从低速
   起调。
2. **视觉任务集与避障行为逻辑**后续还要改。本架构不锁定具体动作参数，只锁定
   "识别发事件、任务管状态、`car` 执行"的分层，改动作时不应触及调度器。

