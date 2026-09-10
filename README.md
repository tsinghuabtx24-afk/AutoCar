# AutoCar

STM32F103ZET6 智能小车。四路电机 + 编码器、四路红外循迹、双路红外避障、超声波
测距、OLED、NEC 红外遥控、UART 视觉识别接入。

控制架构为**主循环调度 + 各模块独立状态机 + 非阻塞动作**。设计理由、分层约束和
改造过程见 [ARCHITECTURE.md](ARCHITECTURE.md)，本文只讲怎么用。

## 编译与烧写

```bash
cmake --preset Debug
cmake --build build/Debug
```

产物在 `build/Debug/`：`AutoCar.elf`（GDB / OpenOCD 用）、`AutoCar.hex`、
`AutoCar.bin`（DAPLink 拖拽用）。工具链是 STM32CubeCLT 自带的
`arm-none-eabi-gcc` + Ninja。

串口：
- **USART1（PA9, 115200）** — `printf` 诊断输出，所有 `[XXX]` 日志走这里。
- **USART2（PD5/PD6, 115200）** — 视觉模块输入，需 AFIO 重映射（代码里已开）。

## 主循环

```c
while (1)
{
  ManualTask_Sense();        /* 1. 采样层：只产生事件，不驱动底盘 */
  // IrAvoid_Sense();        /*    当前注释：阈值与赛场实况不符 */
  // UltrasonicSense_Sense();

  Scheduler_DrainEvents();   /* 2. 调度层：消费事件，决定控制模式 */
  Scheduler_Dispatch();      /* 3. 分派：本轮只有一个模块写电机 */
  Indicator_Step();          /* 4. 指示层：按优先级裁决蜂鸣与 RGB */
  /* 5. 诊断：按 tick 判周期 */
}
```

循环里**没有 `HAL_Delay`**，每轮几十微秒，所以急停和抢占都能及时响应。

## 控制模式

优先级从高到低：

| 模式 | 触发 | 行为 |
| --- | --- | --- |
| `CONTROL_STOP` | key3、`EVENT_FAULT` | 持续制动，只有 key2 能退出 |
| `CONTROL_MANUAL` | 遥控 RED 键**双击** | 遥控手动接管，避障不再保护车体 |
| `CONTROL_IR_AVOID` | 红外/超声波报障 | 避障状态机接管。**当前进不去**：两个采样调用在 `main.c` 里注释着，没有生产者 |
| `CONTROL_TASK` | 视觉识别到任务 | 转弯/鸣笛/入库 |
| `CONTROL_LINE_TRACK` | 默认 | 差速循迹 |
| `CONTROL_IDLE` | — | 停车待命，key2 启动 |

模式切换会在串口打 `[SCHED] A -> B`。

`SCHEDULER_START_MODE`（`scheduler.h`）决定上电初始模式，当前是
`CONTROL_LINE_TRACK`，即**上电立刻开始跑**。想上电先停着就改成 `CONTROL_IDLE`。

## 遥控用法

只认 **RED 键（command `0x00`）连按两次**切换接管/退出。其他遥控键在非手动模式
下一律忽略，不会干扰自动驾驶。

手动模式下：`UP` 前进、`BACK` 后退、`LEFT`/`RIGHT` 原地转、`HORN` 鸣笛。
松手（按键静默 300 ms）自动停车，但**仍留在手动模式**；只有再双击 RED 才交还
控制权。这样遥控丢帧只会让车停下，不会莫名把控制权还给自动驾驶。

## 视觉协议

USART2 收 3 字节帧：`0xAA` / `id` / `0xAA ^ id`。

| id | 含义 | 处理 |
| --- | --- | --- |
| 0 | 限速 | 只改 `task_speed`，不抢占底盘。⚠️ **当前没有实际效果**，见已知问题 |
| 1 | 解除限速 | 恢复默认速度 |
| 2 / 3 | 左转 / 右转 | 原地旋转 90° |
| 4 | 鸣笛 | 停车鸣两声 |
| 5 | 1 号库 | 闪灯后停住，直到识别到新任务 |
| 6 | 2 号库 | 闪灯后满速前进 2 s |
| 7 | 第 8 类 | 停车鸣两声，与 id 4 同一动作；`[DIAG]` 里显示 `HORN_2` 以区分 |

## 模块分工

分层的硬约束：**识别模块只发事件，任务模块管生命周期，`car` 只执行动作。**
下层不认识上层，同一周期只有一个模块写电机。

### 底层

| 模块 | 职责 |
| --- | --- |
| `tim` | 8 路电机 PWM |
| `encoder` | 四路正交解码，里程 / 转速 / 转角估算，由 SysTick 采样 |
| `car` | 底盘动作：非阻塞动作状态机 + 瞬时接口。不判断何时执行 |

### 传感器（只发事件）

| 模块 | 职责 |
| --- | --- |
| `line_tracker` | 四路循迹，差速连续修正 + 大偏差原地转 |
| `ir_avoid` | 双路红外，非阻塞采样，状态变化时发 `EVENT_OBSTACLE_LEFT/RIGHT/BOTH` |
| `ultrasonic_sense` | 超声波前方测距，发 `EVENT_OBSTACLE_FRONT` |
| `ir_remote` | NEC 解码（EXTI 中断） |
| `vision_uart` | 视觉帧解析（UART2 中断），投递视觉事件 |

### 行为（拿到控制权才动）

| 模块 | 职责 |
| --- | --- |
| `scheduler` | 模式仲裁与分派。不读传感器，不写电机 |
| `vision_task` | 视觉任务状态机：转弯 / 鸣笛 / 入库 |
| `avoid_task` | 避障状态机：后退 → 转向 → 确认脱困，带重试上限 |
| `manual_task` | 遥控手动接管 |

### 基础设施

| 模块 | 职责 |
| --- | --- |
| `event` | 环形事件队列（深度 16），`Event_Post()` 中断安全 |
| `indicator` | 蜂鸣器 + RGB 唯一属主，按优先级裁决 |
| `retarget` | `printf` → USART1 |
| `oled` | I2C 显示（阻塞发送，运动期间不要刷） |

## 关键接口

### 非阻塞长动作

```c
Car_StartForwardDistance(70U, 200U);        /* 发起一次 */

switch (Car_ActionStep())                   /* 每轮推进一次 */
{
  case CAR_ACTION_BUSY:    break;                     /* 还在走 */
  case CAR_ACTION_DONE:    task_phase++;        break; /* 完成 */
  case CAR_ACTION_TIMEOUT:
  case CAR_ACTION_FAILED:  task_state = FAILED; break; /* 失败，已制动 */
}
```

角度动作用**带符号角度**：正 = 左转，负 = 右转。
`Car_StartRotateAngle()` / `Car_StartTurnAngle()`。

需要抢占时先 `Car_ActionAbort()`——`Start` 系列遇到已有动作在跑会返回 `FAILED`
且不打断它。

### 瞬时接口

`Car_ForwardRun(speed)`、`Car_Stop()`、`Car_DriveSpeed(left, right)`。
设完 PWM 立即返回，循迹和遥控用这个。`Car_DriveSpeed` 的入参是占空比刻度
−100~100，内部处理死区换算。

### 阻塞接口（仅标定用）

`Car_ForwardDistance()`、`Car_RotateLeftAngle()`、`LineTracker_Run()`、
`IrAvoid_UpdateBlocking()` 等保留作独立标定入口，内部与非阻塞路径共享同一份
逻辑，标定值不会两处不同步。**正式主循环不要调用**，否则调度器在动作期间无法
仲裁。

## 待标定参数

全部是起点值，不是标定结果。

### 循迹（`line_tracker.h`）— 优先级最高

| 参数 | 当前 | 说明 |
| --- | --- | --- |
| `LINE_BASE_SPEED` | 70 | 基础速度。**最后**才调这个 |
| `LINE_DIFF_STEP` | 6 | 一档修正的左右轮速度差。**先调这个** |
| `LINE_PIVOT_SPEED` | 70 | 大偏差原地转速度 |
| `LINE_LOST_COUNT` | 20 | 连续多少周期全白判丢线 |

标定顺序：**直线不画蛇 → 缓弯不切内道 → 再提速**。不要一上来就提速。

⚠️ 死区约束：`LINE_BASE_SPEED - 2 × LINE_DIFF_STEP > CAR_SPEED_BASE(50)`。
违反会让内侧轮掉进死区直接停转，变成急转而非修正。已有 `_Static_assert`
编译期拦截，改参数时会直接编译失败。

### 编码器（`encoder.h`）

`ENC_COUNTS_PER_REV`（521）、`ENC_CAL_NUM/DEN`（5/7 里程补偿）、
`ENC_POLARITY_M1..M4`（前进时四路计数应全为正）。

### 底盘（`car.h`）

`CAR_WHEEL_TRACK_MM`（128）、`CAR_SPEED_BASE`（死区上界，等于
`MOTOR_SPEED_MIN`）、`CAR_ROT_COMP_70/80/100_X1000`（原地旋转滑移补偿，
指令 90° 实测只转 40~45°）。

循迹改差速后**不再依赖滑移补偿表**，它只影响视觉转弯和避障转向。

### 避障（`avoid_task.h` / `ir_avoid.h` / `ultrasonic_sense.h`）

`AVOID_REVERSE_DIST_MM`（80）、`AVOID_ROTATE_DEG10`（450）、
`AVOID_NUDGE_DEG10`（300）、`AVOID_MAX_REVERSES`（3）、
`IR_AVOID_LEFT/RIGHT_THRESHOLD`（1500，实测 ADC 约 60~2000）、
`ULTRASONIC_SENSE_THRESHOLD_MM`（200）。

避障**不设重试上限**，持续尝试直到脱困——障碍是暂时的，避不开不算故障。但后退
次数有上限（`AVOID_MAX_REVERSES`），否则正对墙时反复倒退会累计出很远的位移。
`AVOID_TOTAL_TIMEOUT_MS`（15 s）到期只是结束本轮并交还控制权，不报故障；障碍
仍在的话传感器下一轮会重新触发。

改红外采样周期时记得同步 `AVOID_VERIFY_MS`（120），它要覆盖至少一轮完整采样。

### 超声波（`ultrasonic.h`）

`ULTRASONIC_TIMEOUT_MS`（35，单次测距总超时）、`ULTRASONIC_MIN_GAP_MS`（60，
HC-SR04 手册要求的最小间隔）、`ULTRASONIC_MAX_MM`（4000，超量程一律判无效）。

`ULTRASONIC_SENSE_PERIOD_MS` 必须 ≥ `ULTRASONIC_MIN_GAP_MS`，已有
`_Static_assert` 拦截。

## 诊断串口

`[DIAG]` 每秒一行，阶段 6/7 之后多了两个计数器：

- `resync=` — 视觉帧因丢字节而重新对齐的次数。持续增长说明视觉模块在丢字节
  或波特率有偏差。
- `REMOTE dropped=` — 遥控帧队列满而丢弃的帧数，正常应为 0。

## 已知问题

| 问题 | 影响 | 计划 |
| --- | --- | --- |
| 视觉 id 0 的限速没有实际效果 | `task_speed` 只被 `[DIAG]` 打印，循迹用的是 `LINE_BASE_SPEED`，两者没有连接 | 待定，见下 |
| 超声波 ECHO 与 IR_IN 共用 EXTI15_10 向量 | 遥控解码和测距的中断会互相排队，抖动几百 ns | 不影响毫米级精度，不处理 |
| 左侧 RGB 的 R/G 引脚与命名相反 | 申请红色实际亮绿色，右侧正常 | 硬件接线问题，`indicator.c` 已注明 |
| `ultrasonic_avoid.c` 已被 `avoid_task` 取代 | 保留作调试入口，但直写 RGB 绕过指示层、内部阻塞 | 文件头已标注 |
| `oled` 阻塞 I2C | 运动期间不能刷屏 | 只在 IDLE / STOP 或调试时更新 |

限速那条要接起来的话，得让 `line_tracker` 的基础速度从 `VisionTask_GetSpeed()`
取，而不是直接用宏。但循迹的差速档位和死区约束都是按固定 `LINE_BASE_SPEED`
标定的（`_Static_assert` 也是编译期检查这个宏），改成运行时可变需要把死区
约束改成运行时钳位。**循迹参数还没实车标定完，先不动这里**。

## 编程规则

- 正式模块不得使用无法退出的 `while (1)`；无限循环只允许在独立调试入口。
- 正式主循环不得出现 `HAL_Delay`。
- 识别与执行分层：识别只发事件，行为模块管生命周期，`car` 只执行动作。
- 同一周期只有一个模块写电机，控制权由调度器授予。
- 共享外设（蜂鸣器、RGB）只能通过指示层申请，不直写 GPIO。
- 编码器不做全局清零，动作内部保存起始快照。
- 中断里不 `printf`，只投事件或累加计数。
- 每个动作必须有超时或失败出口，故障时制动停车。
- 参数用带单位的宏名（`_MS`、`_MM`、`_DEG10`），不在主循环里散落裸数。
- 不删除已有调试代码；调试入口与正式入口并存。

## 改造进度

| 阶段 | 内容 | 状态 |
| --- | --- | --- |
| 1 | `car` 非阻塞动作状态机 | ✅ |
| 2 | 事件队列 + 调度器 + 模式机 + 手动接管 | ✅ |
| 3 | 指示层 | ✅ |
| 4 | 拆 `ir_avoid` 为传感器 + `avoid_task` | ✅ |
| 5 | 循迹改差速连续修正 | ✅ 待实车标定 |
| 6 | 超声波改中断式测距（非输入捕获，见下） | ✅ 待实车验证 |
| 7 | 视觉帧同步重整、遥控帧队列、1 号库缺陷 | ✅ 待实车验证 |

阶段 6 **没有用定时器输入捕获**：ECHO 接在 PF12，F103 没有任何定时器通道映射
到 GPIOF，输入捕获必须先改硬件接线。改用 **EXTI 双边沿 + DWT 时间戳**达到同样
目的——中断里只打时间戳，主循环 `Ultrasonic_Step()` 查状态并换算。忙等只剩
TRIG 那 10 µs 触发脉冲，是 HC-SR04 的时序要求。


## AI守则 
 在实机调试时，实际情况远比AI或人的思考重要。所以AI不可直接下定结论，更不可直接修改代码和文档；而是必须要询问实际情况，多给出几种可能，等到被同意后才可以修改代码。