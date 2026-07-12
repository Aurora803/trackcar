# STM32 校赛矩形循迹小车

本项目是基于 STM32F103C8T6、TB6612FNG 和 8 路数字循迹传感器的矩形赛道循迹底盘。`school-competition-line-v1` 为完成连续 5 圈实车测试的校赛稳定版本；循迹模块现已冻结，后续工作转入视觉识别、二维云台独立验收和最小接口联调。

## 校赛版本当前状态

- `LINE_CORNER_ENCODER_TARGET=495` 的同一套参数已现场连续完成 5 圈矩形赛道运行，全程未修改参数。
- 经现场连续 5 圈实车耐久测试验证，车辆能够完成矩形循迹运行；现有串口文件保存了部分尾部状态记录，不能作为完整 20 个角点的日志证明。
- 角点后常见状态链为 `FOLLOW -> CORNER -> BLIND -> RECOVER -> FOLLOW`，并非所有角点都直接从 `CORNER` 回到 `FOLLOW`。
- 角点后容错恢复机制能够有效重新捕获中心线，当前版本满足校赛展示稳定性要求，但仍存在恢复路径偏长的优化空间。
- 当前不再修改循迹核心参数、状态机、引脚和底盘接口。

## 校赛提交状态

| 模块 | 当前状态 |
|---|---|
| STM32循迹底盘 | 已完成，稳定版本已冻结 |
| 连续矩形测试 | 已完成连续5圈实车验证 |
| 蓝牙启停 | 保留用于调试和展示 |
| 视觉识别 | 由视觉成员继续完成 |
| 二维云台 | 由视觉/硬件成员继续完成 |
| 底盘与视觉通信 | 待最小接口联调 |
| README | 本轮更新 |
| 设计说明文档 | 待统一整理 |
| 展示视频 | 待正式拍摄和剪辑 |

## 主要功能

- 8 路数字传感器位置误差计算与纯 P 差速循迹；
- 矩形赛道直角识别、编码器转角累计、丢线搜索和容错恢复；
- TB6612FNG 双电机开环 PWM 控制及底盘安全限幅；
- USART2 蓝牙 `START/STOP` 或 `1/0` 启停；
- 非阻塞串口遥测和状态迁移事件日志；
- 视觉协议与二维云台控制代码预留，当前配置未启用。

## 硬件组成

| 模块 | 源码确认的当前配置 |
|---|---|
| MCU | STM32F103C8T6 |
| 电机驱动 | TB6612FNG，STBY 直接接 3.3V |
| 电机 PWM | TIM1_CH1 PA8 右电机，TIM1_CH2 PA9 左电机，1 kHz |
| 编码器 | TIM2 PA0/PA1 左编码器，TIM4 PB6/PB7 右编码器 |
| 循迹传感器 | 8 路数字输入：X1~X8 对应 PB11、PB10、PB1、PB0、PA7、PA6、PA5、PA4 |
| 调试/蓝牙串口 | USART2 PA2/PA3，9600 8N1 |
| 系统节拍 | SysTick 1 ms；控制周期 10 ms |

循迹模块与编码器的实际供电电压、输出电平类型仍需按实物规格人工确认，详见 [`docs/PINMAP.md`](docs/PINMAP.md)。

## 软件架构

```text
User/main.c
  -> App/app_robot.c          初始化、调度、模式和蓝牙命令
  -> App/app_line_follow.c    矩形循迹状态机
  -> App/app_vision_target.c  视觉/云台预留，当前关闭

Components/
  chassis.c / tb6612_motor.c  底盘抽象与电机适配
  yahboom_tracker8_io.c       8 路采样和误差计算
  pid.c                       通用 PID（当前循迹为纯 P）
  vision_protocol.c           视觉协议预留
  gimbal_servo.c              云台舵机预留

BSP/
  bsp_gpio.c / bsp_pwm.c / bsp_encoder.c / bsp_uart.c
  bsp_systick.c / bsp_servo.c
```

`App/app_config.h` 是当前跨层配置入口；校赛冻结阶段不再拆分或调整。

## 循迹状态机

| 状态 | 编号 | 作用 |
|---|---:|---|
| `START` | 0 | 启动后的 200 ms 稳定等待 |
| `FOLLOW` | 1 | 正常循迹 |
| `BLIND` | 2 | 丢线后按最近方向搜索 |
| `CORNER` | 3 | 矩形直角转向 |
| `RECOVER` | 4 | 重新捕线后的低速稳定恢复 |
| `LOST` | 5 | 搜索超时或传感器故障后停车 |

实车常见角点链路：

```text
FOLLOW --TR=2--> CORNER --TR=7--> BLIND --TR=8--> RECOVER --TR=10--> FOLLOW
                                                   |
                                                   +--TR=11--> BLIND（可再次捕线）
```

`CORNER` 也可因编码器、中心确认或超时进入其他后续状态，详见 [`docs/RECTANGLE_TRACK.md`](docs/RECTANGLE_TRACK.md)。当前效果不应描述为“所有角点状态链完全理想”。

## 当前稳定参数

以下值来自当前源码 `App/app_config.h`：

```c
#define LINE_CORNER_ENCODER_TARGET      495
#define LINE_RECOVER_CORRECTION_LIMIT   40
#define LINE_RECOVER_PWM                200
#define LINE_RECOVER_CENTER_CONFIRM_MS  250U
```

编码器角点退出条件为：

```c
g_corner_encoder_sum >= LINE_CORNER_ENCODER_TARGET
```

其他关键开关为固定左转、开启横线直角识别、启用编码器角点退出、蓝牙控制开启，视觉和云台关闭。历史 `450/550/600` 仅见于调参记录，均已停用，不代表校赛稳定配置。

## 工程目录

```text
App/          应用、状态机及集中配置
BSP/          GPIO、PWM、编码器、UART、SysTick、舵机底层
Components/   底盘、电机、传感器、PID、视觉协议组件
User/         main 与中断入口
Start/        CMSIS、启动和系统文件
Library/      STM32F10x 标准外设库
Project/      Keil/EIDE 导入说明
docs/         接线、循迹、调参、验收和测试文档
```

## 编译与烧录

仓库已有 `project.uvprojx`，可由 Keil MDK 打开并构建。EIDE/ARM GCC 工程使用 `Start/startup_stm32f10x_md_gcc.c`、`Start/syscalls_gcc.c` 和 `STM32F103C8_FLASH.ld`；完整源文件、Include 和 Define 要求见 [`Project/README_Keil_EIDE.md`](Project/README_Keil_EIDE.md)。

仓库文档未确认实际使用的下载器、调试接口和烧录软件，因此校赛材料中保留 TODO：由现场负责人补充实际烧录工具、连接方式和操作步骤，不凭经验指定 ST-Link、串口 ISP 或其他方案。

## 启动和停止

蓝牙控制启用时，上电默认处于 `STOP`。USART2（9600 8N1）支持：

| 命令 | 行结束要求 | 动作 | 返回 |
|---|---|---|---|
| `1` | 无 | 复位循迹状态机，等待 200 ms 后循迹 | `ACK START` |
| `0` | 无 | 立即停止底盘输出 | `ACK STOP` |
| `START` / `GO` | 需要 CR 或 LF | 同 `1` | `ACK START` |
| `STOP` | 需要 CR 或 LF | 同 `0` | `ACK STOP` |

普通 UART 无法判断蓝牙无线链路是否断开，当前没有“蓝牙断连自动停车”。

## 调试日志字段

周期遥测格式包含：

```text
M S RAW ERR LPWM RPWM LE RE C DIR SUM DT OV F TD ARM RM RC ST RCM RLM TR
```

| 字段 | 含义 |
|---|---|
| `M/S` | 机器人模式 / 循迹状态编号 |
| `RAW/ERR` | 8 路有效位 / 位置误差（左负右正） |
| `LPWM/RPWM` | 左右轮 PWM 命令 |
| `LE/RE` | 最近遥测周期内左右编码器累计增量 |
| `C/DIR/SUM` | 角点计数、方向、当前角点编码器累计值 |
| `DT/OV` | 最大控制调度间隔 / 超时次数 |
| `F/TD` | 传感器故障码 / UART TX 队列累计丢字符数 |
| `ARM/RM/RC/ST` | 角点武装、重装计时、中心计时和状态驻留时间 |
| `RCM/RLM` | RECOVER 中心稳定 / 丢线确认计时 |
| `TR` | 最近一次状态迁移原因 |

状态变化还会输出 `EV FROM=... TO=... TR=...`。现有文件只保存了耐久测试的部分尾部日志，不能据此声称完整记录了 20 个角点。

## 实车测试结果

- 使用稳定参数 `495 / 40 / 200 / 250U` 完成连续 5 圈矩形赛道实车测试；
- 测试全程未修改参数，现场观察确认车辆完成 5 圈；
- 角点主要依靠 `BLIND` 和 `RECOVER` 容错重新捕线；
- 部分 `RECOVER` 会以 `TR=11` 短暂返回 `BLIND`，之后仍可恢复；
- 结论：当前版本适合冻结用于校赛展示。详细记录见 [`docs/SCHOOL_COMPETITION_TEST_REPORT.md`](docs/SCHOOL_COMPETITION_TEST_REPORT.md)。

## 已知问题

1. 角点恢复路径有时较长，并非每个角点都按最短理想链路完成。
2. `RECOVER` 偶尔以 `TR=11` 再次进入 `BLIND`，虽可恢复，仍有优化空间。
3. 左编码器历史观测不稳定，当前不用于左右轮速度闭环；底盘仍为开环 PWM。
4. 蓝牙断连自动停车未实现。
5. 视觉、二维云台和底盘通信尚未完成联调。
6. 耐久测试完整原始串口日志未保留，仅有部分尾部状态记录。

## 后续视觉云台联调接口

源码已预留 `$T,dx,dy,valid\n` 形式的 ASCII 视觉协议和云台组件，但 `APP_ENABLE_VISION_TARGET=0`、`APP_ENABLE_GIMBAL_SERVO=0`，且当前没有分配独立视觉串口。下一阶段应先独立验收视觉识别和二维云台，再规划不与 USART2 调试/蓝牙及电机 PWM 冲突的通信接口，最后做最小联调；不得把预留代码视为已完成联调。

## 三人分工

仓库没有记录成员姓名，提交材料先按角色分工，姓名由团队补充：

| 角色 | 分工 | 当前任务 |
|---|---|---|
| 成员 A（底盘） | STM32 循迹底盘、状态机、实车测试与冻结维护 | 保持 `school-competition-line-v1`，不再修改循迹核心参数 |
| 成员 B（视觉） | 视觉识别与输出协议 | 完成独立识别验收，输出最小目标数据 |
| 成员 C（硬件/云台） | 二维云台、电气连接、展示集成 | 完成云台独立验收并配合最小接口联调 |

## 文档索引

- [`docs/PLAN.md`](docs/PLAN.md)：校赛阶段计划与冻结范围；
- [`docs/RECTANGLE_TRACK.md`](docs/RECTANGLE_TRACK.md)：矩形循迹状态机和迁移原因；
- [`docs/TUNING.md`](docs/TUNING.md)：当前基线与历史调参记录；
- [`docs/SCHOOL_COMPETITION_TEST_REPORT.md`](docs/SCHOOL_COMPETITION_TEST_REPORT.md)：连续 5 圈实车报告；
- [`docs/VALIDATION.md`](docs/VALIDATION.md)：验收清单与证据边界；
- [`docs/PINMAP.md`](docs/PINMAP.md)：当前接线与接口冲突说明。
