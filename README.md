# STM32 矩形循迹小车

基于 STM32F103C8T6 的矩形循迹小车工程，当前重点是保持矩形循迹、TB6612FNG 电机控制、8 路循迹输入和 USART2 调试输出稳定。工程已经整理为 `User / App / Components / BSP` 分层结构，后续再扩展速度闭环、OpenMV/OpenCV 视觉和云台打靶。

## 当前状态

- 目标：低速稳定跑完整个矩形赛道，先不追求速度。
- 循迹：8 路循迹传感器 + 位置式 PID 差速。
- 直角弯：默认固定左转，优先使用右编码器累计值退出直角。
- 电机：TB6612FNG，TIM1 输出左右电机 PWM。
- 编码器：右编码器 `RE` 当前有效；左编码器 `LE` 仍不稳定，暂不用于速度闭环。
- 蓝牙控制：USART2 保持 9600 8N1；上电默认停车，收到 `START`/`1` 后启动，`STOP`/`0` 立即停车。
- 视觉/云台：代码预留，默认关闭。

关键配置以 [`App/app_config.h`](App/app_config.h) 为准：

```c
#define RECT_DEFAULT_CORNER_DIR        (-1)
#define RECT_ENABLE_CROSS_CORNER       1
#define LINE_CORNER_USE_ENCODER        1
#define LINE_CORNER_ENCODER_TARGET     550
#define LINE_CORNER_CENTER_ENABLE_ENCODER 500
#define LINE_CORNER_DEBOUNCE_COUNT     6U
#define LINE_CORNER_EXIT_CONFIRM_MS    30U
#define LINE_BASE_PWM_FAST             260
#define LINE_BASE_PWM_MID              240
#define LINE_BASE_PWM_SLOW             220
#define APP_ENABLE_VISION_TARGET       0
#define APP_ENABLE_GIMBAL_SERVO        0
#define APP_ENABLE_BLUETOOTH_CONTROL   1
```

## 工程结构

```text
User/
  main.c                      启动层，只负责 AppRobot_Init / AppRobot_Task

App/
  app_robot.c                 初始化、任务调度、模式切换
  app_line_follow.c           矩形循迹状态机
  app_vision_target.c         视觉/云台预留
  app_config.h                当前跨层配置入口

Components/
  chassis.c                   差速底盘抽象和安全限幅
  tb6612_motor.c              TB6612FNG 方向和 PWM 适配
  yahboom_tracker8_io.c       8 路循迹输入和误差计算
  pid.c                       通用 PID
  vision_protocol.c           视觉串口协议预留
  gimbal_servo.c              云台舵机预留

BSP/
  bsp_gpio.c                  公共 GPIO/LED
  bsp_pwm.c                   TIM1 PA8/PA9 电机 PWM
  bsp_encoder.c               TIM2/TIM4 编码器
  bsp_uart.c                  USART2 调试输出
  bsp_systick.c               1 ms 系统节拍
  bsp_servo.c                 舵机 PWM 预留
```

## 硬件配置

| 模块 | 当前配置 |
|---|---|
| MCU | STM32F103C8T6 |
| 电机驱动 | TB6612FNG |
| 电机 PWM | TIM1_CH1 PA8 右电机，TIM1_CH2 PA9 左电机 |
| 编码器 | TIM2 PA0/PA1 左编码器，TIM4 PB6/PB7 右编码器 |
| 循迹传感器 | 8 路数字输入，X1 到 X8 |
| 调试串口 | USART2 PA2/PA3，9600 8N1，TXE 中断队列发送 |
| 系统节拍 | SysTick 1 ms |

完整引脚表见 [`pinmap.md`](pinmap.md)。

## 构建与导入

### Keil MDK

1. 打开或导入 `project.uvprojx`。
2. 确认芯片为 STM32F103C8T6。
3. Include 路径需要覆盖 `User`、`App`、`BSP`、`Components`、`Start`、`Library`。
4. Define 至少包含 `USE_STDPERIPH_DRIVER`、`STM32F10X_MD`。
5. 启动文件使用 `startup_stm32f10x_md.s`。

### EIDE / VSCode

1. 使用 STM32F103C8T6 标准外设库工程。
2. 加入 `User/App/BSP/Components/Start/Library` 相关源码和头文件路径。
3. 若使用 ARM GCC，确认链接脚本为 STM32F103C8T6 的 64 KB Flash / 20 KB RAM 配置。

更详细说明见 [`Project/README_Keil_EIDE.md`](Project/README_Keil_EIDE.md)。

## 蓝牙启动与停止

蓝牙模块连接 USART2 PA2/PA3，波特率保持 9600。上电后小车处于
`ROBOT_MODE_STOP`，串口发送：

| 命令 | 行结束要求 | 动作 | 返回 |
|---|---|---|---|
| `1` | 不需要 | 复位循迹状态机，等待 200ms 后开始循迹 | `ACK START` |
| `0` | 不需要 | 立即把底盘PWM停止 | `ACK STOP` |
| `START` 或 `GO` | 需要 `\r` 或 `\n` | 与 `1` 相同 | `ACK START` |
| `STOP` | 需要 `\r` 或 `\n` | 与 `0` 相同 | `ACK STOP` |

手机蓝牙串口按钮建议直接配置为发送单字符 `1` 和 `0`。普通UART无法知道蓝牙
无线链路是否已经断开，所以当前“断开蓝牙自动停车”尚未实现；需要模块 STATE
引脚或周期心跳协议才能可靠判断。

## 串口调试

当前 USART2 遥测示例：

```text
M=0 S=1 RAW=0x18 ERR=0 LPWM=260 RPWM=260 LE=0 RE=90 C=1 DIR=-1 SUM=610 DT=10 OV=0 F=0 TD=0
```

常用字段：

| 字段 | 含义 |
|---|---|
| `M` | 当前模式，`0` 为循迹 |
| `S` | 循迹状态机编号 |
| `RAW` | 8 路循迹位，1 表示检测到黑线 |
| `ERR` | 位置误差，左负右正 |
| `LPWM/RPWM` | 左右轮 PWM 命令 |
| `LE/RE` | 左右编码器在最近遥测周期内累计的增量 |
| `C` | 已完成直角弯次数 |
| `DIR` | 当前直角方向，`-1` 左转，`1` 右转 |
| `SUM` | 当前直角弯累计编码器计数 |
| `DT` | 最近遥测窗口内最大的控制调度间隔，正常应接近 10 ms |
| `OV` | 最近遥测窗口内 `DT >= 20 ms` 的次数 |
| `F` | 传感器健康故障：0正常，1长期全未触发，2长期全触发，3驱动无效 |
| `TD` | USART2 TX 队列累计丢弃字符数，正常应保持 0 |

常见正常直线数据：

```text
RAW=0x18 ERR=0
RAW=0x1C ERR=-133
RAW=0x38 ERR=133
```

## 调参建议

赛前优先只调整这些参数：

```c
LINE_CORNER_ENCODER_TARGET
LINE_CORNER_CENTER_ENABLE_ENCODER
LINE_RECOVER_MS
LINE_CORNER_DEBOUNCE_COUNT
LINE_BASE_PWM_FAST
LINE_BASE_PWM_MID
LINE_BASE_PWM_SLOW
```

不建议同时修改 PID、电机方向、PWM 通道、传感器权重和硬件映射。当前调参目标是低速稳定跑完矩形 2 到 3 圈。

## 已知问题

1. 出弯后仍可能恢复不稳，容易进入 `BLIND -> LOST`。
2. 左编码器 `LE` 暂不可靠，不能启用左右轮速度闭环。
3. 当前是开环 PWM，左右电机实际速度可能不一致。
4. 若 `S=3` 时 `SUM` 经常超过 1000 才退出，需要优先确认是否烧录了最新固件，以及编码器退出条件是否生效。
5. 视觉和云台功能仍是预留状态，默认不参与主循环控制。
6. 当前已增加直角出口连续确认和传感器极端状态诊断，但仍需用同一固件完成连续 3 圈实车验收。

## 文档

- [`docs/PLAN.md`](docs/PLAN.md)：当前项目计划与赛前收敛策略。
- [`docs/RECTANGLE_TRACK.md`](docs/RECTANGLE_TRACK.md)：矩形循迹状态机、串口字段和失败链路。
- [`docs/TUNING.md`](docs/TUNING.md)：调参记录与建议。
- [`docs/VALIDATION.md`](docs/VALIDATION.md)：三圈实车、遥测和电气冻结验收清单。
- [`pinmap.md`](pinmap.md)：当前硬件引脚映射。
- [`Project/README_Keil_EIDE.md`](Project/README_Keil_EIDE.md)：Keil / EIDE 导入说明。
