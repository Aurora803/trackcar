# 项目计划与当前状态

## 1. 当前目标

当前目标不是继续大重构，而是在校赛前把系统收敛到两个可用能力：

1. 矩形循迹小车低速稳定跑完赛道；
2. 在循迹基本稳定后，快速接入最小可用的云台打靶。

本阶段优先级：

1. 保持 `User / App / Components / BSP` 分层结构不再大改；
2. 不改已经确认的引脚、PWM、USART、TB6612 方向配置；
3. 循迹先低速稳定，不追求速度；
4. 编码器速度闭环暂缓，只允许在直角弯退出上使用已验证的右编码器；
5. 云台打靶采用最小串口协议和简单 P 控制，不做复杂架构。

## 2. 当前架构

```text
User/main.c
  -> App/app_robot.c          初始化、任务调度、模式切换
  -> App/app_line_follow.c    矩形循迹状态机
  -> App/app_vision_target.c  视觉/云台预留，默认关闭

Components/
  chassis.c                   差速底盘抽象和安全限幅
  tb6612_motor.c              TB6612FNG 方向和 PWM 适配
  yahboom_tracker8_io.c       8 路循迹输入和误差计算
  pid.c                       通用 PID
  vision_protocol.c           视觉串口协议预留
  gimbal_servo.c              云台舵机预留，默认关闭

BSP/
  bsp_gpio.c                  公共 GPIO/LED
  bsp_pwm.c                   TIM1 PA8/PA9 电机 PWM
  bsp_encoder.c               TIM2/TIM4 编码器
  bsp_uart.c                  USART2 调试输出
  bsp_systick.c               1ms 系统节拍
  bsp_servo.c                 舵机 PWM 预留
```

`App/app_config.h` 当前承担跨层配置角色。BSP 和 Components include 它只是读取宏配置，不代表 BSP 反向调用 App 业务逻辑。赛前不再拆分配置文件。

## 3. 当前循迹状态

截至当前测试：

- 直线读取基本正常，常见稳定数据为 `RAW=0x18 / 0x1C / 0x38`；
- 直角方向已改为固定左转：`RECT_DEFAULT_CORNER_DIR = (-1)`；
- 横线/宽黑线直角识别当前开启：`RECT_ENABLE_CROSS_CORNER = 1`；
- 当前使用低速开环 PWM 循迹，速度闭环未启用；
- 右编码器 `RE` 已能看到明显有效计数；
- 左编码器 `LE` 基本仍是 `-1/0/1` 抖动，暂不可用于闭环；
- 当前源码配置 `LINE_CORNER_USE_ENCODER = 1`，左转直角会优先用右编码器累计退出；
- 最新串口日志中，部分直角 `SUM` 仍超过目标很多后才退出。如果烧录后仍这样，优先确认是否烧录了最新固件，其次检查编码器退出逻辑是否实际生效。

当前主要风险：

1. 出弯后恢复不稳，可能在 `RECOVER` 后很快再次进入 `CORNER`；
2. 第三次以后更容易 `RAW=0x00`，进入 `BLIND` 后找不回线，最终 `LOST`；
3. 左右电机开环速度不一致，直线仍可能慢慢偏；
4. 左编码器未通，不能做左右轮速度闭环；
5. 云台打靶尚未开始，需要压缩实现范围。

## 4. 赛前实施顺序

### A. 循迹收敛

目标：低速跑完矩形 3 圈，不追求速度。

只允许优先调这些参数：

```c
LINE_CORNER_ENCODER_TARGET
LINE_CORNER_CENTER_ENABLE_ENCODER
LINE_RECOVER_MS
LINE_CORNER_DEBOUNCE_COUNT
LINE_BASE_PWM_FAST / MID / SLOW
```

不要同时大范围改 PID、传感器权重、电机方向和硬件映射。

### B. 接线可靠性

目标：电池单独供电稳定，跑动时不复位、不掉线。

必须确认：

- STM32 电源灯在电池供电下稳定；
- LM2596 输出稳定 5V；
- 电池负极、LM2596 GND、STM32 GND、TB6612 GND、循迹模块 GND 共地；
- 供电和电机大电流线不要依赖面包板；
- 循迹模块 VCC/GND/8 路信号线接触可靠。

### C. 云台打靶最小版本

目标：先能打中固定或低速目标，不做复杂识别链路。

建议协议：

```text
T,dx,dy\n
```

含义：

- `dx`：目标中心相对画面中心的水平偏差；
- `dy`：目标中心相对画面中心的垂直偏差；
- STM32 只做简单 P 控制，先不做完整 PID。

默认不要启用底盘多模式复杂切换。先保证循迹不被云台任务阻塞。

## 5. 暂不做的事

赛前暂不做：

- 大规模重构；
- 左右轮速度闭环；
- 复杂里程计；
- 多模式 UI；
- 高速循迹；
- 云台复杂 PID；
- 同时改多个硬件接口。

这些内容赛后再恢复推进。
