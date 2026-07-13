# 项目计划与当前状态

## 1. 当前目标

当前目标不是继续大重构，而是在校赛前把两套独立控制器分别收敛到可用状态：

1. STM32 通过 8 路循迹、编码器和 TB6612FNG 低速稳定跑完矩形赛道，并用 HC-05 完成启停和遥测；
2. OpenMV 独立识别红色色块，并直接控制一只 MG996R 完成水平方向跟踪。

两套控制器之间没有 UART 或其他数据通信。本独立运行分支不在 STM32 中接入视觉协议、目标跟踪模式或二维云台控制。当前仓库没有 OpenMV 程序目录，因此 OpenMV 侧实现只能作为待提供、待实测的仓库外工作，不能标记为已完成。

本阶段优先级：

1. 保持 `User / App / Components / BSP` 分层结构不再大改；
2. 不改已经确认的引脚、PWM、USART、TB6612 方向配置；
3. 保持已连续完成 5 圈实车测试的循迹参数冻结；
4. 编码器速度闭环暂缓，只允许在直角弯退出上使用已验证的右编码器；
5. OpenMV 和 MG996R 独立调试，不因视觉侧工作改动 STM32 循迹参数或串口链路。

## 2. 当前架构

```text
控制器 A：STM32（本仓库）
  User/main.c
    -> App/app_robot.c          初始化、任务调度、模式切换
    -> App/app_line_follow.c    矩形循迹状态机

  Components/
    chassis.c                   差速底盘抽象和安全限幅
    tb6612_motor.c              TB6612FNG 方向和 PWM 适配
    yahboom_tracker8_io.c       8 路循迹输入和误差计算
    pid.c                       通用 PID

  BSP/
    bsp_gpio.c                  公共 GPIO/LED
    bsp_pwm.c                   TIM1 PA8/PA9 电机 PWM
    bsp_encoder.c               TIM2/TIM4 编码器
    bsp_uart.c                  USART2 HC-05 控制与遥测
    bsp_systick.c               1ms 系统节拍

控制器 B：OpenMV（仓库外，程序缺失）
  红色色块识别 -> MG996R 水平云台

控制器 A 与控制器 B：无 UART、无数据通信、无任务等待关系
```

`App/app_config.h` 当前承担跨层配置角色。BSP 和 Components include 它只是读取宏配置，不代表 BSP 反向调用 App 业务逻辑。赛前不再拆分配置文件。

## 3. 当前循迹状态

截至当前测试：

- `LINE_CORNER_ENCODER_TARGET=495`、`LINE_RECOVER_CORRECTION_LIMIT=40`、`LINE_RECOVER_PWM=200`、`LINE_RECOVER_CENTER_CONFIRM_MS=250U` 的同一套参数已现场连续完成 5 圈；
- 测试全程未修改参数，当前底盘适合冻结用于校赛展示；
- 直线读取正常，常见稳定数据为 `RAW=0x18 / 0x1C / 0x38`；
- 直角方向已改为固定左转：`RECT_DEFAULT_CORNER_DIR = (-1)`；
- 横线/宽黑线直角识别当前开启：`RECT_ENABLE_CROSS_CORNER = 1`；
- 当前使用低速开环 PWM 循迹，速度闭环未启用；
- 右编码器 `RE` 已能看到明显有效计数；
- 左编码器 `LE` 基本仍是 `-1/0/1` 抖动，暂不可用于闭环；
- 当前源码配置 `LINE_CORNER_USE_ENCODER = 1`，左转直角会优先用右编码器累计退出；
- USART2 保持蓝牙 9600 波特率，`printf` 已改为 TXE 中断队列，不再等待逐字节发送；
- 上电默认 `STOP`，蓝牙发送 `START/1` 才进入循迹，`STOP/0` 立即停止底盘输出；
- 直角传感器出口增加 30ms 连续确认，BLIND/LOST 重获线按实际采样帧累计；
- 已增加 `DT/OV/F/TD` 调度、传感器和串口队列诊断字段；
- 现场常见角点链路包含 `CORNER -> BLIND -> RECOVER -> FOLLOW`，恢复路径并非每次最短，但能够重新捕线。

当前主要风险：

1. 个别角点的恢复路径较长，`RECOVER` 可能短暂返回 `BLIND`；
2. 左右电机开环速度不一致，实际速度差未通过闭环消除；
3. 左编码器历史观测不稳定，不能做左右轮速度闭环；
4. 完整 5 圈原始串口日志未保留，只有部分尾部状态；
5. 仓库内没有 OpenMV 程序，无法在本工程中审查或验证红色色块识别和 MG996R 水平控制。

## 4. 赛前实施顺序

### A. 循迹冻结与复测

目标：不改变冻结参数，在正式展示前复现稳定结果。

以下参数和逻辑禁止调整：

```c
LINE_CORNER_ENCODER_TARGET
LINE_RECOVER_CORRECTION_LIMIT
LINE_RECOVER_PWM
LINE_RECOVER_CENTER_CONFIRM_MS
APP_CONTROL_PERIOD_MS
```

同时冻结 PID、状态机、传感器权重、电机方向、定时器和硬件映射。出现差异时优先检查烧录版本、供电、接线、传感器高度和机械状态。

验收时必须使用同一个 commit 和配置连续跑完 3 圈，并保存完整遥测；目标为
`OV=0`、`F=0`、`TD=0`，且每个直角退出时 `SUM` 不出现异常跃迁。
每次上电还必须确认未发送 START 前车轮不转，STOP 命令在当前主循环内生效。

### B. 接线可靠性

目标：电池单独供电稳定，跑动时不复位、不掉线。

必须确认：

- STM32 电源灯在电池供电下稳定；
- LM2596 输出稳定 5V；
- 电池负极、LM2596 GND、STM32 GND、TB6612 GND、循迹模块 GND 共地；
- 供电和电机大电流线不要依赖面包板；
- 循迹模块 VCC/GND/8 路信号线接触可靠。

### C. OpenMV 独立子系统验收

目标：OpenMV 在不依赖 STM32 数据的情况下识别红色色块，并直接控制 MG996R 进行单轴水平跟踪。

本仓库当前没有 OpenMV 程序目录，不能在此分支实现或声称完成该功能。取得实际 OpenMV 脚本后，应在 OpenMV 侧单独进行：

- 红色色块识别与丢失目标处理；
- MG996R 水平角度范围、方向、中心位和限位验证；
- OpenMV 单独重启或故障时，STM32 底盘仍能接收 HC-05 命令并循迹；
- STM32 单独重启或停止时，OpenMV 任务不等待任何 STM32 消息；
- 确认两控制器之间没有 UART、软串口或隐含心跳依赖。

## 5. 暂不做的事

赛前暂不做：

- 大规模重构；
- 左右轮速度闭环；
- 复杂里程计；
- 多模式 UI；
- 高速循迹；
- 在 STM32 中恢复视觉串口协议、目标跟踪模式或二维云台控制；
- 在独立版两控制器之间新增数据通信；
- 同时改多个硬件接口。

这些内容赛后再恢复推进。
