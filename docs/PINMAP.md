# STM32F103C8T6 循迹小车接线图（矩形循迹版）

本文件按用户最新上传的接线图更新。当前目标：**只跑矩形循迹，不启用视觉/云台**。

## 1. TB6612FNGVM 电机驱动

| STM32 引脚 | TB6612 引脚 | 外设模式 | 说明 |
|---|---|---|---|
| PA9 | PWMA | TIM1_CH2 / AF_PP | 左电机 PWM |
| PA8 | PWMB | TIM1_CH1 / AF_PP | 右电机 PWM |
| PB12 | AIN1 | GPIO_OUT | 左电机方向 1 |
| PB13 | AIN2 | GPIO_OUT | 左电机方向 2 |
| PB14 | BIN1 | GPIO_OUT | 右电机方向 1 |
| PB15 | BIN2 | GPIO_OUT | 右电机方向 2 |
| 3.3V | STBY | 固定高电平 | TB6612 使能，不占用 STM32 GPIO |
| 12V 电池正极 | VM | 电机电源 | 给电机供电 |
| 3.3V | VCC | 逻辑电源 | TB6612 逻辑电平 |
| GND | GND | 共地 | 必须和 STM32、LM2596 共地 |

PWM 参数：TIM1，1 kHz，PSC=71，ARR=999，占空比 0~1000。

方向逻辑按当前工程定义：

| 状态 | AIN1/PB12 | AIN2/PB13 | BIN1/PB14 | BIN2/PB15 |
|---|---:|---:|---:|---:|
| 前进 | 0 | 1 | 1 | 0 |
| 后退 | 1 | 0 | 0 | 1 |
| 空转停止 | 0 | 0 | 0 | 0 |
| 刹车 | 1 | 1 | 1 | 1 |

> 工程已经按这张方向表把 `MOTOR_LEFT_INVERT` 默认设为 1、`MOTOR_RIGHT_INVERT` 默认设为 0。如果实车方向仍不对，优先改这两个宏，不要直接改 TB6612 底层函数。

## 2. 编码器

| STM32 引脚 | 功能 | 外设模式 |
|---|---|---|
| PA0 | 左编码器 A | TIM2_CH1 |
| PA1 | 左编码器 B | TIM2_CH2 |
| PB6 | 右编码器 A | TIM4_CH1 |
| PB7 | 右编码器 B | TIM4_CH2 |

## 3. 8 路循迹模块

从车头视角、传感器阵列从左到右：

| 传感器 | STM32 引脚 | 工程 bit 位 | 权重 |
|---|---|---:|---:|
| X1 最左 | PB11 | bit0 | -1200 |
| X2 | PB10 | bit1 | -800 |
| X3 | PB1 | bit2 | -400 |
| X4 | PB0 | bit3 | -100 |
| X5 | PA7 | bit4 | +100 |
| X6 | PA6 | bit5 | +400 |
| X7 | PA5 | bit6 | +800 |
| X8 最右 | PA4 | bit7 | +1200 |

默认黑线低电平有效：`TRACKER_BLACK_ACTIVE_LOW = 1`。

## 4. 调试串口

| STM32 引脚 | 功能 | 连接 USB-TTL |
|---|---|---|
| PA2 | USART2_TX | 接 USB-TTL RX |
| PA3 | USART2_RX | 接 USB-TTL TX |
| GND | GND | 接 USB-TTL GND |

参数：9600，8N1，无流控。

> PA2/PA3 也是后续 OpenMV/树莓派视觉串口最可能复用的位置。当前 STM32F103C8T6 接线下没有第二组不冲突的硬件 UART，调试 USB-TTL 和视觉主机不能同时独立占用 USART2。

## 5. 系统节拍与预留定时器

当前控制节拍由 SysTick 提供：`SysTick_Handler()` 每 1 ms 调用 `BSP_SysTick_Inc()`，`AppRobot_Task()` 在主循环中按毫秒差值调度 10 ms 控制、50 ms 遥测和 500 ms LED。

TIM3 当前代码未占用。若后续要加二自由度舵机云台，优先评估 TIM3 部分重映射到 PB4/PB5；不要使用 PB0/PB1，因为它们已经接循迹 X4/X3。

## 6. 电源

| 连接 | 说明 |
|---|---|
| 12V 电池正极 -> TB6612 VM | 电机电源 |
| 12V 电池正极 -> LM2596 IN+ | 降压输入 |
| 电池负极 -> LM2596 IN- / TB6612 GND / STM32 GND | 所有 GND 必须共地 |
| LM2596 5V -> STM32 5V | 给 BluePill/最小系统板供电 |
| LM2596 5V -> 8 路循迹 VCC | 模块供电 |
| STM32 3.3V -> TB6612 VCC | TB6612 逻辑电源 |
| STM32 3.3V -> TB6612 STBY | 固定使能 |

## 7. 引脚冲突提醒

1. PA8/PA9 已用于 TIM1 电机 PWM，不能再作为 USART1 或云台 PWM。
2. PB0/PB1 已用于循迹 X4/X3，不能再作为 TIM3 电机 PWM。
3. PA2/PA3 已用于 USART2 调试，不能再给循迹或视觉串口同时使用。
4. PB4/PB5 当前可作为后续 TIM3_CH1/CH2 舵机 PWM 候选，但需要关闭 JTAG 并启用 TIM3 部分重映射。
5. 先不启用视觉；后续加 OpenMV/树莓派时，需要重新规划串口或使用软件串口/外部扩展。
