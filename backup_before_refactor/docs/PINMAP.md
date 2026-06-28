# 按用户接线图更新后的引脚表

## 电机驱动 TB6612FNG

| STM32 引脚 | 连接 | 外设/模式 | 说明 |
|---|---|---|---|
| PB0 | PWMA | TIM3_CH3 / AF_PP | 左电机 PWM |
| PB1 | PWMB | TIM3_CH4 / AF_PP | 右电机 PWM |
| PB12 | AIN1 | GPIO_OUT | 左电机方向 1 |
| PB13 | AIN2 | GPIO_OUT | 左电机方向 2 |
| PB14 | BIN1 | GPIO_OUT | 右电机方向 1 |
| PB15 | BIN2 | GPIO_OUT | 右电机方向 2 |
| PA8 | STBY | GPIO_OUT | 高电平工作，低电平待机 |

## 编码器

| STM32 引脚 | 连接 | 外设/模式 |
|---|---|---|
| PA0 | 左编码器 A | TIM2_CH1 / Encoder |
| PA1 | 左编码器 B | TIM2_CH2 / Encoder |
| PB6 | 右编码器 A | TIM4_CH1 / Encoder |
| PB7 | 右编码器 B | TIM4_CH2 / Encoder |

## 8 路循迹

| STM32 引脚 | 连接 | 模式 |
|---|---|---|
| PA2 | S1 | GPIO_IPU |
| PA3 | S2 | GPIO_IPU |
| PA4 | S3 | GPIO_IPU |
| PA5 | S4 | GPIO_IPU |
| PB8 | S5 | GPIO_IPU |
| PB9 | S6 | GPIO_IPU |
| PB10 | S7 | GPIO_IPU |
| PB11 | S8 | GPIO_IPU |

## 串口 / 视觉预留

| STM32 引脚 | 连接 | 外设/模式 | 说明 |
|---|---|---|---|
| PA9 | USART1_TX | USART1 | 调试输出，也可接视觉端 RX |
| PA10 | USART1_RX | USART1 | 调试输入，也可接视觉端 TX |

> 注意：PA2/PA3 已用于循迹 S1/S2，所以本版默认不再初始化 USART2。视觉协议预留改为 USART1。
> 如果后续要同时使用“USB-TTL 调试”和“OpenMV/树莓派视觉串口”，建议新增别的串口映射方案，或将 8 路循迹改为 I2C/串口方式释放 PA2/PA3。

## 电源

| 连接 | 说明 |
|---|---|
| 电池正极 -> TB6612 VM | 电机电源输入 |
| 电池正极 -> LM2596 IN+ | 降压输入正极 |
| 电池负极 -> LM2596 IN- / TB6612 GND / STM32 GND | 必须共地 |
| LM2596 5V -> STM32 5V | STM32 板 5V 输入 |
| LM2596 5V -> 8 路循迹模块 VCC | 循迹模块供电 |
| LM2596 5V -> OpenMV / 树莓派 / 云台舵机预留 | 后续扩展供电，注意电流余量 |
| STM32 3.3V -> TB6612 VCC | TB6612 逻辑电源 |
| 所有 GND -> 共地 | 不共地会导致 PWM/串口/方向控制异常 |

## 关键冲突说明

1. PB0/PB1 现在用于 TIM3_CH3/TIM3_CH4 电机 PWM，不能再给循迹输入。
2. PA2/PA3 现在用于循迹 S1/S2，不能再启用 USART2。
3. PA8 现在用于 TB6612 STBY，不能再作为 TIM1_CH1 舵机 PWM。
4. 当前未给二维云台分配控制引脚，只预留了供电和 USART1 通信协议接口。
