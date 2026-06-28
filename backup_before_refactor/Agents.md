# STM32 循迹小车项目规则

## 项目背景

这是一个基于 STM32 的循迹小车项目，包含 main、track、pid、motor、pwm、encoder、timer、usart、sys 等模块。后续需要扩展八路循迹、速度闭环、OpenCV/OpenMV 云台打靶、多模式切换等功能。

## 最高优先级要求

- 修改任何文件前，必须先备份当前工程。
- 如果当前目录是 Git 仓库，先创建新分支 `refactor-car-architecture`，并提交原始代码，提交信息为 `backup: original project before refactor`。
- 如果不是 Git 仓库，先复制完整工程到 `backup_before_refactor` 文件夹。
- 备份完成前，不允许修改、删除、移动任何现有文件。
- 不要破坏现有硬件引脚、定时器、PWM、编码器、USART 等底层配置。
- 不确定的硬件参数必须保留原值，并添加注释说明。
- 不要一次性推倒重写，要在现有代码基础上渐进式重构。

## 架构目标

项目应按以下层次组织：

1. 感知层：track.c/h、encoder.c/h  
   只负责读取传感器、编码器数据，返回循迹偏差、丢线状态、速度或计数。

2. 算法层：pid.c/h  
   只负责 PID 计算。不得直接读取传感器，不得直接控制电机或 PWM。

3. 执行层：motor.c/h、pwm.c/h  
   只负责电机方向、PWM 输出、速度设置和停止。

4. 控制层：car_control.c/h  
   负责把 track、pid、encoder、motor 串起来，承载循迹控制、速度控制、模式切换。

5. 应用层：main.c  
   只负责初始化和任务调度，不堆大量控制算法。

6. 定时器层：timer.c/h  
   中断中只置控制标志位，不执行复杂 PID、printf 或电机控制逻辑。

7. 通信层：usart.c/h  
   只负责调试输出和参数接收，不直接操作底层 PWM。

## 模块设计原则

- track 模块不得直接调用 motor。
- encoder 模块不得参与 PID 运算。
- pid 模块不得读传感器，不得写 PWM。
- motor 模块不得关心循迹算法。
- main.c 不应包含大段循迹控制细节。
- 尽量减少 extern 全局变量。
- 中断和主循环共享变量必须使用 volatile。
- PID 必须支持积分限幅和输出限幅。
- 电机 PWM 限幅应统一处理。
- 保留旧函数时应添加兼容注释，不要随意删除不确定用途的函数。

## 建议接口

PID:
- PID_Init
- PID_Calculate
- PID_Reset

Track:
- Track_Init
- Track_ReadSensors
- Track_GetError
- Track_IsLostLine

Motor:
- Motor_Init
- Motor_SetPWM
- Motor_SetSpeed
- Motor_Stop

Encoder:
- Encoder_Init
- Encoder_Update
- Encoder_GetLeftSpeed
- Encoder_GetRightSpeed

CarControl:
- CarControl_Init
- CarControl_Task
- CarControl_SetMode
- CarControl_SetBaseSpeed
- CarControl_Stop

Timer:
- Timer_Init
- extern volatile uint8_t control_flag

## 输出要求

每次修改后必须说明：

1. 已完成的备份位置。
2. 当前项目主要架构问题。
3. 本次修改了哪些文件。
4. 新增了哪些文件。
5. 每个文件的关键修改点。
6. 是否有无法自动确认的硬件问题，例如电机方向、传感器黑白电平、PWM 范围、编码器方向。