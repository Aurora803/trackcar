# STM32 循迹小车项目规则

## 项目背景

这是一个基于 STM32F103C8T6 的矩形循迹小车项目。当前工程已经从早期扁平模块
演进为 `User / App / Components / BSP` 分层结构，重点功能是保持现有矩形循迹、
TB6612FNG 电机控制、8 路循迹输入和 USART2 调试输出稳定。后续再扩展速度闭环、
OpenCV/OpenMV 云台打靶和多模式切换。

## 当前状态说明

- 本文件最早记录的是重构目标和协作规则，其中“先备份、渐进式修改、不要破坏硬件配置”等流程要求仍然有效。
- 早期提到的 `track.c`、`motor.c`、`car_control.c`、`timer.c`、`control_flag` 等文件名不是当前工程结构，不要为了匹配旧文档而新建或回退到这些模块。
- 当前权威架构以 `docs/PLAN.md` 和实际源码为准：`User/main.c` 只调度，`App` 承载应用逻辑，`Components` 承载可复用组件和抽象接口，`BSP` 承载外设初始化与寄存器访问。
- `App/app_config.h` 当前承担跨层 Config 角色，BSP/Components include 它只是读取宏配置，不代表 BSP 反向调用 App 业务逻辑。

## 最高优先级要求

- 修改任何文件前，必须先备份当前工程。
- 如果当前目录是 Git 仓库，先创建新分支 `refactor-car-architecture`，并提交原始代码，提交信息为 `backup: original project before refactor`。
- 如果不是 Git 仓库，先复制完整工程到 `backup_before_refactor` 文件夹。
- 备份完成前，不允许修改、删除、移动任何现有文件。
- 不要破坏现有硬件引脚、定时器、PWM、编码器、USART 等底层配置。
- 不确定的硬件参数必须保留原值，并添加注释说明。
- 不要一次性推倒重写，要在现有代码基础上渐进式重构。

## 架构目标

项目当前按以下层次组织：

1. User 启动层：`User/main.c`
   只负责 `AppRobot_Init()` 和主循环内反复调用 `AppRobot_Task()`，不堆控制算法。

2. App 应用层：`App/app_robot.c/h`、`App/app_line_follow.c/h`、`App/app_vision_target.c/h`
   `app_robot` 负责初始化、任务调度和模式切换；`app_line_follow` 负责矩形循迹状态机和循迹 PID 调度；`app_vision_target` 是视觉/云台预留，默认关闭。

3. Components 组件层：`chassis`、`tb6612_motor`、`yahboom_tracker8_io`、`pid`、`vision_protocol`、`gimbal_*`
   组件层提供可复用逻辑和 driver 抽象接口。`motor_if.h`、`tracker8_if.h`、`gimbal_if.h` 用于隔离 App 和具体硬件实现。

4. BSP 板级层：`bsp_gpio`、`bsp_pwm`、`bsp_encoder`、`bsp_uart`、`bsp_systick`、`bsp_servo`
   只负责外设初始化、GPIO 读写、PWM、编码器、USART、SysTick 等底层访问。除公共 AFIO/LED 外，具体外设 GPIO 应由对应模块 Init 负责。

5. Config 配置角色：`App/app_config.h`
   集中保存引脚、PWM、方向、电平、控制周期、PID 初值和功能开关。后续若进一步解耦，可拆为 `Config/board_config.h` 和 `App/app_config.h`。

## 模块设计原则

- `yahboom_tracker8_io` 只负责读取 8 路循迹输入、转换黑线有效电平和计算位置误差，不得直接调用电机。
- `bsp_encoder` 只负责 TIM2/TIM4 编码器模式和增量读取，不参与 PID 运算。
- `pid` 只负责 PID 计算，不得读传感器，不得写 PWM。
- `tb6612_motor` 只负责 TB6612 方向、刹车、空转和 PWM 命令适配，不关心循迹算法。
- `chassis` 负责差速底盘抽象和底盘安全限幅，上层不直接操作 TB6612 细节。
- `User/main.c` 不应包含大段循迹控制细节。
- 尽量减少 extern 全局变量。
- 中断和主循环共享变量必须使用 volatile。
- PID 必须支持积分限幅和输出限幅。
- 电机 PWM 限幅应统一处理。
- 保留旧函数时应添加兼容注释，不要随意删除不确定用途的函数。
- 不要为了匹配旧文档而新建 `track.c`、`motor.c`、`car_control.c` 或 `control_flag` 机制；当前工程使用 App/BSP/Components 分层和 SysTick 时间戳调度。

## 建议接口

当前主要接口：

App:
- `AppRobot_Init`
- `AppRobot_Task`
- `AppRobot_SetMode`
- `AppRobot_GetMode`
- `AppLineFollow_Init`
- `AppLineFollow_Update`
- `AppLineFollow_GetDebug`

Components:
- `PID_Init`
- `PID_Update`
- `PID_Reset`
- `PID_SetGains`
- `Chassis_Init`
- `Chassis_SetPWM`
- `Chassis_UpdateEncoder`
- `Chassis_GetState`
- `Chassis_StopCoast`
- `TB6612_Init`
- `TB6612_SetSpeedPermille`
- `TB6612_Brake`
- `TB6612_Coast`
- `TB6612_GetDriver`
- `YahboomTracker8IO_Init`
- `YahboomTracker8IO_Read`
- `YahboomTracker8IO_GetDriver`

BSP:
- `BSP_GPIO_InitAll`
- `BSP_PWM_MotorInit`
- `BSP_PWM_SetMotorDutyPermille`
- `BSP_Encoder_Init`
- `BSP_Encoder_ReadLeftDelta`
- `BSP_Encoder_ReadRightDelta`
- `BSP_UART_Init`
- `BSP_DebugUART_SendChar`
- `BSP_SysTick_Init`
- `BSP_GetTickMs`

## 输出要求

每次修改后必须说明：

1. 已完成的备份位置。
2. 当前项目主要架构问题。
3. 本次修改了哪些文件。
4. 新增了哪些文件。
5. 每个文件的关键修改点。
6. 是否有无法自动确认的硬件问题，例如电机方向、传感器黑白电平、PWM 范围、编码器方向。
