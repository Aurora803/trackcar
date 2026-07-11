# 移植说明

## 1. 先改配置

优先修改：

```text
App/app_config.h
```

这里集中管理：

- PWM 频率；
- 电机方向反转；
- 编码器方向反转；
- 循迹黑线电平；
- 控制周期；
- PID 初始参数；
- 串口波特率。

## 2. 换引脚

如果只换引脚，不建议改 `app_line_follow.c`、`chassis.c`、`pid.c` 等逻辑文件。应改：

- `App/app_config.h`：宏定义；
- `Components/tb6612_motor.c`：TB6612 方向控制 GPIO 初始化；
- `Components/yahboom_tracker8_io.c`：8 路循迹输入 GPIO 初始化；
- `BSP/bsp_gpio.c`：AFIO/SWD、板载 LED 等公共 GPIO 初始化；
- `BSP/bsp_pwm.c`：PWM 定时器通道；
- `BSP/bsp_encoder.c`：编码器定时器。

## 3. 调试顺序

1. 只烧录后观察 PC13 LED 是否闪烁；
2. USART2 调试串口是否输出启动信息；
3. 断开电机负载，测试 TB6612 方向逻辑；
4. 架空车轮，测试编码器正反；
5. 串口打印 8 路循迹 bit mask；
6. 低速调循迹 PID；
7. 再考虑编码器速度闭环和视觉云台。

## 4. 常见问题

### 电机一边反向

改 `MOTOR_LEFT_INVERT` 或 `MOTOR_RIGHT_INVERT`。

### 编码器计数方向反了

改 `ENCODER_LEFT_INVERT` 或 `ENCODER_RIGHT_INVERT`。

### 循迹检测结果全反

改 `TRACKER_BLACK_ACTIVE_LOW`。

### PB3/PB4 读不到

确认保留 SWD、关闭 JTAG；公共 AFIO 配置位于 `BSP_GPIO_InitAll()`。

### 调试串口输出影响实时性

当前 `printf` 通过 USART2 TXE 中断队列非阻塞发送，蓝牙保持 9600 8N1，
遥测周期为 500ms。若增加日志字段或提高输出频率，应通过
`BSP_DebugUART_GetTxDroppedCount()` 检查队列是否溢出。

蓝牙控制默认启用：`1` 或带行结束的 `START/GO` 启动，`0` 或带行结束的
`STOP` 停止。移植到其他蓝牙模块时，如果需要断连自动停车，必须额外接入
模块 STATE 引脚，或让上位机周期发送心跳并在固件中实现超时停车。

### 接上树莓派后 STM32 复位

通常是供电跌落或共地/电流不足。树莓派不要直接从 STM32 3.3V 取电，建议单独 5V 降压供电，共地即可。
