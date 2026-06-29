# 调参建议

## 1. 矩形赛道循迹 PID

当前实际生效参数以 `App/app_config.h` 为准。下面数值是当前基线和初始调参建议，
用于低速跑通矩形赛道；不要为了匹配文档而把代码参数改回旧值。

```c
#define LINE_BASE_PWM_FAST  300
#define LINE_BASE_PWM_MID   260
#define LINE_BASE_PWM_SLOW  220
#define LINE_PID_KP         0.16f
#define LINE_PID_KI         0.00f
#define LINE_PID_KD         0.004f
```

调参顺序：

1. `KI` 保持 0；
2. 先调 `LINE_BASE_PWM_FAST/MID/SLOW`，确保直线和弯道都能稳定慢速行驶；
3. 增大 `KP`，让小车能明显修正偏差；
4. 如果弯道来回摆动，适当增加 `KD`；
5. 如果过弯冲出，先降低 `LINE_BASE_PWM_FAST/MID/SLOW`；
6. 当前 `LINE_CORNER_USE_ENCODER = 0`，直角弯先调 `LINE_CORNER_TIME_MS`、`LINE_CORNER_INNER_PWM`、`LINE_CORNER_OUTER_PWM`；
7. 稳定后再考虑速度闭环。

## 2. 串口遥测频率

当前 USART2 调试输出是阻塞式 `printf`，`APP_TELEMETRY_PERIOD_MS = 50U` 时方便排查
TB6612、电机方向、循迹 RAW 位和编码器增量。提高输出频率或增加字段会占用更多主循环时间，
后续做速度闭环前应评估改为中断或 DMA 发送；本轮不改变默认周期和输出字段。

## 3. 矩形赛道状态机

当前逻辑：

- FOLLOW：正常 PID 循迹，误差小速度高，误差大速度低；
- BLIND：连续丢线超过 20ms 后，按最后一次有效误差低速偏转追线；
- CORNER：识别矩形直角弯后，按一侧慢、一侧快的方式定量转弯；
- RECOVER：转弯或丢线找回后低速恢复 160ms；
- LOST：长时间找不到线后停车等待。

矩形默认转弯方向由 `RECT_DEFAULT_CORNER_DIR` 控制：`1` 为右转/顺时针，`-1` 为左转/逆时针。

## 4. 8 路权重

默认权重：

```text
-1200, -800, -400, -100, 100, 400, 800, 1200
```

权重越大，纠偏越激进。车速越快，通常需要更平滑的权重和更强的 D 项抑制摆动。

## 5. 编码器采样与速度闭环

当前工程仍是 PWM 开环循迹，直角弯也暂时使用定时退出，但 `Chassis_UpdateEncoder()` 已经挪到 10ms 控制周期内。这样做的目的不是现在立刻闭环，而是避免后续接速度 PID 时拿 50ms 遥测数据去算 10ms 控制速度。
