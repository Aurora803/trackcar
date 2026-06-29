# 矩形赛道循迹说明

本版本先不接 OpenMV / 树莓派视觉，底盘只按闭合矩形赛道循迹。

## 1. 状态机

```text
START -> FOLLOW -> CORNER -> RECOVER -> FOLLOW
              \-> BLIND -> RECOVER -> FOLLOW
              \-> LOST  -> RECOVER -> FOLLOW
```

| 状态 | 作用 |
|---|---|
| `LINE_STATE_START` | 上电后等待 200ms，避免刚上电传感器/电机状态不稳 |
| `LINE_STATE_FOLLOW` | 正常循迹，使用 8 路误差 + 位置式 PID 输出左右轮差速 |
| `LINE_STATE_BLIND` | 连续丢线超过 20ms 后，用最后一次有效误差进行惯性追线 |
| `LINE_STATE_CORNER` | 检测到矩形直角弯后，按固定方向/传感器方向执行定量转弯 |
| `LINE_STATE_RECOVER` | 转弯或丢线找回后低速恢复，避免直接回 FOLLOW 造成蛇形 |
| `LINE_STATE_LOST` | 长时间找不到线，停车等待重新检测到线 |

## 2. 直角弯识别

8 路循迹从左到右对应 X1~X8：

```text
X1 X2 X3 X4 X5 X6 X7 X8
左 ------------------ 右
```

识别规则：

- 左侧 X1~X4 中 3 路以上触发、右侧 X5~X8 很少触发：判断左直角；
- 右侧 X5~X8 中 3 路以上触发、左侧 X1~X4 很少触发：判断右直角；
- 6 路以上同时触发，认为是宽黑线/横线特征；当前 `RECT_ENABLE_CROSS_CORNER = 0`，默认不直接按横线进入直角弯，确认赛道直角处稳定出现该特征后再打开。

默认：

```c
#define RECT_DEFAULT_CORNER_DIR 1   /* 1 右转/顺时针，-1 左转/逆时针 */
```

如果你的车在矩形赛道上需要逆时针跑，把它改成：

```c
#define RECT_DEFAULT_CORNER_DIR (-1)
```

## 3. 调参入口

重点参数都在 `App/app_config.h`：

```c
#define LINE_BASE_PWM_FAST          300
#define LINE_BASE_PWM_MID           260
#define LINE_BASE_PWM_SLOW          220
#define LINE_CORNER_INNER_PWM       80
#define LINE_CORNER_OUTER_PWM       300
#define LINE_CORNER_USE_ENCODER     0
#define LINE_CORNER_TIME_MS         330U
#define LINE_CORNER_ENCODER_TARGET  550
#define LINE_PID_KP                 0.16f
#define LINE_PID_KD                 0.004f
```

调试顺序建议：

1. 先把 `LINE_BASE_PWM_FAST/MID/SLOW` 降低到车能慢速稳定跑；
2. 再调 `LINE_PID_KP`，让车能明显向黑线修正；
3. 如果直线左右蛇形，略微降低 `KP` 或增加一点 `KD`；
4. 当前 `LINE_CORNER_USE_ENCODER = 0`，直角弯主要按 `LINE_CORNER_TIME_MS` 定时退出；
5. 如果直角弯转不够，先小幅增大 `LINE_CORNER_TIME_MS` 或 `LINE_CORNER_OUTER_PWM`；
6. 如果直角弯转过头，先减小 `LINE_CORNER_TIME_MS` 或降低 `LINE_CORNER_OUTER_PWM`；
7. 编码器反馈稳定后，再把 `LINE_CORNER_USE_ENCODER` 改为 1 并重新调 `LINE_CORNER_ENCODER_TARGET`。

## 4. 串口调试字段

当前 USART2 输出格式：

```text
M=0 S=1 RAW=0x18 ERR=0 LPWM=260 RPWM=260 LE=12 RE=13 C=0 DIR=0 SUM=0
```

字段含义：

| 字段 | 含义 |
|---|---|
| `M` | 机器人模式，0 为循迹 |
| `S` | 循迹状态机状态，0 START、1 FOLLOW、2 BLIND、3 CORNER、4 RECOVER、5 LOST |
| `RAW` | 8 路循迹原始位，bit0=X1，bit7=X8，1 表示检测到黑线 |
| `ERR` | 位置误差，左负右正 |
| `LPWM/RPWM` | 左右轮当前 PWM 命令 |
| `LE/RE` | 左右编码器在最近 10ms 控制周期内的增量 |
| `C` | 已完成直角弯次数 |
| `DIR` | 当前直角弯方向，-1 左转，1 右转 |
| `SUM` | 当前直角弯累计编码器脉冲 |
