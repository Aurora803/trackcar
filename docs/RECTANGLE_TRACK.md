# 矩形循迹说明

本文记录当前矩形循迹的实际状态、串口字段含义和赛前调试重点。当前版本先不接 OpenMV / 树莓派视觉，底盘只做矩形循迹。

## 1. 当前模式

当前循迹模式：

- 赛道方向：按当前配置固定左转；
- 直线循迹：8 路循迹传感器 + 位置式 PID 差速；
- 直角弯：传感器识别进入 `CORNER`，优先按编码器累计退出；
- 速度控制：仍是 PWM 开环，不做左右轮速度闭环；
- 编码器状态：右编码器 `RE` 有效，左编码器 `LE` 暂不可靠。
- 启停方式：上电默认 STOP，蓝牙 `START/1` 启动，`STOP/0` 停车。

当前关键配置以 `App/app_config.h` 为准：

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
#define LINE_CORNER_INNER_PWM          60
#define LINE_CORNER_OUTER_PWM          280
#define LINE_CORNER_ALIGN_INNER_PWM    120
#define LINE_CORNER_ALIGN_OUTER_PWM    180
#define LINE_BLIND_TIMEOUT_MS          2000U
```

注意：如果烧录后串口里 `S=3` 时 `SUM` 仍经常跑到 1000 以上才退出，说明固件可能没有烧到最新版本，或者编码器退出条件没有真正生效。

## 2. 状态机

```text
START -> FOLLOW -> CORNER -> RECOVER -> FOLLOW
              \-> BLIND -> RECOVER -> FOLLOW
              \-> LOST
```

| 状态 | 编号 | 含义 |
|---|---:|---|
| `LINE_STATE_START` | 0 | 每次蓝牙启动后的 200ms 稳定等待 |
| `LINE_STATE_FOLLOW` | 1 | 正常循迹 |
| `LINE_STATE_BLIND` | 2 | 暂时丢线，按最后误差方向低速找线 |
| `LINE_STATE_CORNER` | 3 | 直角弯转向 |
| `LINE_STATE_RECOVER` | 4 | 出弯或找回线后的低速恢复 |
| `LINE_STATE_LOST` | 5 | 长时间找不到线，停车 |

当前最常见失败链路：

```text
S=3 CORNER
S=4 RECOVER
S=2 BLIND RAW=0x00
S=5 LOST
```

这表示直角后传感器没有重新压回黑线，最后停车。

## 3. 8 路传感器

传感器从车头视角左到右对应：

```text
X1 X2 X3 X4 X5 X6 X7 X8
左 ------------------ 右
bit0              bit7
```

当前直线较好的串口形态：

```text
RAW=0x18 ERR=0
RAW=0x1C ERR=-133
RAW=0x38 ERR=133
```

需要重点检查的异常：

- `RAW=0x00`：完全丢线，或者传感器没有看到黑线；
- `RAW=0xFF`：全黑，悬空测试时可能出现，放在跑道上若频繁出现则说明阈值/高度过灵敏；
- `RAW=0x1A / 0x3A`：出现 X2、X4 亮但 X3 不亮，建议单独检查 X3 的高度、阈值和接线。

## 4. 直角识别

当前代码中，一侧 3 路以上触发且另一侧很少触发时成为直角候选，连续 6 帧同方向候选才进入直角弯。方向不再由左右侧形态决定，而是统一使用：

```c
#define RECT_DEFAULT_CORNER_DIR (-1)
```

也就是所有直角默认左转。这样做是为了避免同一个矩形跑道上由于传感器形态变化导致 `DIR=-1 / DIR=1` 来回跳。

达到中心门槛或编码器目标后，电机会先切到 120/180 的柔和对线 PWM；中心线或
有效线连续确认 30ms 后才进入 `RECOVER`，单帧噪声不会直接结束直角。

如果之后换成全右转赛道，只改：

```c
#define RECT_DEFAULT_CORNER_DIR 1
```

## 5. 串口字段

当前 USART2 输出示例：

```text
M=0 S=1 RAW=0x18 ERR=0 LPWM=260 RPWM=260 LE=0 RE=90 C=1 DIR=-1 SUM=610 DT=10 OV=0 F=0 TD=0
```

| 字段 | 含义 |
|---|---|
| `M` | 模式，0 为循迹 |
| `S` | 状态机编号 |
| `RAW` | 8 路循迹位，1 表示检测到黑线 |
| `ERR` | 位置误差，左负右正 |
| `LPWM/RPWM` | 当前左右轮 PWM 命令 |
| `LE/RE` | 左右编码器在最近遥测周期内累计的增量 |
| `C` | 已完成直角弯次数 |
| `DIR` | 当前直角方向，-1 左转，1 右转 |
| `SUM` | 当前直角弯累计编码器计数 |
| `DT` | 最近 500ms 内最大的控制调度间隔 |
| `OV` | 最近 500ms 内控制间隔达到 20ms 的次数 |
| `F` | 传感器健康故障码：0正常，1全未触发超时，2全触发超时，3驱动无效 |
| `TD` | USART2 TX 队列累计丢字节数 |

当前判读重点：

- 直线稳定：`S=1`，`RAW` 在 `0x18 / 0x1C / 0x38` 附近；
- 正确进弯：`S=3`，`DIR=-1`；
- 编码器退出是否生效：`S=3` 退出时 `SUM` 应接近 `LINE_CORNER_ENCODER_TARGET`，而不是长期跑到 1000 以上；
- 出弯稳定：`S=4` 后应回到 `S=1`，不要长期 `RAW=0x00`；
- 失败停车：`S=5 RAW=0x00 LPWM=0 RPWM=0`。
- 调度稳定：`DT` 应接近 10，`OV=0`；串口队列不溢出时 `TD=0`。
- 传感器健康：正常跑道应保持 `F=0`；`F!=0` 时底盘进入 `LOST` 停车。

## 6. 当前已知问题

1. 出弯后仍可能稳不住，第三个弯后容易进入 `BLIND -> LOST`。
2. 右编码器 `RE` 可用，左编码器 `LE` 暂不可用。
3. 当前不能做速度闭环，只能做开环低速循迹。
4. 左右轮实际速度可能不一致，需要后续机械/电机补偿或闭环解决。
5. 若悬空测试，`RAW=0xFF` 不一定代表跑道上异常；必须以贴近跑道高度测试为准。
