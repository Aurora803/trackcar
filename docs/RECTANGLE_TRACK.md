# 矩形循迹说明

本文描述 `school-competition-line-v1` 的真实源码行为和连续 5 圈实车结果。循迹模块已冻结；视觉和云台当前未启用。

## 1. 当前稳定配置

以下参数来自 `App/app_config.h`，是校赛稳定配置：

```c
#define RECT_DEFAULT_CORNER_DIR          (-1)
#define RECT_ENABLE_CROSS_CORNER         1
#define LINE_CORNER_USE_ENCODER          1
#define LINE_CORNER_ENCODER_TARGET       495
#define LINE_RECOVER_CORRECTION_LIMIT    40
#define LINE_RECOVER_PWM                 200
#define LINE_RECOVER_CENTER_CONFIRM_MS   250U
```

源码中的角点编码器判定是：

```c
g_corner_encoder_sum >= LINE_CORNER_ENCODER_TARGET
```

历史出现过的 `450/550/600` 均为历史实验参数，当前已停用，不代表校赛稳定配置。

## 2. 工作模式

- 赛道方向按当前配置固定左转；
- 直线由 8 路循迹传感器计算位置误差，使用纯 P 差速；
- 直角由传感器识别进入 `CORNER`，优先使用对应外轮编码器累计值；
- 底盘为开环 PWM，不启用左右轮速度闭环；
- 蓝牙启用时上电默认 STOP，`START/1` 启动，`STOP/0` 停车。

## 3. 状态机与实车链路

| 状态 | 编号 | 含义 |
|---|---:|---|
| `START` | 0 | 启动后 200 ms 稳定等待 |
| `FOLLOW` | 1 | 正常循迹 |
| `BLIND` | 2 | 暂时丢线并搜索 |
| `CORNER` | 3 | 直角转向 |
| `RECOVER` | 4 | 捕线后的低速恢复 |
| `LOST` | 5 | 搜索超时或故障后停车 |

现场常见的角点状态链不是 `CORNER` 直接回到 `FOLLOW`，而是：

```text
FOLLOW -> CORNER -> BLIND -> RECOVER -> FOLLOW
 TR=2      TR=7      TR=8       TR=10
```

部分情况下还会出现：

```text
RECOVER --TR=11--> BLIND --TR=8--> RECOVER --TR=10--> FOLLOW
```

这说明恢复期间短暂再次丢线，但车辆仍可重新捕获中心线。不能把当前效果描述为“所有角点状态链完全理想”。准确结论是：角点后容错恢复机制能够有效重新捕获中心线，当前版本满足校赛展示稳定性要求，但仍存在恢复路径偏长的优化空间。

## 4. 迁移原因码

| `TR` | 源码含义 |
|---:|---|
| 1 | START 到 FOLLOW |
| 2 | FOLLOW 识别直角进入 CORNER |
| 3 | FOLLOW 丢线进入 BLIND |
| 4 | CORNER 达编码器目标退出 |
| 5 | CORNER 中心确认退出 |
| 6 | CORNER 超时退出 |
| 7 | CORNER 转入 BLIND 找线 |
| 8 | BLIND 重新捕线进入 RECOVER |
| 9 | BLIND 超时进入 LOST |
| 10 | RECOVER 稳定回到 FOLLOW |
| 11 | RECOVER 再次丢线回到 BLIND |
| 12 | LOST 重新捕线进入 RECOVER |
| 13 | 传感器故障进入 LOST |

## 5. 传感器与角点处理

X1~X8 从车头视角由左到右，对应 `RAW` 的 bit0~bit7。常见中心附近数据为 `RAW=0x18`，轻微偏移可见 `0x1C` 或 `0x38`；`RAW=0x00` 表示未检测到线，`RAW=0xFF` 表示全触发。

直角候选需连续 6 帧成立。进入 `CORNER` 后累计对应编码器绝对增量；达到目标或满足其他退出条件后尝试中心确认。若不能在角点阶段稳定捕获有效线，状态机以 `TR=7` 进入 `BLIND`，再以 `TR=8` 进入 `RECOVER`。`RECOVER` 只有在恢复时长满足且中心连续确认达到 250 ms 后，才以 `TR=10` 返回 `FOLLOW`。

## 6. 调试日志

周期遥测字段：

```text
M S RAW ERR LPWM RPWM LE RE C DIR SUM DT OV F TD ARM RM RC ST RCM RLM TR
```

状态变化事件格式：

```text
EV FROM=<旧状态> TO=<新状态> TR=<原因> OST=<旧状态时长> RAW=<采样> ERR=<误差> SUM=<角点累计> ...
```

判读重点：

- `S` 为当前状态，`TR` 为最近一次迁移原因；
- `SUM` 是当前角点编码器累计值；
- `RCM` 是 RECOVER 中心稳定确认计时，`RLM` 是 RECOVER 丢线确认计时；
- `DT/OV` 反映调度，`F` 是传感器故障码，`TD` 是串口 TX 队列累计丢字符数。

本次连续 5 圈测试的串口工具只保存了部分尾部状态记录。因此日志可辅助分析恢复链路，但不能表述为“完整串口日志证明了 20 个角点”。

## 7. 连续 5 圈结果

- `LINE_CORNER_ENCODER_TARGET=495` 的版本现场连续完成 5 圈；
- 全程未修改参数；
- 现场观察确认车辆能够完成矩形循迹运行；
- 角点主要依靠 `BLIND` 与 `RECOVER` 容错重新捕线；
- 当前版本适合冻结用于校赛展示。

## 8. 已知限制

1. 恢复链路偶尔较长，`RECOVER` 可能短暂返回 `BLIND`。
2. 左编码器历史观测不稳定，当前不用于速度闭环。
3. 左右轮仍为开环 PWM，实际速度差未通过闭环消除。
4. 完整 5 圈原始串口日志未保留。
5. 视觉与云台尚未联调，不参与当前循迹运行。
