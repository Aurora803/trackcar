# 调参记录与建议

本文只记录当前赛前可执行的调参策略。当前目标是稳定跑完矩形，不追求速度和完整闭环。

## 1. 当前基线

当前实际参数以 `App/app_config.h` 为准：

```c
#define RECT_DEFAULT_CORNER_DIR        (-1)
#define RECT_ENABLE_CROSS_CORNER       1

#define LINE_CORNER_USE_ENCODER        1
#define LINE_CORNER_ENCODER_TARGET     550
#define LINE_CORNER_CENTER_ENABLE_ENCODER 500
#define LINE_CORNER_DEBOUNCE_COUNT     6U
#define LINE_CORNER_EXIT_CONFIRM_MS    30U

#define LINE_BASE_PWM_FAST             240
#define LINE_BASE_PWM_MID              220
#define LINE_BASE_PWM_SLOW             200
#define LINE_RECOVER_PWM               200
#define LINE_BLIND_BASE_PWM            90
#define LINE_BLIND_TURN_PWM            220
#define LINE_CORNER_INNER_PWM          60
#define LINE_CORNER_OUTER_PWM          280
#define LINE_CORNER_ALIGN_INNER_PWM    120
#define LINE_CORNER_ALIGN_OUTER_PWM    180

#define LINE_PID_KP                    0.16f
#define LINE_PID_KI                    0.00f
#define LINE_PID_KD                    0.000f
```

当前结论：

- 直线读取基本正常；
- 全程默认左转；
- 右编码器有效，左编码器暂不可靠；
- 当前只尝试用右编码器稳定左转直角退出；
- 不做左右轮速度闭环。

## 2. 调参原则

每次只改一个方向，不要同时改很多参数。

优先顺序：

1. 先确认烧录的是最新固件；
2. 先看串口状态，不凭肉眼直接猜；
3. 先调直角退出，再调恢复，再调速度；
4. 不改 TB6612 引脚、PWM 通道、电机方向宏；
5. 不碰左编码器闭环，除非 `LE` 已经稳定。

## 3. 串口判读

### 直线正常

```text
S=1 RAW=0x18 ERR=0 LPWM=260 RPWM=260
S=1 RAW=0x1C ERR=-133
S=1 RAW=0x38 ERR=133
```

这是可接受状态。

同时检查新增诊断字段：

```text
DT=10 OV=0 F=0 TD=0
```

- `DT`：最近 500ms 内最大控制间隔，正常应接近 10ms；
- `OV`：控制间隔达到 20ms 的次数，稳定运行应为 0；
- `F`：传感器健康故障码，正常为 0；
- `TD`：USART2 TX 队列累计丢字符数，正常应保持 0。

### 进入左直角

```text
S=3 ... DIR=-1
```

如果应该左转但出现 `DIR=1`，检查：

```c
#define RECT_DEFAULT_CORNER_DIR (-1)
```

### 直角转过头或出弯丢线

```text
S=3 RAW=0x00
S=2 RAW=0x00
S=5 RAW=0x00
```

说明传感器出弯后没有重新压回线。

### 编码器退出没有生效

```text
LINE_CORNER_ENCODER_TARGET = 550
但 S=3 退出时 SUM 经常 > 1000
```

优先检查：

1. 是否重新编译并烧录了最新固件；
2. `LINE_CORNER_USE_ENCODER` 是否为 `1`；
3. 直角方向是否为左转，左转才使用当前有效的右编码器；
4. `RE` 是否仍有稳定计数。

## 4. 下一步推荐调整

当前最新日志显示：第一个弯能找回，后续弯容易在 `BLIND` 或 `LOST` 卡住。下一步不要改 PID，先验证编码器退出。

### 4.1 如果 `SUM` 仍超过 1000 才退出

先不要改目标值，先确认固件烧录。

正确预期：

```text
S=3 ... SUM 接近 550 后进入 S=4
```

如果确认固件无误但仍过头，再临时降低：

```c
#define LINE_CORNER_ENCODER_TARGET     450
```

### 4.2 如果转角不够

每次只加 50：

```c
#define LINE_CORNER_ENCODER_TARGET     600
```

### 4.3 如果出弯后很快又误进 CORNER

优先增加恢复时间：

```c
#define LINE_RECOVER_MS                300U
```

当前源码为 `320U`。如果串口中频繁出现：

```text
S=4 -> S=1 -> S=3
```

可以再试 `350U`。

### 4.4 如果直线频繁误触发直角

先关掉横线直角：

```c
#define RECT_ENABLE_CROSS_CORNER       0
```

再看直线时是否还有 `S=3`。

## 5. 不建议现在做的调整

暂时不要做：

- 增大基础速度；
- 打开速度闭环；
- 调大 PID；
- 同时修改权重和 PID；
- 根据悬空 `RAW=0xFF` 判断传感器坏；
- 用左编码器做右转或里程计。

## 6. 编码器状态

当前测试中：

```text
RE: 有明显累计值，约几十到一百多
LE: 多数是 -1/0/1 抖动
```

因此：

- 左转直角可以尝试用右编码器退出；
- 右转直角不能依赖左编码器；
- 速度闭环不能启用；
- 后续若要闭环，必须先修左编码器接线、TIM2 配置或方向读取。

## 7. 云台打靶前的冻结点

当底盘达到以下状态，就冻结循迹参数，开始云台最小版本：

```text
连续跑完 2-3 圈
没有长时间 S=5
直线 RAW 大多为 0x18 / 0x1C / 0x38
直角 DIR 符合赛道方向
```

冻结后只允许小范围改：

```c
LINE_CORNER_ENCODER_TARGET
LINE_RECOVER_MS
```

不要再重调 PID 和硬件映射。
