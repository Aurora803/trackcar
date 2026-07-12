/**
 * @file app_config.h
 * @brief 全工程集中配置入口。
 * @layer App
 *
 * 本文件集中保存控制周期、串口波特率、电机 PWM 范围、编码器方向、
 * 8 路循迹权重、矩形赛道状态机参数、视觉/云台预留开关等配置。
 * 虽然物理位置在 App 目录，但它当前实际承担跨层 Config 角色；
 * BSP/Components include 本文件只读取宏配置，不代表反向调用 App 业务逻辑。
 *
 * 移植或调参时优先改这里；不确定的硬件方向、电平和通道不要在 BSP
 * 或算法代码里硬改，应先通过这里的宏做小范围验证。
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "stm32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 系统参数 ===================== */
#define SYS_CORE_CLOCK_HZ              72000000UL
/* 主控制周期，单位 ms。当前循迹状态机、编码器采样与未来速度闭环都按此周期调度。 */
#define APP_CONTROL_PERIOD_MS          10U
/* 控制调度间隔达到该值时计为一次明显超时，并通过遥测 OV 字段报告。 */
#define APP_CONTROL_OVERRUN_WARN_MS    20U
/* 串口遥测周期，单位 ms。printf 使用中断 TX 队列；周期过短仍可能填满队列并丢字符。 */
#define APP_TELEMETRY_PERIOD_MS        200U
/* 1: 每次状态真正变化时通过非阻塞调试串口输出一条 EV 诊断日志。 */
#define LINE_ENABLE_TRANSITION_TRACE   1

/* 1: boot into wheel speed test demo. Lift the car before flashing/running. */
#define APP_ENABLE_MOTOR_SPEED_TEST_DEMO 0
#define MOTOR_TEST_PWM                 260
#define MOTOR_TEST_START_DELAY_MS      2000U
#define MOTOR_TEST_RUN_MS              3000U
#define MOTOR_TEST_STOP_MS             1000U

/* ===================== 串口参数 ===================== */
/* 当前调试口实际为 USART2(PA2/PA3)，保留 DEBUG 命名避免上层关心具体串口号。 */
#define DEBUG_UART_BAUDRATE            9600U
/* 1：使用同一 USART2 蓝牙链路接收 START/STOP；启用后默认上电停车。 */
#define APP_ENABLE_BLUETOOTH_CONTROL   1
/* 视觉协议预留波特率；当前未分配独立视觉串口。 */
#define VISION_UART_BAUDRATE           115200U

/* ===================== 电机与驱动参数 ===================== */
/* TIM1 电机 PWM 频率。更改前需确认 TB6612、电机噪声和低速扭矩表现。 */
#define MOTOR_PWM_FREQ_HZ              1000U
/* TB6612 PWM 命令上限，单位 permille：1000 表示 100%，不是 0~100 或 0~255。 */
#define MOTOR_PWM_MAX_PERMILLE         1000
/* 底盘层统一安全限幅，所有上层控制输出最终都会被限制到该范围内。不要用它调 PID。 */
#define CHASSIS_PWM_LIMIT              750
/* 左电机方向软件取反。按接线图方向表：左电机前进需要 AIN1=0/AIN2=1。 */
#define MOTOR_LEFT_INVERT              1
/* 右电机方向软件取反。当前接线下右电机前进保持 TB6612 默认方向。 */
#define MOTOR_RIGHT_INVERT             0

/* TB6612 引脚：按最新接线图更新。STBY 直接接 3.3V，不再由 STM32 GPIO 控制。 */
#define TB6612_STBY_CONTROL_BY_GPIO    0

#define TB6612_AIN1_PORT               GPIOB
#define TB6612_AIN1_PIN                GPIO_Pin_12
#define TB6612_AIN2_PORT               GPIOB
#define TB6612_AIN2_PIN                GPIO_Pin_13
#define TB6612_BIN1_PORT               GPIOB
#define TB6612_BIN1_PIN                GPIO_Pin_14
#define TB6612_BIN2_PORT               GPIOB
#define TB6612_BIN2_PIN                GPIO_Pin_15

/* PWM: TIM1_CH2 PA9 -> PWMA(左电机)，TIM1_CH1 PA8 -> PWMB(右电机)。不要与 USART1/云台 PWM 复用。 */
#define MOTOR_PWM_TIMER                TIM1
#define MOTOR_LEFT_PWM_CHANNEL         2U
#define MOTOR_RIGHT_PWM_CHANNEL        1U

/* ===================== 编码器参数 ===================== */
/* 当前编码器使用 TIM2/TIM4 正交编码模式。方向是否正确需要架空车轮实测确认。 */
#define ENCODER_LEFT_TIMER             TIM2
#define ENCODER_RIGHT_TIMER            TIM4
/* 左编码器计数方向取反开关；当前保持硬件读数原方向。 */
#define ENCODER_LEFT_INVERT            0
/* 右编码器计数方向取反开关；接线图标注右编码器方向需要软件取反。 */
#define ENCODER_RIGHT_INVERT           1

/* 这里填真实电机编码器每圈计数。不同 JGB37520 减速比/编码器规格不同，先用于后续速度闭环。 */
#define ENCODER_COUNTS_PER_WHEEL_REV   1560.0f

/* ===================== 8 路循迹参数 ===================== */
/* 1：黑线输出低电平；0：黑线输出高电平。只影响读取转换，不改变 GPIO 上拉配置。 */
#define TRACKER_BLACK_ACTIVE_LOW       1
/* 循迹传感器数量。当前权重、raw_bits 和直角识别都按 X1~X8 八路设计。 */
#define TRACKER_SENSOR_COUNT           8U
/* 兼容旧调参入口：当前丢线搜索实际使用 LINE_BLIND_* 参数。 */
#define TRACKER_LOST_SEARCH_PWM        260

/* ===================== 矩形赛道状态机参数 ===================== */
/* 1 表示启用矩形赛道直角弯专用识别；0 时只做普通循迹/丢线恢复。 */
#define APP_RECTANGLE_TRACK_ONLY       1

/* 闭合矩形线一般每个角都按同一方向转。1：默认右转/顺时针；-1：默认左转/逆时针。
 * 如果传感器能明确看到左/右侧直角，代码会优先采用传感器判断；
 * 如果出现 6 路以上同时触发这种“横线/宽黑线”特征，则按这里的默认方向转。
 */
#define RECT_DEFAULT_CORNER_DIR        (-1)
/* 0：不把 6 路以上同时触发直接当作直角弯，避免宽黑线/反光误触发。
 * 矩形调稳后，如果你的赛道直角处经常是整排横线，可再改成 1。
 */
#define RECT_ENABLE_CROSS_CORNER       1

/* START 状态上电稳定时间。 */
#define LINE_START_STABLE_MS           200U
/* FOLLOW 中连续丢线超过该时间后进入 BLIND。 */
#define LINE_BLIND_ENTER_MS            20U
/* BLIND/LOST 中连续重新看到线达到该时间后进入 RECOVER。 */
#define LINE_BLIND_REACQUIRE_MS        40U
/* BLIND 搜索超时后进入 LOST 停车等待。 */
#define LINE_BLIND_TIMEOUT_MS          2000U
/* RECOVER 低速平滑恢复时间。 */
#define LINE_RECOVER_MS                320U
/* CORNER 最短保持时间，避免刚进入直角就被中心压线误判退出。 */
#define LINE_CORNER_MIN_MS             260U
/* CORNER 超时后退回 BLIND 找线。 */
#define LINE_CORNER_TIMEOUT_MS         950U
/* 当前右编码器可用于固定左转退出，左编码器仍不稳定，暂不用于速度闭环。
 * 若右编码器失效，可临时改为 0 使用固定时间退出。
 */
/* 1：直角弯优先按编码器累计退出；0：编码器未确认前按固定时间退出。 */
#define LINE_CORNER_USE_ENCODER        1
/* LINE_CORNER_USE_ENCODER=0 时的直角弯定时退出时间，单位 ms，需要实车低速微调。 */
#define LINE_CORNER_TIME_MS            520U
#define LINE_CORNER_ENCODER_TARGET     495
#define LINE_CORNER_CENTER_ENABLE_ENCODER 500  /* 使用编码器退出时：至少转过这段计数后，才允许中心压线结束转角。 */
#define LINE_CORNER_CENTER_SEARCH_MS   220U
#define LINE_CORNER_DEBOUNCE_COUNT     6U  /* 连续检测到同向直角特征后才切入转角状态。 */
/* 编码器/中心线出口也要求连续多帧成立，避免单帧噪声提前退出直角。 */
#define LINE_CORNER_EXIT_CONFIRM_MS    30U
#define LINE_CORNER_REARM_MS           700U
#define LINE_CORNER_REARM_CENTER_MS    200U

/* 传感器健康诊断：全触发通常表示短路/粘低，全未触发过久可能表示断线。 */
#define TRACKER_ALL_ACTIVE_FAULT_MS    300U
#define TRACKER_ALL_INACTIVE_FAULT_MS  2500U
#define TRACKER_FAULT_CLEAR_MS         200U

#define LINE_RECOVER_LOST_CONFIRM_MS 30U
#define LINE_RECOVER_LOST_PWM        170

#define LINE_RECOVER_CENTER_CONFIRM_MS  250U
#define LINE_RECOVER_CENTER_ERROR_MAX    200

#define LINE_CORNER_ERROR_THRESHOLD    850
#define LINE_RECOVER_ERROR_THRESHOLD   650

/* 循迹基础 PWM：误差越小使用越快的档位，误差越大自动降速。 */
#define LINE_BASE_PWM_FAST             240
#define LINE_BASE_PWM_MID              220
#define LINE_BASE_PWM_SLOW             200
/* 转弯恢复、丢线搜索、直角弯专用 PWM。方向和实际速度需要实车低速验证。 */
#define LINE_RECOVER_PWM               200
#define LINE_RECOVER_CORRECTION_LIMIT  40
#define LINE_BLIND_BASE_PWM            90
#define LINE_BLIND_TURN_PWM            220
#define LINE_CORNER_INNER_PWM          60
#define LINE_CORNER_OUTER_PWM          280
#define LINE_CORNER_ALIGN_INNER_PWM    120
#define LINE_CORNER_ALIGN_OUTER_PWM    180

/* 权重单位越大，转向响应越强。左负右正。 */
#define TRACKER_WEIGHT_0               (-1200)
#define TRACKER_WEIGHT_1               (-800)
#define TRACKER_WEIGHT_2               (-400)
#define TRACKER_WEIGHT_3               (-100)
#define TRACKER_WEIGHT_4               (100)
#define TRACKER_WEIGHT_5               (400)
#define TRACKER_WEIGHT_6               (800)
#define TRACKER_WEIGHT_7               (1200)

/* ===================== 循迹 PID 初值 ===================== */
#define LINE_BASE_PWM                  LINE_BASE_PWM_MID
/* 循迹应用层输出限幅。底盘层还会用 CHASSIS_PWM_LIMIT 做最终安全限幅。
 * 这里单独收紧循迹输出，避免出弯恢复时产生过大的单轮 PWM。
 */
#define LINE_PWM_LIMIT                 380
/* PID 输入是循迹位置误差，输出是左右轮差速修正量。KP 决定纠偏力度。 */
#define LINE_PID_KP                    0.16f
/* 当前保持 KI=0，且 KD=0，因此实际为纯 P 控制。 */
#define LINE_PID_KI                    0.00f
/* KD 抑制蛇形摆动；过大可能放大传感器抖动。 */
#define LINE_PID_KD                    0.000f
/* PID 输出和积分项限幅，避免丢线/大误差时积分或差速过大。 */
#define LINE_PID_OUT_LIMIT             160.0f
#define LINE_PID_INTEGRAL_LIMIT        800.0f

/* ===================== 视觉/云台预留 ===================== */
/* 视觉目标跟踪应用开关。本轮保持 0，不调度视觉协议和云台目标控制。 */
#define APP_ENABLE_VISION_TARGET       0
/* 云台舵机 PWM 开关。本轮保持 0；启用前必须重新规划不冲突的 PWM 引脚。 */
#define APP_ENABLE_GIMBAL_SERVO        0
/* 当前 PA2/PA3 用作 USART2 调试串口；本版不启用视觉通信。 */
#define VISION_UART_USE_USART1         0

/* 云台角度预留参数。当前 APP_ENABLE_GIMBAL_SERVO=0，不会输出舵机 PWM。 */
#define GIMBAL_PAN_CENTER_DEG          90.0f
#define GIMBAL_TILT_CENTER_DEG         90.0f
#define GIMBAL_PAN_MIN_DEG             20.0f
#define GIMBAL_PAN_MAX_DEG             160.0f
#define GIMBAL_TILT_MIN_DEG            30.0f
#define GIMBAL_TILT_MAX_DEG            150.0f

#ifdef __cplusplus
}
#endif

#endif /* APP_CONFIG_H */
