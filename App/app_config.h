#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "stm32f10x.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===================== 系统参数 ===================== */
#define SYS_CORE_CLOCK_HZ              72000000UL
#define APP_CONTROL_PERIOD_MS          10U
#define APP_TELEMETRY_PERIOD_MS        50U

/* ===================== 串口参数 ===================== */
#define DEBUG_UART_BAUDRATE            115200U
#define VISION_UART_BAUDRATE           115200U

/* ===================== 电机与驱动参数 ===================== */
#define MOTOR_PWM_FREQ_HZ              1000U
#define MOTOR_PWM_MAX_PERMILLE         1000
#define CHASSIS_PWM_LIMIT              750
#define MOTOR_LEFT_INVERT              1  /* 按接线图方向表：左电机前进 AIN1=0/AIN2=1 */
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

/* PWM: TIM1_CH2 PA9 -> PWMA(左电机)，TIM1_CH1 PA8 -> PWMB(右电机) */
#define MOTOR_PWM_TIMER                TIM1
#define MOTOR_LEFT_PWM_CHANNEL         2U
#define MOTOR_RIGHT_PWM_CHANNEL        1U

/* ===================== 编码器参数 ===================== */
#define ENCODER_LEFT_TIMER             TIM2
#define ENCODER_RIGHT_TIMER            TIM4
#define ENCODER_LEFT_INVERT            0
#define ENCODER_RIGHT_INVERT           1  /* 接线图标注右编码器方向软件取反 */

/* 这里填真实电机编码器每圈计数。不同 JGB37520 减速比/编码器规格不同，先用于后续速度闭环。 */
#define ENCODER_COUNTS_PER_WHEEL_REV   1560.0f

/* ===================== 8 路循迹参数 ===================== */
/* 1：黑线输出低电平；0：黑线输出高电平。按你的模块实际输出修改。 */
#define TRACKER_BLACK_ACTIVE_LOW       1
#define TRACKER_SENSOR_COUNT           8U
#define TRACKER_LOST_SEARCH_PWM        260

/* ===================== 矩形赛道状态机参数 ===================== */
#define APP_RECTANGLE_TRACK_ONLY       1

/* 闭合矩形线一般每个角都按同一方向转。1：默认右转/顺时针；-1：默认左转/逆时针。
 * 如果传感器能明确看到左/右侧直角，代码会优先采用传感器判断；
 * 如果出现 6 路以上同时触发这种“横线/宽黑线”特征，则按这里的默认方向转。
 */
#define RECT_DEFAULT_CORNER_DIR        1
/* 0：不把 6 路以上同时触发直接当作直角弯，避免宽黑线/反光误触发。
 * 矩形调稳后，如果你的赛道直角处经常是整排横线，可再改成 1。
 */
#define RECT_ENABLE_CROSS_CORNER       0

#define LINE_START_STABLE_MS           200U
#define LINE_BLIND_ENTER_MS            20U
#define LINE_BLIND_REACQUIRE_MS        40U
#define LINE_BLIND_TIMEOUT_MS          600U
#define LINE_RECOVER_MS                160U
#define LINE_CORNER_MIN_MS             180U
#define LINE_CORNER_TIMEOUT_MS         700U
/* 当前你的串口 LE/RE/SUM 一直为 0，说明编码器反馈未通。
 * 先用定时直角弯跑通矩形；后续编码器修好后改成 1。
 */
#define LINE_CORNER_USE_ENCODER        0
#define LINE_CORNER_TIME_MS            330U
#define LINE_CORNER_ENCODER_TARGET     550
#define LINE_CORNER_CENTER_ENABLE_ENCODER 420  /* 使用编码器退出时：至少转过这段计数后，才允许中心压线结束转角。 */
#define LINE_CORNER_DEBOUNCE_COUNT     2U  /* 连续检测到同向直角特征后才切入转角状态。 */

#define LINE_CORNER_ERROR_THRESHOLD    850
#define LINE_RECOVER_ERROR_THRESHOLD   450

#define LINE_BASE_PWM_FAST             300
#define LINE_BASE_PWM_MID              260
#define LINE_BASE_PWM_SLOW             220
#define LINE_RECOVER_PWM               220
#define LINE_BLIND_BASE_PWM            90
#define LINE_BLIND_TURN_PWM            220
#define LINE_CORNER_INNER_PWM          80
#define LINE_CORNER_OUTER_PWM          300

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
#define LINE_PWM_LIMIT                 CHASSIS_PWM_LIMIT
#define LINE_PID_KP                    0.16f
#define LINE_PID_KI                    0.00f
#define LINE_PID_KD                    0.004f
#define LINE_PID_OUT_LIMIT             220.0f
#define LINE_PID_INTEGRAL_LIMIT        800.0f

/* ===================== 视觉/云台预留 ===================== */
/* 本版本先不启用视觉，只保留源文件和协议文档，主循环不调度视觉任务。 */
#define APP_ENABLE_VISION_TARGET       0
#define APP_ENABLE_GIMBAL_SERVO        0
/* 当前 PA2/PA3 用作 USART2 调试串口；本版不启用视觉通信。 */
#define VISION_UART_USE_USART1         0

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
