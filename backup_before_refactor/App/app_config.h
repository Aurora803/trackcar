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
#define APP_VISION_PERIOD_MS           20U

/* ===================== 串口参数 ===================== */
#define DEBUG_UART_BAUDRATE            115200U
#define VISION_UART_BAUDRATE           115200U

/* ===================== 电机与驱动参数 ===================== */
#define MOTOR_PWM_FREQ_HZ              20000U
#define MOTOR_PWM_MAX_PERMILLE         1000
#define MOTOR_LEFT_INVERT              0
#define MOTOR_RIGHT_INVERT             0

/* TB6612 引脚：按用户接线图更新 */
#define TB6612_STBY_PORT               GPIOA
#define TB6612_STBY_PIN                GPIO_Pin_8

#define TB6612_AIN1_PORT               GPIOB
#define TB6612_AIN1_PIN                GPIO_Pin_12
#define TB6612_AIN2_PORT               GPIOB
#define TB6612_AIN2_PIN                GPIO_Pin_13
#define TB6612_BIN1_PORT               GPIOB
#define TB6612_BIN1_PIN                GPIO_Pin_14
#define TB6612_BIN2_PORT               GPIOB
#define TB6612_BIN2_PIN                GPIO_Pin_15

/* PWM: TIM3_CH3 PB0 -> PWMA, TIM3_CH4 PB1 -> PWMB */
#define MOTOR_PWM_TIMER                TIM3
#define MOTOR_LEFT_PWM_CHANNEL         3U
#define MOTOR_RIGHT_PWM_CHANNEL        4U

/* ===================== 编码器参数 ===================== */
#define ENCODER_LEFT_TIMER             TIM2
#define ENCODER_RIGHT_TIMER            TIM4
#define ENCODER_LEFT_INVERT            0
#define ENCODER_RIGHT_INVERT           0

/* 这里填真实电机编码器每圈计数。不同 JGB37520 减速比/编码器规格不同，先用于后续速度闭环。 */
#define ENCODER_COUNTS_PER_WHEEL_REV   1560.0f

/* ===================== 8 路循迹参数 ===================== */
/* 1：黑线输出低电平；0：黑线输出高电平。按你的模块实际输出修改。 */
#define TRACKER_BLACK_ACTIVE_LOW       1
#define TRACKER_SENSOR_COUNT           8U
#define TRACKER_LOST_SEARCH_PWM        260

/* 权重单位越大，转向响应越强。左负右正。 */
#define TRACKER_WEIGHT_0               (-3500)
#define TRACKER_WEIGHT_1               (-2500)
#define TRACKER_WEIGHT_2               (-1500)
#define TRACKER_WEIGHT_3               (-500)
#define TRACKER_WEIGHT_4               (500)
#define TRACKER_WEIGHT_5               (1500)
#define TRACKER_WEIGHT_6               (2500)
#define TRACKER_WEIGHT_7               (3500)

/* ===================== 循迹 PID 初值 ===================== */
#define LINE_BASE_PWM                  360
#define LINE_PWM_LIMIT                 750
#define LINE_PID_KP                    0.18f
#define LINE_PID_KI                    0.00f
#define LINE_PID_KD                    0.035f
#define LINE_PID_OUT_LIMIT             380.0f
#define LINE_PID_INTEGRAL_LIMIT        1000.0f

/* ===================== 视觉/云台预留 ===================== */
#define APP_ENABLE_VISION_TARGET       0
#define APP_ENABLE_GIMBAL_SERVO        0

/* 新接线图中 PA2/PA3 已用于 8 路循迹 S1/S2，因此视觉通信预留到 USART1 PA9/PA10。
 * 注意：USART1 同时用于调试打印和视觉接收时，不要同时接 USB-TTL 与 OpenMV/RPi，
 * 否则 TX/RX 线上会互相影响。正式视觉模式建议减少调试打印。
 */
#define VISION_UART_USE_USART1         1

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
