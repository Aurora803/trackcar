/**
 * @file bsp_servo.h
 * @brief 舵机 PWM 底层预留接口。
 * @layer BSP
 *
 * 当前默认不启用云台舵机 PWM。启用前必须重新确认可用定时器和引脚。
 */
#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化舵机 PWM 输出。
 */
void BSP_Servo_Init(void);

/**
 * @brief 设置舵机角度。
 * @param channel 预留通道号。
 * @param angle_deg 目标角度，单位 degree。
 */
void BSP_Servo_SetAngleDeg(uint8_t channel, float angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SERVO_H */
