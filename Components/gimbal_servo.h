/**
 * @file gimbal_servo.h
 * @brief 舵机云台驱动适配接口。
 * @layer Components
 *
 * 当前默认不启用云台输出。启用前需要先确认安全 PWM 引脚或外部舵机驱动板。
 */
#ifndef GIMBAL_SERVO_H
#define GIMBAL_SERVO_H

#include "gimbal_if.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取云台驱动接口。
 */
const gimbal_driver_t *GimbalServo_GetDriver(void);

/**
 * @brief 初始化云台并回中。
 */
void GimbalServo_Init(void);

/**
 * @brief 设置云台水平和俯仰角度。
 */
void GimbalServo_SetAngleDeg(float pan_deg, float tilt_deg);

/**
 * @brief 云台回中。
 */
void GimbalServo_Center(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_SERVO_H */
