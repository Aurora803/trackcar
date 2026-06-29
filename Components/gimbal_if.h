/**
 * @file gimbal_if.h
 * @brief 二维云台驱动抽象接口。
 * @layer Components
 *
 * 视觉控制只依赖角度接口，不直接依赖舵机 PWM 或 PCA9685 等具体实现。
 */
#ifndef GIMBAL_IF_H
#define GIMBAL_IF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* 初始化云台驱动。 */
    void (*init)(void);
    /* 设置水平/俯仰角度，单位 degree。 */
    void (*set_angle_deg)(float pan_deg, float tilt_deg);
    /* 回中。 */
    void (*center)(void);
} gimbal_driver_t;

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_IF_H */
