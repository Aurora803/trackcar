/**
 * @file chassis.h
 * @brief 底盘抽象层接口。
 * @layer Components
 *
 * Chassis 负责把左右轮 PWM 命令、安全限幅、停车和编码器增量封装起来。
 * 上层不直接调用具体电机驱动芯片。
 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>
#include "motor_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* 最近一次下发的左右轮 PWM 命令，单位 permille。 */
    int16_t left_pwm;
    int16_t right_pwm;
    /* 最近一次控制周期读取到的左右编码器增量。 */
    int16_t left_encoder_delta;
    int16_t right_encoder_delta;
} chassis_state_t;

/**
 * @brief 初始化底盘并绑定电机驱动。
 */
void Chassis_Init(const motor_driver_t *motor_driver);

/**
 * @brief 设置左右轮 PWM 命令。
 */
void Chassis_SetPWM(int16_t left_pwm, int16_t right_pwm);

/**
 * @brief 空转停止左右轮。
 */
void Chassis_StopCoast(void);

/**
 * @brief 刹车停止左右轮。
 */
void Chassis_StopBrake(void);

/**
 * @brief 读取并刷新编码器增量，通常与控制周期同步调用。
 */
void Chassis_UpdateEncoder(void);

/**
 * @brief 获取底盘状态快照。
 */
chassis_state_t Chassis_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H */
