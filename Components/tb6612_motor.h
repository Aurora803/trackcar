/**
 * @file tb6612_motor.h
 * @brief TB6612FNG 电机驱动适配层接口。
 * @layer Components
 *
 * 本模块实现 motor_driver_t，负责 TB6612 的方向、PWM、刹车、空转和 STBY 语义。
 */
#ifndef TB6612_MOTOR_H
#define TB6612_MOTOR_H

#include "motor_if.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取 TB6612 电机驱动接口。
 */
const motor_driver_t *TB6612_GetDriver(void);

/**
 * @brief 初始化 TB6612 相关 PWM 并默认空转停止。
 */
void TB6612_Init(void);

/**
 * @brief 设置单个电机速度，单位 permille，正负表示方向。
 */
void TB6612_SetSpeedPermille(motor_channel_t channel, int16_t speed_permille);

/**
 * @brief TB6612 短刹车。
 */
void TB6612_Brake(motor_channel_t channel);

/**
 * @brief TB6612 空转停止。
 */
void TB6612_Coast(motor_channel_t channel);

/**
 * @brief 控制 STBY 待机。
 * @note 当前接线 STBY 固定 3.3V 时，该函数为空操作。
 */
void TB6612_Standby(uint8_t enable_standby);

#ifdef __cplusplus
}
#endif

#endif /* TB6612_MOTOR_H */
