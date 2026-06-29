/**
 * @file bsp_pwm.h
 * @brief 电机 PWM 底层接口。
 * @layer BSP
 *
 * 当前电机 PWM 使用 TIM1_CH1/CH2，命令单位为 permille。
 */
#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化电机 PWM 定时器和输出引脚。
 */
void BSP_PWM_MotorInit(void);

/**
 * @brief 设置电机 PWM 占空比。
 * @param channel 当前仅支持 1(TIM1_CH1/右电机) 和 2(TIM1_CH2/左电机)。
 * @param duty_permille 占空比，0~1000。
 */
void BSP_PWM_SetMotorDutyPermille(uint8_t channel, int16_t duty_permille);

/**
 * @brief 获取 TIM1 电机 PWM 的 ARR 周期值。
 */
uint16_t BSP_PWM_GetMotorPeriod(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_PWM_H */
