/**
 * @file bsp_encoder.h
 * @brief 编码器定时器底层接口。
 * @layer BSP
 *
 * 当前使用 TIM2/TIM4 正交编码器模式，只提供增量读取，不参与速度 PID 运算。
 */
#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化左右编码器 GPIO 和定时器。
 */
void BSP_Encoder_Init(void);

/**
 * @brief 读取并清零左编码器增量。
 */
int16_t BSP_Encoder_ReadLeftDelta(void);

/**
 * @brief 读取并清零右编码器增量。
 */
int16_t BSP_Encoder_ReadRightDelta(void);

/**
 * @brief 清零左右编码器计数器。
 */
void BSP_Encoder_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ENCODER_H */
