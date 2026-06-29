/**
 * @file bsp_systick.h
 * @brief SysTick 毫秒节拍接口。
 * @layer BSP
 *
 * SysTick 中断只递增毫秒计数，控制任务在主循环中按 tick 差值调度。
 */
#ifndef BSP_SYSTICK_H
#define BSP_SYSTICK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 SysTick 为 1ms 中断。
 */
void BSP_SysTick_Init(void);

/**
 * @brief SysTick_Handler 中调用，递增系统毫秒计数。
 */
void BSP_SysTick_Inc(void);

/**
 * @brief 获取系统启动后的毫秒计数。
 */
uint32_t BSP_GetTickMs(void);

/**
 * @brief 简单阻塞延时。
 * @note 控制主循环中不建议使用。
 */
void BSP_DelayMs(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SYSTICK_H */
