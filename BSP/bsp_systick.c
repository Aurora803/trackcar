/**
 * @file bsp_systick.c
 * @brief SysTick 1ms 系统节拍实现。
 * @layer BSP
 */
#include "bsp_systick.h"
#include "stm32f10x.h"

static volatile uint32_t g_systick_ms = 0U;

/**
 * @brief 配置 SysTick 每 1ms 触发一次中断。
 */
void BSP_SysTick_Init(void)
{
    SystemCoreClockUpdate();
    (void)SysTick_Config(SystemCoreClock / 1000U);
}

/**
 * @brief 毫秒计数自增，应只在 SysTick_Handler 中调用。
 */
void BSP_SysTick_Inc(void)
{
    g_systick_ms++;
}

/**
 * @brief 读取当前毫秒 tick。
 */
uint32_t BSP_GetTickMs(void)
{
    return g_systick_ms;
}

/**
 * @brief 基于毫秒 tick 的忙等延时。
 */
void BSP_DelayMs(uint32_t ms)
{
    uint32_t start = BSP_GetTickMs();
    while ((BSP_GetTickMs() - start) < ms)
    {
        ;
    }
}
