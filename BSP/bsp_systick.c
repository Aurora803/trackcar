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
    /* Cortex-M3 对对齐的 32 位读写是原子的；tick 回绕交由调用方使用无符号
     * 减法处理，因此此处不需要为了读取而关闭中断。
     */
    return g_systick_ms;
}

/**
 * @brief 基于毫秒 tick 的忙等延时。
 */
void BSP_DelayMs(uint32_t ms)
{
    /* 仅适合上电自检等阻塞场景。若在主控制循环调用，会停止串口命令、
     * 编码器采样和循迹状态机调度。
     */
    uint32_t start = BSP_GetTickMs();
    while ((BSP_GetTickMs() - start) < ms)
    {
        ;
    }
}
