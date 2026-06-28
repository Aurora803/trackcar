#include "bsp_systick.h"
#include "stm32f10x.h"

static volatile uint32_t g_systick_ms = 0U;

void BSP_SysTick_Init(void)
{
    SystemCoreClockUpdate();
    (void)SysTick_Config(SystemCoreClock / 1000U);
}

void BSP_SysTick_Inc(void)
{
    g_systick_ms++;
}

uint32_t BSP_GetTickMs(void)
{
    return g_systick_ms;
}

void BSP_DelayMs(uint32_t ms)
{
    uint32_t start = BSP_GetTickMs();
    while ((BSP_GetTickMs() - start) < ms)
    {
        ;
    }
}
