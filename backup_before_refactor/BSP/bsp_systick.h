#ifndef BSP_SYSTICK_H
#define BSP_SYSTICK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_SysTick_Init(void);
void BSP_SysTick_Inc(void);
uint32_t BSP_GetTickMs(void);
void BSP_DelayMs(uint32_t ms);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SYSTICK_H */
