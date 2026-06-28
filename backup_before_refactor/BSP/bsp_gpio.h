#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "stm32f10x.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
} gpio_pin_t;

void BSP_GPIO_InitAll(void);
void BSP_GPIO_Write(gpio_pin_t pin, uint8_t high);
uint8_t BSP_GPIO_Read(gpio_pin_t pin);
void BSP_LED_Set(uint8_t on);
void BSP_LED_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_GPIO_H */
