/**
 * @file bsp_gpio.h
 * @brief STM32 GPIO 底层接口。
 * @layer BSP
 *
 * BSP GPIO 只负责引脚初始化和读写，不包含循迹、PID 或模式逻辑。
 */
#ifndef BSP_GPIO_H
#define BSP_GPIO_H

#include "stm32f10x.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* GPIO 端口指针，例如 GPIOA/GPIOB。 */
    GPIO_TypeDef *port;
    /* GPIO_Pin_x 掩码。 */
    uint16_t pin;
} gpio_pin_t;

/**
 * @brief 初始化当前板级使用的全部 GPIO。
 */
void BSP_GPIO_InitAll(void);

/**
 * @brief 写一个 GPIO 输出脚。
 */
void BSP_GPIO_Write(gpio_pin_t pin, uint8_t high);

/**
 * @brief 读取一个 GPIO 输入脚。
 */
uint8_t BSP_GPIO_Read(gpio_pin_t pin);

/**
 * @brief 设置板载 LED。
 */
void BSP_LED_Set(uint8_t on);

/**
 * @brief 翻转板载 LED。
 */
void BSP_LED_Toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_GPIO_H */
