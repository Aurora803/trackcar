/**
 * @file bsp_gpio.c
 * @brief 当前 STM32F103C8T6 接线的公共 GPIO 初始化和读写。
 * @layer BSP
 *
 * 本文件只负责全局 GPIO/板级公共初始化和通用读写。具体外设 GPIO
 * 应由对应模块 Init 负责，例如 TB6612 方向脚、循迹输入、PWM、编码器和 USART。
 */
#include "bsp_gpio.h"

/**
 * @brief 初始化推挽输出 GPIO。
 */
static void gpio_init_output_pp(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = pin;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &gpio);
}

/**
 * @brief 初始化全局 GPIO / 板级公共配置。
 *
 * 本函数不再负责所有外设 GPIO。TB6612 方向脚由 TB6612_Init() 初始化，
 * 8 路循迹输入由 YahboomTracker8IO_Init() 初始化，PWM/编码器/USART 也由
 * 各自 BSP Init 负责。这里仅保留 AFIO/SWD 公共配置和 PC13 板载 LED。
 */
void BSP_GPIO_InitAll(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);

    /* 关闭 JTAG，保留 SWD。当前接线未使用 PB3/PB4，但保留该设置便于后续扩展。 */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* PC13 板载 LED，BluePill 常见低电平点亮。 */
    gpio_init_output_pp(GPIOC, GPIO_Pin_13);
    BSP_LED_Set(0U);
}

/**
 * @brief 写 GPIO 输出。
 */
void BSP_GPIO_Write(gpio_pin_t pin, uint8_t high)
{
    if (high)
    {
        GPIO_SetBits(pin.port, pin.pin);
    }
    else
    {
        GPIO_ResetBits(pin.port, pin.pin);
    }
}

/**
 * @brief 读 GPIO 输入。
 */
uint8_t BSP_GPIO_Read(gpio_pin_t pin)
{
    return GPIO_ReadInputDataBit(pin.port, pin.pin) ? 1U : 0U;
}

/**
 * @brief 控制 PC13 板载 LED。
 * @note BluePill 常见 PC13 低电平点亮。
 */
void BSP_LED_Set(uint8_t on)
{
    if (on)
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    }
    else
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
    }
}

/**
 * @brief 翻转 PC13 板载 LED。
 */
void BSP_LED_Toggle(void)
{
    if (GPIO_ReadOutputDataBit(GPIOC, GPIO_Pin_13))
    {
        GPIO_ResetBits(GPIOC, GPIO_Pin_13);
    }
    else
    {
        GPIO_SetBits(GPIOC, GPIO_Pin_13);
    }
}
