#include "bsp_gpio.h"
#include "app_config.h"

static void gpio_init_output_pp(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = pin;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &gpio);
}

static void gpio_init_input_pullup(GPIO_TypeDef *port, uint16_t pin)
{
    GPIO_InitTypeDef gpio;
    gpio.GPIO_Pin = pin;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(port, &gpio);
}

void BSP_GPIO_InitAll(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO |
                           RCC_APB2Periph_GPIOA |
                           RCC_APB2Periph_GPIOB |
                           RCC_APB2Periph_GPIOC, ENABLE);

    /* 关闭 JTAG，保留 SWD。当前接线未使用 PB3/PB4，但保留该设置便于后续扩展。 */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* TB6612 方向与待机控制 */
    gpio_init_output_pp(TB6612_STBY_PORT, TB6612_STBY_PIN);
    gpio_init_output_pp(TB6612_AIN1_PORT, TB6612_AIN1_PIN);
    gpio_init_output_pp(TB6612_AIN2_PORT, TB6612_AIN2_PIN);
    gpio_init_output_pp(TB6612_BIN1_PORT, TB6612_BIN1_PIN);
    gpio_init_output_pp(TB6612_BIN2_PORT, TB6612_BIN2_PIN);

    /* 8 路循迹输入：S1-S8
     * PA2/PA3 原本可作为 USART2，但本接线图中已分配给循迹，因此 USART2 不再初始化。
     */
    gpio_init_input_pullup(GPIOA, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5);
    gpio_init_input_pullup(GPIOB, GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11);

    /* PC13 板载 LED，BluePill 常见低电平点亮。 */
    gpio_init_output_pp(GPIOC, GPIO_Pin_13);
    BSP_LED_Set(0U);
}

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

uint8_t BSP_GPIO_Read(gpio_pin_t pin)
{
    return GPIO_ReadInputDataBit(pin.port, pin.pin) ? 1U : 0U;
}

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
