#include "bsp_uart.h"
#include "app_config.h"
#include "stm32f10x.h"
#include <stdio.h>

#define UART2_RX_BUFFER_SIZE 128U

static volatile char g_uart2_rx_buffer[UART2_RX_BUFFER_SIZE];
static volatile uint16_t g_uart2_rx_head = 0U;
static volatile uint16_t g_uart2_rx_tail = 0U;

static void uart_gpio_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    /* USART2 调试串口：PA2 TX, PA3 RX。 */
    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_3;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &gpio);
}

static void uart_init_one(USART_TypeDef *uart, uint32_t baudrate)
{
    USART_InitTypeDef usart;

    usart.USART_BaudRate = baudrate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_No;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    usart.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(uart, &usart);
    USART_Cmd(uart, ENABLE);
}

void BSP_UART_Init(void)
{
    NVIC_InitTypeDef nvic;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    uart_gpio_init();
    uart_init_one(USART2, DEBUG_UART_BAUDRATE);

    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

    nvic.NVIC_IRQChannel = USART2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 2U;
    nvic.NVIC_IRQChannelSubPriority = 2U;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
}

void BSP_UART1_SendChar(char ch)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
    {
        ;
    }
    USART_SendData(USART2, (uint16_t)ch);
}

void BSP_UART1_SendString(const char *str)
{
    if (str == 0) return;
    while (*str != '\0')
    {
        BSP_UART1_SendChar(*str++);
    }
}

void BSP_UART1_SendInt(const char *name, int32_t value)
{
    char buf[48];
    (void)snprintf(buf, sizeof(buf), "%s=%ld\r\n", name, (long)value);
    BSP_UART1_SendString(buf);
}

void BSP_VisionUART_SendChar(char ch)
{
    /* 本版不启用视觉通信，保留接口用于后续移植。 */
    BSP_UART1_SendChar(ch);
}

void BSP_VisionUART_SendString(const char *str)
{
    BSP_UART1_SendString(str);
}

int BSP_VisionUART_ReadCharNonBlocking(char *out_ch)
{
    if (out_ch == 0) return 0;

    if (g_uart2_rx_tail == g_uart2_rx_head)
    {
        return 0;
    }

    *out_ch = g_uart2_rx_buffer[g_uart2_rx_tail];
    g_uart2_rx_tail = (uint16_t)((g_uart2_rx_tail + 1U) % UART2_RX_BUFFER_SIZE);
    return 1;
}

void BSP_UART2_IRQHandler(void)
{
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        char ch = (char)USART_ReceiveData(USART2);
        uint16_t next = (uint16_t)((g_uart2_rx_head + 1U) % UART2_RX_BUFFER_SIZE);
        if (next != g_uart2_rx_tail)
        {
            g_uart2_rx_buffer[g_uart2_rx_head] = ch;
            g_uart2_rx_head = next;
        }
        USART_ClearITPendingBit(USART2, USART_IT_RXNE);
    }
}

#if defined(__GNUC__)
int _write(int file, char *ptr, int len)
{
    int i;
    (void)file;
    for (i = 0; i < len; ++i)
    {
        BSP_UART1_SendChar(ptr[i]);
    }
    return len;
}
#endif
