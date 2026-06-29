/**
 * @file bsp_uart.c
 * @brief USART2 调试串口和预留视觉串口接口。
 * @layer BSP
 *
 * 当前 PA2/PA3 用作 USART2 调试口。printf/fputc 采用阻塞发送，适合调试，
 * 不适合作为高频实时日志。USART2 接收中断只把字节放入环形缓冲。
 */
#include "bsp_uart.h"
#include "app_config.h"
#include "stm32f10x.h"
#include <stdio.h>

#define UART2_RX_BUFFER_SIZE 128U

static volatile char g_uart2_rx_buffer[UART2_RX_BUFFER_SIZE];
static volatile uint16_t g_uart2_rx_head = 0U;
static volatile uint16_t g_uart2_rx_tail = 0U;

#if VISION_UART_USE_USART1
#error "VISION_UART_USE_USART1 is reserved: USART1_TX PA9 conflicts with TIM1_CH2 left motor PWM on current hardware."
#endif

/**
 * @brief 初始化 USART2 TX/RX GPIO。
 */
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

/**
 * @brief 初始化指定 USART 外设。
 */
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

/**
 * @brief 初始化 USART2 调试串口和 RXNE 中断。
 */
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

/**
 * @brief 阻塞发送一个字符。
 * @note 高频调用会占用主循环时间，控制周期敏感时应降低输出频率。
 */
void BSP_DebugUART_SendChar(char ch)
{
    while (USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
    {
        ;
    }
    USART_SendData(USART2, (uint16_t)ch);
}

/**
 * @brief 阻塞发送字符串。
 */
void BSP_DebugUART_SendString(const char *str)
{
    if (str == 0) return;
    while (*str != '\0')
    {
        BSP_DebugUART_SendChar(*str++);
    }
}

/**
 * @brief 发送 name=value 调试行。
 */
void BSP_DebugUART_SendInt(const char *name, int32_t value)
{
    char buf[48];
    (void)snprintf(buf, sizeof(buf), "%s=%ld\r\n", name, (long)value);
    BSP_DebugUART_SendString(buf);
}

/**
 * @brief 兼容旧 UART1 命名的字符发送接口。
 */
void BSP_UART1_SendChar(char ch)
{
    /* 兼容旧接口名：当前调试串口实际是 USART2。新代码优先使用 BSP_DebugUART_*。 */
    BSP_DebugUART_SendChar(ch);
}

/**
 * @brief 兼容旧 UART1 命名的字符串发送接口。
 */
void BSP_UART1_SendString(const char *str)
{
    BSP_DebugUART_SendString(str);
}

/**
 * @brief 兼容旧 UART1 命名的整数调试接口。
 */
void BSP_UART1_SendInt(const char *name, int32_t value)
{
    BSP_DebugUART_SendInt(name, value);
}

/**
 * @brief 视觉串口发送字符预留接口。
 */
void BSP_VisionUART_SendChar(char ch)
{
    /* 本版不启用独立视觉串口，接口临时复用 USART2 调试通道。 */
    BSP_DebugUART_SendChar(ch);
}

/**
 * @brief 视觉串口发送字符串预留接口。
 */
void BSP_VisionUART_SendString(const char *str)
{
    BSP_DebugUART_SendString(str);
}

/**
 * @brief 从 USART2 环形缓冲非阻塞读取一个字节。
 */
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

/**
 * @brief USART2 RXNE 中断处理。
 *
 * 中断内只读接收寄存器并写入环形缓冲，不做 printf、协议解析或控制逻辑。
 */
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
/**
 * @brief GCC newlib 的 write 重定向到调试串口。
 */
int _write(int file, char *ptr, int len)
{
    int i;
    (void)file;
    for (i = 0; i < len; ++i)
    {
        BSP_DebugUART_SendChar(ptr[i]);
    }
    return len;
}
#endif

/**
 * @brief printf/fputc 重定向到调试串口。
 */
int fputc(int ch, FILE *f)
{
    (void)f;
    BSP_DebugUART_SendChar((char)ch);
    return ch;
}
