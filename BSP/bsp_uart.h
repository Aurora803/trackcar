/**
 * @file bsp_uart.h
 * @brief 串口底层接口。
 * @layer BSP
 *
 * 当前调试串口为 USART2(PA2/PA3)。视觉通信接口为后续预留，本版本临时复用
 * 同一接收缓冲，不代表调试和视觉已有独立硬件串口。
 */
#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 USART2 调试串口和 RX 中断。
 */
void BSP_UART_Init(void);

/**
 * @brief 阻塞发送一个调试字符。
 */
void BSP_DebugUART_SendChar(char ch);

/**
 * @brief 阻塞发送调试字符串。
 */
void BSP_DebugUART_SendString(const char *str);

/**
 * @brief 发送 name=value 调试行。
 */
void BSP_DebugUART_SendInt(const char *name, int32_t value);

/* 兼容旧命名：历史上函数名保留了 UART1，但当前硬件实际使用 USART2(PA2/PA3)。 */
void BSP_UART1_SendChar(char ch);
void BSP_UART1_SendString(const char *str);
void BSP_UART1_SendInt(const char *name, int32_t value);

/* 视觉通信抽象接口。本版先不启用视觉，接口临时复用 USART2 调试串口。 */
void BSP_VisionUART_SendChar(char ch);
void BSP_VisionUART_SendString(const char *str);
/**
 * @brief 非阻塞读取一个视觉/USART2 接收缓冲字符。
 */
int BSP_VisionUART_ReadCharNonBlocking(char *out_ch);

/* 当前启用 USART2 中断，PA2/PA3 用作调试串口。 */
/**
 * @brief USART2 中断服务的 BSP 层处理函数。
 */
void BSP_UART2_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_H */
