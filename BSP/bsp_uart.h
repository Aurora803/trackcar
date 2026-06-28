#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_UART_Init(void);
void BSP_UART1_SendChar(char ch);
void BSP_UART1_SendString(const char *str);
void BSP_UART1_SendInt(const char *name, int32_t value);

/* 视觉通信抽象接口。本版先不启用视觉，接口临时复用 USART2 调试串口。 */
void BSP_VisionUART_SendChar(char ch);
void BSP_VisionUART_SendString(const char *str);
int BSP_VisionUART_ReadCharNonBlocking(char *out_ch);

/* 当前启用 USART2 中断，PA2/PA3 用作调试串口。 */
void BSP_UART2_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_H */
