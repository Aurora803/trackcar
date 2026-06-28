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

/* 视觉通信抽象接口。当前接线图使用 USART1 PA9/PA10 预留视觉串口。 */
void BSP_VisionUART_SendChar(char ch);
void BSP_VisionUART_SendString(const char *str);
int BSP_VisionUART_ReadCharNonBlocking(char *out_ch);

/* 当前仅启用 USART1 中断。PA2/PA3 被循迹 S1/S2 占用，不再初始化 USART2。 */
void BSP_UART1_IRQHandler(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_H */
