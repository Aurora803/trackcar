/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "bsp_systick.h"
#include "bsp_uart.h"

/*
 * 工程自定义说明：
 * 本文件属于 User/中断层。SysTick 只递增 1ms tick，USART2 中断只转发到 BSP
 * 接收缓冲处理；不要在中断中加入 PID、printf 或电机控制等耗时逻辑。
 */

/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  故障时紧急关闭电机输出。
  *         直接操作寄存器：即使外设尚未完整初始化也不会二次故障。
  *         先确保 TIM1/GPIOB 时钟已开启，然后关闭 TIM1 主输出、
  *         清零两路 PWM 占空比、复位 PB12~PB15 方向脚。
  * @param  None
  * @retval None
  */
static void Fault_Motor_Shutdown(void)
{
    /* 确保 TIM1 和 GPIOB 时钟已开启（幂等操作，不会破坏已配置的时钟）。 */
    RCC->APB2ENR |= (uint32_t)(RCC_APB2ENR_TIM1EN | RCC_APB2ENR_IOPBEN);
    __DSB();
    __ISB();

    /* 关闭 TIM1 主输出使能，PWM 立即停止。 */
    TIM1->BDTR &= (uint16_t)(~((uint16_t)TIM_BDTR_MOE));

    /* 两路 PWM 占空比清零。 */
    TIM1->CCR1 = 0;
    TIM1->CCR2 = 0;

    /* PB12~PB15 方向脚全部拉低，电机驱动芯片进入停止态。 */
    GPIOB->BRR = (uint32_t)(GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
}

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* 先强制关闭电机输出，再进入死循环。 */
  Fault_Motor_Shutdown();

  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* 先强制关闭电机输出，再进入死循环。 */
  Fault_Motor_Shutdown();

  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* 先强制关闭电机输出，再进入死循环。 */
  Fault_Motor_Shutdown();

  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* 先强制关闭电机输出，再进入死循环。 */
  Fault_Motor_Shutdown();

  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  /* 1 ms 系统节拍。主循环里的控制、遥测和 LED 调度都依赖这个计数递增。 */
  BSP_SysTick_Inc();
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @brief  This function handles USART2 global interrupt request.
  * @param  None
  * @retval None
  */
void USART2_IRQHandler(void)
{
  /* 启动文件向量表要求使用 USART2_IRQHandler 这个符号名。
   * BSP 层只负责串口收发细节，这里只做一次薄转发，避免中断落入 Default_Handler。
   */
  BSP_UART2_IRQHandler();
}

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
