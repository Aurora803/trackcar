/**
 * @file bsp_encoder.c
 * @brief TIM2/TIM4 正交编码器采样实现。
 * @layer BSP
 *
 * 本文件只负责定时器编码器模式配置和计数增量读取。速度计算和 PID
 * 应在 Chassis/控制层完成，不放在 BSP 中。
 */
#include "bsp_encoder.h"
#include "app_config.h"
#include "stm32f10x.h"

/**
 * @brief 将指定定时器配置为 TI1/TI2 正交编码器模式。
 */
static void encoder_timer_init(TIM_TypeDef *timx)
{
    TIM_TimeBaseInitTypeDef tb;
    TIM_ICInitTypeDef ic;

    tb.TIM_Prescaler = 0U;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
    /* 16 位自由运行计数器。读取时转换为 int16_t，因而两次读取间的
     * 实际位移必须小于 32768 个计数；当前 10 ms 控制周期远小于该上限。
     */
    tb.TIM_Period = 0xFFFFU;
    tb.TIM_ClockDivision = TIM_CKD_DIV1;
    tb.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(timx, &tb);

    TIM_EncoderInterfaceConfig(timx,
                               TIM_EncoderMode_TI12,
                               TIM_ICPolarity_Rising,
                               TIM_ICPolarity_Rising);

    ic.TIM_Channel = TIM_Channel_1;
    ic.TIM_ICPolarity = TIM_ICPolarity_Rising;
    ic.TIM_ICSelection = TIM_ICSelection_DirectTI;
    ic.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    /* 数字滤波抑制电机噪声造成的毛刺；数值越大抗干扰越强，但会降低
     * 高速边沿响应。此处对小车编码器取折中值。
     */
    ic.TIM_ICFilter = 6U;
    TIM_ICInit(timx, &ic);

    ic.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(timx, &ic);

    TIM_SetCounter(timx, 0U);
    TIM_Cmd(timx, ENABLE);
}

/**
 * @brief 初始化左 TIM2(PA0/PA1) 和右 TIM4(PB6/PB7) 编码器。
 */
void BSP_Encoder_Init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM4, ENABLE);

    /* TIM2_CH1/CH2: PA0/PA1 */
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* TIM4_CH1/CH2: PB6/PB7 */
    gpio.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    encoder_timer_init(TIM2);
    encoder_timer_init(TIM4);
}

/**
 * @brief 读取左编码器自上次读取以来的增量并清零计数器。
 */
int16_t BSP_Encoder_ReadLeftDelta(void)
{
    /* 把 16 位补码计数直接转为 int16_t，可自然得到向后运动或回绕后的
     * 有符号小增量；读取后立即清零，增量窗口由调用周期决定。
     */
    int16_t delta = (int16_t)TIM_GetCounter(TIM2);
    TIM_SetCounter(TIM2, 0U);
#if ENCODER_LEFT_INVERT
    /* 软件反向仅统一“前进为正”的上层约定，不改变定时器的解码方向。 */
    delta = (int16_t)(-delta);
#endif
    return delta;
}

/**
 * @brief 读取右编码器自上次读取以来的增量并清零计数器。
 */
int16_t BSP_Encoder_ReadRightDelta(void)
{
    int16_t delta = (int16_t)TIM_GetCounter(TIM4);
    TIM_SetCounter(TIM4, 0U);
#if ENCODER_RIGHT_INVERT
    delta = (int16_t)(-delta);
#endif
    return delta;
}

/**
 * @brief 清零左右编码器硬件计数器。
 */
void BSP_Encoder_Reset(void)
{
    TIM_SetCounter(TIM2, 0U);
    TIM_SetCounter(TIM4, 0U);
}
