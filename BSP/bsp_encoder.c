#include "bsp_encoder.h"
#include "app_config.h"
#include "stm32f10x.h"

static void encoder_timer_init(TIM_TypeDef *timx)
{
    TIM_TimeBaseInitTypeDef tb;
    TIM_ICInitTypeDef ic;

    tb.TIM_Prescaler = 0U;
    tb.TIM_CounterMode = TIM_CounterMode_Up;
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
    ic.TIM_ICFilter = 6U;
    TIM_ICInit(timx, &ic);

    ic.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(timx, &ic);

    TIM_SetCounter(timx, 0U);
    TIM_Cmd(timx, ENABLE);
}

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

int16_t BSP_Encoder_ReadLeftDelta(void)
{
    int16_t delta = (int16_t)TIM_GetCounter(TIM2);
    TIM_SetCounter(TIM2, 0U);
#if ENCODER_LEFT_INVERT
    delta = (int16_t)(-delta);
#endif
    return delta;
}

int16_t BSP_Encoder_ReadRightDelta(void)
{
    int16_t delta = (int16_t)TIM_GetCounter(TIM4);
    TIM_SetCounter(TIM4, 0U);
#if ENCODER_RIGHT_INVERT
    delta = (int16_t)(-delta);
#endif
    return delta;
}

void BSP_Encoder_Reset(void)
{
    TIM_SetCounter(TIM2, 0U);
    TIM_SetCounter(TIM4, 0U);
}
