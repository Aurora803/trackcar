#include "bsp_pwm.h"
#include "app_config.h"
#include "common_types.h"
#include "stm32f10x.h"

static uint16_t g_motor_pwm_period = 0U;

void BSP_PWM_MotorInit(void)
{
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;
    uint32_t timer_clock = SYS_CORE_CLOCK_HZ;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    /* 用户接线图：PB0 -> PWMA, PB1 -> PWMB。
     * 对应 TIM3_CH3 / TIM3_CH4 默认复用功能，无需 TIM3 重映射。
     */
    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &gpio);

    g_motor_pwm_period = (uint16_t)((timer_clock / MOTOR_PWM_FREQ_HZ) - 1U);

    tim.TIM_Prescaler = 0U;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = g_motor_pwm_period;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM3, &tim);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse = 0U;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC3Init(TIM3, &oc);
    TIM_OC4Init(TIM3, &oc);
    TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);

    TIM_Cmd(TIM3, ENABLE);
}

void BSP_PWM_SetMotorDutyPermille(uint8_t channel, int16_t duty_permille)
{
    uint16_t pulse;

    duty_permille = clamp_i16(duty_permille, 0, 1000);
    pulse = (uint16_t)(((uint32_t)duty_permille * (uint32_t)(g_motor_pwm_period + 1U)) / 1000U);

    if (channel == 1U)
    {
        TIM_SetCompare1(TIM3, pulse);
    }
    else if (channel == 2U)
    {
        TIM_SetCompare2(TIM3, pulse);
    }
    else if (channel == 3U)
    {
        TIM_SetCompare3(TIM3, pulse);
    }
    else if (channel == 4U)
    {
        TIM_SetCompare4(TIM3, pulse);
    }
    else
    {
        /* 无效通道，不处理。 */
    }
}

uint16_t BSP_PWM_GetMotorPeriod(void)
{
    return g_motor_pwm_period;
}
