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

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1 | RCC_APB2Periph_AFIO, ENABLE);

    /* 最新接线图：PA9 -> PWMA(左电机), PA8 -> PWMB(右电机)。
     * 对应 TIM1_CH2 / TIM1_CH1。TIM1 是高级定时器，必须调用 TIM_CtrlPWMOutputs()。
     */
    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* 72MHz / (71+1) = 1MHz；ARR=999 -> 1kHz PWM。 */
    g_motor_pwm_period = (uint16_t)((SYS_CORE_CLOCK_HZ / 72U / MOTOR_PWM_FREQ_HZ) - 1U);

    tim.TIM_Prescaler = 71U;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = g_motor_pwm_period;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM1, &tim);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse = 0U;
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM1, &oc); /* PA8  -> TIM1_CH1 -> PWMB / 右电机 */
    TIM_OC2Init(TIM1, &oc); /* PA9  -> TIM1_CH2 -> PWMA / 左电机 */
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);

    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
}

void BSP_PWM_SetMotorDutyPermille(uint8_t channel, int16_t duty_permille)
{
    uint16_t pulse;

    duty_permille = clamp_i16(duty_permille, 0, 1000);
    pulse = (uint16_t)(((uint32_t)duty_permille * (uint32_t)(g_motor_pwm_period + 1U)) / 1000U);

    if (channel == 1U)
    {
        TIM_SetCompare1(TIM1, pulse); /* 右电机 PWMB：PA8/TIM1_CH1 */
    }
    else if (channel == 2U)
    {
        TIM_SetCompare2(TIM1, pulse); /* 左电机 PWMA：PA9/TIM1_CH2 */
    }
    else
    {
        /* 当前接线只初始化 TIM1_CH1(PA8) / TIM1_CH2(PA9)。
         * 传入其他通道属于配置错误，直接忽略。
         */
    }
}

uint16_t BSP_PWM_GetMotorPeriod(void)
{
    return g_motor_pwm_period;
}
