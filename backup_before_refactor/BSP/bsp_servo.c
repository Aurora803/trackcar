#include "bsp_servo.h"
#include "app_config.h"
#include "common_types.h"
#include "stm32f10x.h"

/*
 * 舵机/云台预留说明：
 * 当前接线图中 PA8 已用于 TB6612 STBY，PA2/PA3/PA4/PA5/PB8/PB9/PB10/PB11 用于 8 路循迹，
 * 可直接用于硬件 PWM 舵机的空闲定时器通道不足。
 * 因此本文件默认不启用。后续建议：
 * 1. 将 8 路循迹模块改为 I2C/串口读取，释放 TIM2/TIM4/TIM1 相关引脚；或
 * 2. 外接 PCA9685 舵机驱动板；或
 * 3. 重排 PA8/STBY 等引脚后再启用 TIM1 舵机 PWM。
 */

#if APP_ENABLE_GIMBAL_SERVO
#error "当前接线图未给二维云台分配安全的硬件 PWM 引脚。请先重新规划云台 PWM 引脚或改用 PCA9685。"
static uint16_t angle_to_pulse(float angle_deg)
{
    float us;
    angle_deg = clamp_f32(angle_deg, 0.0f, 180.0f);
    us = 500.0f + (angle_deg / 180.0f) * 2000.0f; /* 0.5ms~2.5ms */
    return (uint16_t)us;
}
#endif

void BSP_Servo_Init(void)
{
#if APP_ENABLE_GIMBAL_SERVO
    GPIO_InitTypeDef gpio;
    TIM_TimeBaseInitTypeDef tim;
    TIM_OCInitTypeDef oc;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_TIM1, ENABLE);

    gpio.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_11;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &gpio);

    /* 72MHz / 72 = 1MHz，ARR=19999 -> 50Hz。 */
    tim.TIM_Prescaler = 71U;
    tim.TIM_CounterMode = TIM_CounterMode_Up;
    tim.TIM_Period = 19999U;
    tim.TIM_ClockDivision = TIM_CKD_DIV1;
    tim.TIM_RepetitionCounter = 0U;
    TIM_TimeBaseInit(TIM1, &tim);

    oc.TIM_OCMode = TIM_OCMode_PWM1;
    oc.TIM_OutputState = TIM_OutputState_Enable;
    oc.TIM_OutputNState = TIM_OutputNState_Disable;
    oc.TIM_Pulse = angle_to_pulse(90.0f);
    oc.TIM_OCPolarity = TIM_OCPolarity_High;
    oc.TIM_OCNPolarity = TIM_OCNPolarity_High;
    oc.TIM_OCIdleState = TIM_OCIdleState_Reset;
    oc.TIM_OCNIdleState = TIM_OCNIdleState_Reset;

    TIM_OC1Init(TIM1, &oc);
    TIM_OC4Init(TIM1, &oc);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);
#endif
}

void BSP_Servo_SetAngleDeg(uint8_t channel, float angle_deg)
{
#if APP_ENABLE_GIMBAL_SERVO
    uint16_t pulse = angle_to_pulse(angle_deg);
    if (channel == 1U)
    {
        TIM_SetCompare1(TIM1, pulse);
    }
    else if (channel == 2U)
    {
        TIM_SetCompare4(TIM1, pulse);
    }
#else
    (void)channel;
    (void)angle_deg;
#endif
}
