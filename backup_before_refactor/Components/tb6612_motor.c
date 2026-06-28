#include "tb6612_motor.h"
#include "bsp_gpio.h"
#include "bsp_pwm.h"
#include "app_config.h"
#include "common_types.h"
#include <stdlib.h>

static const gpio_pin_t STBY_PIN = {TB6612_STBY_PORT, TB6612_STBY_PIN};
static const gpio_pin_t AIN1_PIN = {TB6612_AIN1_PORT, TB6612_AIN1_PIN};
static const gpio_pin_t AIN2_PIN = {TB6612_AIN2_PORT, TB6612_AIN2_PIN};
static const gpio_pin_t BIN1_PIN = {TB6612_BIN1_PORT, TB6612_BIN1_PIN};
static const gpio_pin_t BIN2_PIN = {TB6612_BIN2_PORT, TB6612_BIN2_PIN};

static void set_dir(gpio_pin_t in1, gpio_pin_t in2, int16_t speed)
{
    if (speed > 0)
    {
        BSP_GPIO_Write(in1, 1U);
        BSP_GPIO_Write(in2, 0U);
    }
    else if (speed < 0)
    {
        BSP_GPIO_Write(in1, 0U);
        BSP_GPIO_Write(in2, 1U);
    }
    else
    {
        /* 空转停止 */
        BSP_GPIO_Write(in1, 0U);
        BSP_GPIO_Write(in2, 0U);
    }
}

void TB6612_Init(void)
{
    BSP_PWM_MotorInit();
    TB6612_Standby(0U);
    TB6612_Coast(MOTOR_CHANNEL_LEFT);
    TB6612_Coast(MOTOR_CHANNEL_RIGHT);
}

void TB6612_SetSpeedPermille(motor_channel_t channel, int16_t speed_permille)
{
    int16_t speed = clamp_i16(speed_permille, -1000, 1000);
    int16_t duty;

    if (channel == MOTOR_CHANNEL_LEFT)
    {
#if MOTOR_LEFT_INVERT
        speed = (int16_t)(-speed);
#endif
        set_dir(AIN1_PIN, AIN2_PIN, speed);
        duty = (int16_t)abs(speed);
        BSP_PWM_SetMotorDutyPermille(MOTOR_LEFT_PWM_CHANNEL, duty);
    }
    else
    {
#if MOTOR_RIGHT_INVERT
        speed = (int16_t)(-speed);
#endif
        set_dir(BIN1_PIN, BIN2_PIN, speed);
        duty = (int16_t)abs(speed);
        BSP_PWM_SetMotorDutyPermille(MOTOR_RIGHT_PWM_CHANNEL, duty);
    }
}

void TB6612_Brake(motor_channel_t channel)
{
    if (channel == MOTOR_CHANNEL_LEFT)
    {
        BSP_GPIO_Write(AIN1_PIN, 1U);
        BSP_GPIO_Write(AIN2_PIN, 1U);
        BSP_PWM_SetMotorDutyPermille(MOTOR_LEFT_PWM_CHANNEL, 1000);
    }
    else
    {
        BSP_GPIO_Write(BIN1_PIN, 1U);
        BSP_GPIO_Write(BIN2_PIN, 1U);
        BSP_PWM_SetMotorDutyPermille(MOTOR_RIGHT_PWM_CHANNEL, 1000);
    }
}

void TB6612_Coast(motor_channel_t channel)
{
    if (channel == MOTOR_CHANNEL_LEFT)
    {
        BSP_GPIO_Write(AIN1_PIN, 0U);
        BSP_GPIO_Write(AIN2_PIN, 0U);
        BSP_PWM_SetMotorDutyPermille(MOTOR_LEFT_PWM_CHANNEL, 0);
    }
    else
    {
        BSP_GPIO_Write(BIN1_PIN, 0U);
        BSP_GPIO_Write(BIN2_PIN, 0U);
        BSP_PWM_SetMotorDutyPermille(MOTOR_RIGHT_PWM_CHANNEL, 0);
    }
}

void TB6612_Standby(uint8_t enable_standby)
{
    /* enable_standby=1 表示进入待机；0 表示正常工作。 */
    BSP_GPIO_Write(STBY_PIN, enable_standby ? 0U : 1U);
}

static const motor_driver_t g_tb6612_driver =
{
    TB6612_Init,
    TB6612_SetSpeedPermille,
    TB6612_Brake,
    TB6612_Coast,
    TB6612_Standby
};

const motor_driver_t *TB6612_GetDriver(void)
{
    return &g_tb6612_driver;
}
