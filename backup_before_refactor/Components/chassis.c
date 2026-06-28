#include "chassis.h"
#include "bsp_encoder.h"
#include "common_types.h"
#include "app_config.h"

static const motor_driver_t *g_motor = 0;
static chassis_state_t g_state;

void Chassis_Init(const motor_driver_t *motor_driver)
{
    g_motor = motor_driver;
    g_state.left_pwm = 0;
    g_state.right_pwm = 0;
    g_state.left_encoder_delta = 0;
    g_state.right_encoder_delta = 0;

    if (g_motor != 0 && g_motor->init != 0)
    {
        g_motor->init();
    }

    BSP_Encoder_Init();
}

void Chassis_SetPWM(int16_t left_pwm, int16_t right_pwm)
{
    left_pwm = clamp_i16(left_pwm, -LINE_PWM_LIMIT, LINE_PWM_LIMIT);
    right_pwm = clamp_i16(right_pwm, -LINE_PWM_LIMIT, LINE_PWM_LIMIT);

    g_state.left_pwm = left_pwm;
    g_state.right_pwm = right_pwm;

    if (g_motor != 0 && g_motor->set_speed_permille != 0)
    {
        g_motor->set_speed_permille(MOTOR_CHANNEL_LEFT, left_pwm);
        g_motor->set_speed_permille(MOTOR_CHANNEL_RIGHT, right_pwm);
    }
}

void Chassis_StopCoast(void)
{
    g_state.left_pwm = 0;
    g_state.right_pwm = 0;

    if (g_motor != 0 && g_motor->coast != 0)
    {
        g_motor->coast(MOTOR_CHANNEL_LEFT);
        g_motor->coast(MOTOR_CHANNEL_RIGHT);
    }
}

void Chassis_StopBrake(void)
{
    g_state.left_pwm = 0;
    g_state.right_pwm = 0;

    if (g_motor != 0 && g_motor->brake != 0)
    {
        g_motor->brake(MOTOR_CHANNEL_LEFT);
        g_motor->brake(MOTOR_CHANNEL_RIGHT);
    }
}

void Chassis_UpdateEncoder(void)
{
    g_state.left_encoder_delta = BSP_Encoder_ReadLeftDelta();
    g_state.right_encoder_delta = BSP_Encoder_ReadRightDelta();
}

chassis_state_t Chassis_GetState(void)
{
    return g_state;
}
