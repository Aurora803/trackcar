/**
 * @file chassis.c
 * @brief 底盘抽象层实现。
 * @layer Components
 *
 * 本层不关心循迹、视觉或模式切换，只统一处理左右轮 PWM 命令、
 * 停车动作和编码器增量采样。
 */
#include "chassis.h"
#include "bsp_encoder.h"
#include "common_types.h"
#include "app_config.h"

static const motor_driver_t *g_motor = 0;
static chassis_state_t g_state;

/**
 * @brief 初始化底盘状态、电机驱动和编码器。
 */
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

/**
 * @brief 下发左右轮 PWM，并在底盘层做统一限幅。
 */
void Chassis_SetPWM(int16_t left_pwm, int16_t right_pwm)
{
    /* 底盘层只做通用 PWM 安全限幅，不关心当前是循迹还是后续视觉/云台模式。 */
    left_pwm = clamp_i16(left_pwm, -CHASSIS_PWM_LIMIT, CHASSIS_PWM_LIMIT);
    right_pwm = clamp_i16(right_pwm, -CHASSIS_PWM_LIMIT, CHASSIS_PWM_LIMIT);

    g_state.left_pwm = left_pwm;
    g_state.right_pwm = right_pwm;

    if (g_motor != 0 && g_motor->set_speed_permille != 0)
    {
        g_motor->set_speed_permille(MOTOR_CHANNEL_LEFT, left_pwm);
        g_motor->set_speed_permille(MOTOR_CHANNEL_RIGHT, right_pwm);
    }
}

/**
 * @brief 空转停止左右轮，适合常规停车和丢线等待。
 */
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

/**
 * @brief 刹车停止左右轮，适合需要更强制动的场景。
 */
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

/**
 * @brief 同步读取左右编码器增量。
 *
 * 当前项目仍是 PWM 开环循迹；该接口已按 10ms 控制周期调用，为后续速度闭环预留。
 */
void Chassis_UpdateEncoder(void)
{
    g_state.left_encoder_delta = BSP_Encoder_ReadLeftDelta();
    g_state.right_encoder_delta = BSP_Encoder_ReadRightDelta();
}

/**
 * @brief 返回底盘状态快照。
 */
chassis_state_t Chassis_GetState(void)
{
    return g_state;
}
