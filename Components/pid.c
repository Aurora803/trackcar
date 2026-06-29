/**
 * @file pid.c
 * @brief 通用 PID 控制器实现。
 * @layer Components
 */
#include "pid.h"
#include "common_types.h"

/**
 * @brief 初始化 PID 参数、积分限幅和输出限幅。
 */
void PID_Init(pid_t *pid,
              float kp,
              float ki,
              float kd,
              float out_min,
              float out_max,
              float integral_min,
              float integral_max)
{
    if (pid == 0) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_min = integral_min;
    pid->integral_max = integral_max;
    pid->first_update = 1U;
}

/**
 * @brief 清空动态状态，保留 PID 参数和限幅。
 */
void PID_Reset(pid_t *pid)
{
    if (pid == 0) return;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->first_update = 1U;
}

/**
 * @brief 执行一次 PID 计算。
 *
 * 本函数不做任何硬件访问。调用者负责提供固定周期 dt_s，并把输出映射到
 * 电机、舵机或其他执行机构。
 */
float PID_Update(pid_t *pid, float setpoint, float measurement, float dt_s)
{
    float error;
    float derivative;
    float output;

    if (pid == 0 || dt_s <= 0.0f)
    {
        return 0.0f;
    }

    error = setpoint - measurement;

    pid->integral += error * dt_s;
    pid->integral = clamp_f32(pid->integral, pid->integral_min, pid->integral_max);

    if (pid->first_update)
    {
        derivative = 0.0f;
        pid->first_update = 0U;
    }
    else
    {
        derivative = (error - pid->prev_error) / dt_s;
    }

    output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    output = clamp_f32(output, pid->out_min, pid->out_max);

    pid->prev_error = error;
    return output;
}

/**
 * @brief 修改 PID 参数，适用于运行中调参。
 */
void PID_SetGains(pid_t *pid, float kp, float ki, float kd)
{
    if (pid == 0) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
