#include "pid.h"
#include "common_types.h"

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

void PID_Reset(pid_t *pid)
{
    if (pid == 0) return;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->first_update = 1U;
}

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

void PID_SetGains(pid_t *pid, float kp, float ki, float kd)
{
    if (pid == 0) return;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
