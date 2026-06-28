#ifndef PID_H
#define PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float out_min;
    float out_max;
    float integral_min;
    float integral_max;
    uint8_t first_update;
} pid_t;

void PID_Init(pid_t *pid,
              float kp,
              float ki,
              float kd,
              float out_min,
              float out_max,
              float integral_min,
              float integral_max);

void PID_Reset(pid_t *pid);
float PID_Update(pid_t *pid, float setpoint, float measurement, float dt_s);
void PID_SetGains(pid_t *pid, float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
