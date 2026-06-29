/**
 * @file pid.h
 * @brief 通用 PID 控制器接口。
 * @layer Components
 *
 * PID 模块只做数学计算，不读取传感器、不控制电机、不写 PWM。
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* 比例、积分、微分系数。 */
    float kp;
    float ki;
    float kd;
    /* 积分累计值和上一次误差。 */
    float integral;
    float prev_error;
    /* 输出限幅。 */
    float out_min;
    float out_max;
    /* 积分限幅，用于抑制积分饱和。 */
    float integral_min;
    float integral_max;
    /* 首次更新时不计算微分项，避免 D 项突跳。 */
    uint8_t first_update;
} pid_t;

/**
 * @brief 初始化 PID 参数和限幅。
 */
void PID_Init(pid_t *pid,
              float kp,
              float ki,
              float kd,
              float out_min,
              float out_max,
              float integral_min,
              float integral_max);

/**
 * @brief 复位积分、上次误差和首次更新标志。
 */
void PID_Reset(pid_t *pid);

/**
 * @brief 运行一次 PID 计算。
 * @param setpoint 目标值。
 * @param measurement 测量值。
 * @param dt_s 距离上次调用的时间，单位秒，必须大于 0。
 * @return 限幅后的 PID 输出。
 */
float PID_Update(pid_t *pid, float setpoint, float measurement, float dt_s);

/**
 * @brief 在线更新 PID 三项系数，不清除历史状态。
 */
void PID_SetGains(pid_t *pid, float kp, float ki, float kd);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
