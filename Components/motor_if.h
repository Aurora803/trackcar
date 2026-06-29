/**
 * @file motor_if.h
 * @brief 电机驱动抽象接口。
 * @layer Components
 *
 * 上层底盘只依赖本接口，不直接依赖 TB6612 或具体 GPIO/PWM。
 * 更换电机驱动芯片时实现新的 motor_driver_t 即可。
 */
#ifndef MOTOR_IF_H
#define MOTOR_IF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    /* 左轮电机通道。 */
    MOTOR_CHANNEL_LEFT = 0,
    /* 右轮电机通道。 */
    MOTOR_CHANNEL_RIGHT = 1
} motor_channel_t;

typedef struct
{
    /* 初始化驱动芯片和相关 PWM/GPIO。 */
    void (*init)(void);
    /* 设置速度命令，单位 permille，正负表示前进/后退方向。 */
    void (*set_speed_permille)(motor_channel_t channel, int16_t speed_permille);
    /* 主动刹车。不同驱动芯片语义可能不同，需要实现层说明。 */
    void (*brake)(motor_channel_t channel);
    /* 空转停止。 */
    void (*coast)(motor_channel_t channel);
    /* 进入或退出待机，若硬件 STBY 固定接高电平可为空操作。 */
    void (*standby)(uint8_t enable_standby);
} motor_driver_t;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_IF_H */
