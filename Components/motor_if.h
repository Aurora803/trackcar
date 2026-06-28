#ifndef MOTOR_IF_H
#define MOTOR_IF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MOTOR_CHANNEL_LEFT = 0,
    MOTOR_CHANNEL_RIGHT = 1
} motor_channel_t;

typedef struct
{
    void (*init)(void);
    void (*set_speed_permille)(motor_channel_t channel, int16_t speed_permille);
    void (*brake)(motor_channel_t channel);
    void (*coast)(motor_channel_t channel);
    void (*standby)(uint8_t enable_standby);
} motor_driver_t;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_IF_H */
