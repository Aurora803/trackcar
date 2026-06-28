#ifndef TB6612_MOTOR_H
#define TB6612_MOTOR_H

#include "motor_if.h"

#ifdef __cplusplus
extern "C" {
#endif

const motor_driver_t *TB6612_GetDriver(void);
void TB6612_Init(void);
void TB6612_SetSpeedPermille(motor_channel_t channel, int16_t speed_permille);
void TB6612_Brake(motor_channel_t channel);
void TB6612_Coast(motor_channel_t channel);
void TB6612_Standby(uint8_t enable_standby);

#ifdef __cplusplus
}
#endif

#endif /* TB6612_MOTOR_H */
