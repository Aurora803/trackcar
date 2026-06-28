#ifndef CHASSIS_H
#define CHASSIS_H

#include <stdint.h>
#include "motor_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int16_t left_pwm;
    int16_t right_pwm;
    int16_t left_encoder_delta;
    int16_t right_encoder_delta;
} chassis_state_t;

void Chassis_Init(const motor_driver_t *motor_driver);
void Chassis_SetPWM(int16_t left_pwm, int16_t right_pwm);
void Chassis_StopCoast(void);
void Chassis_StopBrake(void);
void Chassis_UpdateEncoder(void);
chassis_state_t Chassis_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* CHASSIS_H */
