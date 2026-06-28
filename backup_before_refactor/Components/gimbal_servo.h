#ifndef GIMBAL_SERVO_H
#define GIMBAL_SERVO_H

#include "gimbal_if.h"

#ifdef __cplusplus
extern "C" {
#endif

const gimbal_driver_t *GimbalServo_GetDriver(void);
void GimbalServo_Init(void);
void GimbalServo_SetAngleDeg(float pan_deg, float tilt_deg);
void GimbalServo_Center(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_SERVO_H */
