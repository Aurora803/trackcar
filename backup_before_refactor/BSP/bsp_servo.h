#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_Servo_Init(void);
void BSP_Servo_SetAngleDeg(uint8_t channel, float angle_deg);

#ifdef __cplusplus
}
#endif

#endif /* BSP_SERVO_H */
