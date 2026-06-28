#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_PWM_MotorInit(void);
void BSP_PWM_SetMotorDutyPermille(uint8_t channel, int16_t duty_permille);
uint16_t BSP_PWM_GetMotorPeriod(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_PWM_H */
