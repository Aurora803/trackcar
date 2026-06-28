#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BSP_Encoder_Init(void);
int16_t BSP_Encoder_ReadLeftDelta(void);
int16_t BSP_Encoder_ReadRightDelta(void);
void BSP_Encoder_Reset(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ENCODER_H */
