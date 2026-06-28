#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    int16_t x_error;
    int16_t y_error;
    uint8_t valid;
    uint32_t timestamp_ms;
} vision_target_t;

void VisionProtocol_Init(void);
uint8_t VisionProtocol_Poll(vision_target_t *out_target, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* VISION_PROTOCOL_H */
