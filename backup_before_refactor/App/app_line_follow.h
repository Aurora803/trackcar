#ifndef APP_LINE_FOLLOW_H
#define APP_LINE_FOLLOW_H

#include <stdint.h>
#include "tracker8_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    tracker8_sample_t tracker;
    int16_t left_pwm;
    int16_t right_pwm;
    int16_t correction;
} line_follow_debug_t;

void AppLineFollow_Init(const tracker8_driver_t *tracker_driver);
void AppLineFollow_Update(uint32_t dt_ms);
line_follow_debug_t AppLineFollow_GetDebug(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_LINE_FOLLOW_H */
