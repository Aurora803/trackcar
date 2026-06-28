#ifndef APP_LINE_FOLLOW_H
#define APP_LINE_FOLLOW_H

#include <stdint.h>
#include "tracker8_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    LINE_STATE_START = 0,
    LINE_STATE_FOLLOW,
    LINE_STATE_BLIND,
    LINE_STATE_CORNER,
    LINE_STATE_RECOVER,
    LINE_STATE_LOST
} line_follow_state_t;

typedef struct
{
    tracker8_sample_t tracker;
    line_follow_state_t state;
    int16_t left_pwm;
    int16_t right_pwm;
    int16_t correction;
    int8_t corner_dir;          /* -1 left, +1 right, 0 none */
    uint16_t corner_count;      /* completed 90-degree corners */
    uint16_t corner_encoder_sum;
    uint32_t state_time_ms;
} line_follow_debug_t;

void AppLineFollow_Init(const tracker8_driver_t *tracker_driver);
void AppLineFollow_Update(uint32_t dt_ms);
line_follow_debug_t AppLineFollow_GetDebug(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_LINE_FOLLOW_H */
