#ifndef TRACKER8_IF_H
#define TRACKER8_IF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    TRACKER_STATUS_OK = 0,
    TRACKER_STATUS_LOST,
    TRACKER_STATUS_CROSS,
    TRACKER_STATUS_INVALID
} tracker_status_t;

typedef struct
{
    uint8_t raw_bits;          /* bit0~bit7：1 表示检测到黑线 */
    uint8_t active_count;
    int16_t position_error;    /* 左负右正，0 为居中 */
    int16_t last_valid_error;
    tracker_status_t status;
} tracker8_sample_t;

typedef struct
{
    void (*init)(void);
    tracker8_sample_t (*read)(void);
} tracker8_driver_t;

#ifdef __cplusplus
}
#endif

#endif /* TRACKER8_IF_H */
