#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    APP_OK = 0,
    APP_ERROR = -1,
    APP_TIMEOUT = -2,
    APP_INVALID_PARAM = -3
} app_status_t;

typedef enum
{
    APP_FALSE = 0,
    APP_TRUE = 1
} app_bool_t;

static inline int16_t clamp_i16(int32_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return (int16_t)value;
}

static inline float clamp_f32(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

#ifdef __cplusplus
}
#endif

#endif /* COMMON_TYPES_H */
