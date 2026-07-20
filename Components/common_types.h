/**
 * @file common_types.h
 * @brief 跨模块通用类型和简单限幅工具。
 * @layer Components
 */
#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    /* 通用成功返回。 */
    APP_OK = 0,
    /* 通用失败返回。 */
    APP_ERROR = -1,
    /* 等待或通信超时。 */
    APP_TIMEOUT = -2,
    /* 入参为空、越界或不支持。 */
    APP_INVALID_PARAM = -3
} app_status_t;

typedef enum
{
    APP_FALSE = 0,
    APP_TRUE = 1
} app_bool_t;

/**
 * @brief int16_t 限幅，常用于 PWM 命令和编码器调试值。
 */
static inline int16_t clamp_i16(int32_t value, int16_t min_value, int16_t max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return (int16_t)value;
}

/**
 * @brief float 限幅，常用于 PID 输出和积分项。
 */
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
