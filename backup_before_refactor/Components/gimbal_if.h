#ifndef GIMBAL_IF_H
#define GIMBAL_IF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    void (*init)(void);
    void (*set_angle_deg)(float pan_deg, float tilt_deg);
    void (*center)(void);
} gimbal_driver_t;

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_IF_H */
