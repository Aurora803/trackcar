#ifndef APP_VISION_TARGET_H
#define APP_VISION_TARGET_H

#include <stdint.h>
#include "gimbal_if.h"

#ifdef __cplusplus
extern "C" {
#endif

void AppVisionTarget_Init(const gimbal_driver_t *gimbal_driver);
void AppVisionTarget_Update(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_VISION_TARGET_H */
