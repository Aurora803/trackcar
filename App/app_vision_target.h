/**
 * @file app_vision_target.h
 * @brief 视觉目标跟踪应用接口预留。
 * @layer App
 *
 * 当前默认 APP_ENABLE_VISION_TARGET=0，主循环不会调度视觉控制。
 * 该接口为后续 OpenMV/OpenCV/树莓派目标误差和二维云台控制预留。
 */
#ifndef APP_VISION_TARGET_H
#define APP_VISION_TARGET_H

#include <stdint.h>
#include "gimbal_if.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化视觉目标跟踪模块。
 * @param gimbal_driver 可为空；为空时只解析视觉协议，不输出云台动作。
 */
void AppVisionTarget_Init(const gimbal_driver_t *gimbal_driver);

/**
 * @brief 运行一次视觉目标跟踪更新。
 * @param now_ms 当前系统毫秒 tick。
 */
void AppVisionTarget_Update(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* APP_VISION_TARGET_H */
