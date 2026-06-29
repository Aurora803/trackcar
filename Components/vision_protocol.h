/**
 * @file vision_protocol.h
 * @brief OpenMV/OpenCV/树莓派视觉目标协议接口。
 * @layer Components
 *
 * 当前协议为 ASCII 行：$T,<x_err>,<y_err>,<valid>。正式打靶前可扩展校验、
 * 目标类型、时间戳或帧序号。
 */
#ifndef VISION_PROTOCOL_H
#define VISION_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    /* 目标相对画面中心的水平误差，左负右正，单位由视觉端定义。 */
    int16_t x_error;
    /* 目标相对画面中心的垂直误差，上负下正或按视觉端约定。 */
    int16_t y_error;
    /* 1 表示目标有效，0 表示目标丢失。 */
    uint8_t valid;
    /* STM32 接收并解析成功时的系统时间。 */
    uint32_t timestamp_ms;
} vision_target_t;

/**
 * @brief 初始化视觉协议解析状态。
 */
void VisionProtocol_Init(void);

/**
 * @brief 轮询解析一帧视觉目标消息。
 * @return 1 表示解析出一帧有效格式消息，0 表示暂无完整消息。
 */
uint8_t VisionProtocol_Poll(vision_target_t *out_target, uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif /* VISION_PROTOCOL_H */
