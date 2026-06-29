/**
 * @file yahboom_tracker8_io.h
 * @brief Yahboom 8 路循迹模块适配层接口。
 * @layer Components
 *
 * 本模块实现 tracker8_driver_t，负责将具体 GPIO 电平转换为统一的 8 路采样结果。
 */
#ifndef YAHBOOM_TRACKER8_IO_H
#define YAHBOOM_TRACKER8_IO_H

#include "tracker8_if.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 获取 8 路循迹传感器驱动接口。
 */
const tracker8_driver_t *YahboomTracker8IO_GetDriver(void);

/**
 * @brief 初始化传感器适配层状态。
 */
void YahboomTracker8IO_Init(void);

/**
 * @brief 读取一次 8 路循迹状态。
 */
tracker8_sample_t YahboomTracker8IO_Read(void);

#ifdef __cplusplus
}
#endif

#endif /* YAHBOOM_TRACKER8_IO_H */
