/**
 * @file tracker8_if.h
 * @brief 8 路循迹传感器抽象接口。
 * @layer Components
 *
 * App 层通过本接口读取 raw_bits、压线数量、位置误差和状态，不关心具体 GPIO 接线。
 */
#ifndef TRACKER8_IF_H
#define TRACKER8_IF_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    /* 正常识别到黑线，position_error 有效。 */
    TRACKER_STATUS_OK = 0,
    /* 当前没有任何有效压线，position_error 保持 last_valid_error。 */
    TRACKER_STATUS_LOST,
    /* 多路同时触发，可能是横线、宽黑线或交叉区域。 */
    TRACKER_STATUS_CROSS,
    /* 采样无效或硬件未初始化。 */
    TRACKER_STATUS_INVALID
} tracker_status_t;

typedef struct
{
    uint8_t raw_bits;          /* bit0~bit7：1 表示检测到黑线 */
    /* 当前检测到黑线的传感器数量。 */
    uint8_t active_count;
    int16_t position_error;    /* 左负右正，0 为居中 */
    /* 最近一次有效误差，用于丢线搜索方向判断。 */
    int16_t last_valid_error;
    /* 当前采样状态。 */
    tracker_status_t status;
} tracker8_sample_t;

typedef struct
{
    /* 初始化传感器硬件或软件状态。 */
    void (*init)(void);
    /* 读取一次 8 路传感器并返回归一化采样结果。 */
    tracker8_sample_t (*read)(void);
} tracker8_driver_t;

#ifdef __cplusplus
}
#endif

#endif /* TRACKER8_IF_H */
