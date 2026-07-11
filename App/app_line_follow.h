/**
 * @file app_line_follow.h
 * @brief 8 路红外循迹控制接口。
 * @layer App
 *
 * 本模块只暴露初始化、周期更新和调试快照。传感器读取通过 tracker8_driver_t
 * 注入，底盘输出通过 Chassis 层完成，避免直接绑定具体 GPIO 或 TB6612。
 */
#ifndef APP_LINE_FOLLOW_H
#define APP_LINE_FOLLOW_H

#include <stdint.h>
#include "tracker8_if.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    /* 上电稳定：电机输出 0，等待 LINE_START_STABLE_MS 后进入 FOLLOW。 */
    LINE_STATE_START = 0,
    /* 正常循迹：读取 8 路误差，PID 差速输出，识别直角和丢线。 */
    LINE_STATE_FOLLOW,
    /* 丢线搜索：按最后一次有效误差低速偏转找线，超时进入 LOST。 */
    LINE_STATE_BLIND,
    /* 矩形直角：一侧低速一侧高速定量转弯，按时间或编码器/中心线退出。 */
    LINE_STATE_CORNER,
    /* 找回后恢复：低速 PID 恢复一小段时间，再回 FOLLOW。 */
    LINE_STATE_RECOVER,
    /* 长时间找不到线：停车等待连续重新看到线。 */
    LINE_STATE_LOST
} line_follow_state_t;

typedef enum
{
    LINE_SENSOR_FAULT_NONE = 0,
    LINE_SENSOR_FAULT_ALL_INACTIVE,
    LINE_SENSOR_FAULT_ALL_ACTIVE,
    LINE_SENSOR_FAULT_DRIVER_INVALID
} line_sensor_fault_t;

typedef struct
{
    /* 最近一次 8 路循迹采样。 */
    tracker8_sample_t tracker;
    /* 当前状态机状态。 */
    line_follow_state_t state;
    /* 最近一次下发到底盘层的 PWM 命令，单位 permille。 */
    int16_t left_pwm;
    int16_t right_pwm;
    /* PID 差速修正量，左负右正误差会映射到左右轮差速。 */
    int16_t correction;
    int8_t corner_dir;          /* -1 left, +1 right, 0 none */
    uint16_t corner_count;      /* completed 90-degree corners */
    /* 当前直角弯累计编码器计数；时间转弯模式下仍保留用于调试。 */
    uint16_t corner_encoder_sum;
    /* 0 正常，1 长时间全未触发，2 长时间全触发，3 驱动报告无效。 */
    line_sensor_fault_t sensor_fault;
    /* 当前状态已持续时间，单位 ms。 */
    uint32_t state_time_ms;
} line_follow_debug_t;

/**
 * @brief 初始化循迹控制器并绑定传感器驱动。
 */
void AppLineFollow_Init(const tracker8_driver_t *tracker_driver);

/**
 * @brief 运行一次循迹状态机。
 * @param dt_ms 距离上次调用的时间间隔，通常为 APP_CONTROL_PERIOD_MS。
 */
void AppLineFollow_Update(uint32_t dt_ms);

/**
 * @brief 获取循迹调试快照，用于串口遥测。
 */
line_follow_debug_t AppLineFollow_GetDebug(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_LINE_FOLLOW_H */
