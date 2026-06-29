/**
 * @file app_vision_target.c
 * @brief 视觉目标误差到云台角度的应用层预留实现。
 * @layer App
 *
 * 当前工程默认不启用视觉/云台。启用前必须先确认独立通信串口、
 * 云台 PWM 引脚或外部舵机驱动板，不要复用已占用的电机 PWM/调试串口。
 */
#include "app_vision_target.h"
#include "app_config.h"
#include "vision_protocol.h"
#include "common_types.h"

static const gimbal_driver_t *g_gimbal = 0;
#if APP_ENABLE_VISION_TARGET
static float g_pan = GIMBAL_PAN_CENTER_DEG;
static float g_tilt = GIMBAL_TILT_CENTER_DEG;
static uint32_t g_last_target_ms = 0U;
#endif

/**
 * @brief 初始化视觉协议和可选云台驱动。
 */
void AppVisionTarget_Init(const gimbal_driver_t *gimbal_driver)
{
    VisionProtocol_Init();
    g_gimbal = gimbal_driver;

    if (g_gimbal != 0 && g_gimbal->init != 0)
    {
        g_gimbal->init();
    }
}

/**
 * @brief 解析视觉目标误差并更新云台角度。
 *
 * 当前比例系数为占位逻辑，正式打靶前应根据画面分辨率、云台方向和舵机安装方向实测。
 */
void AppVisionTarget_Update(uint32_t now_ms)
{
#if APP_ENABLE_VISION_TARGET
    vision_target_t target;

    if (VisionProtocol_Poll(&target, now_ms))
    {
        g_last_target_ms = now_ms;

        if (target.valid)
        {
            /* 简单比例控制：误差单位为像素。后续可替换为 PID。 */
            g_pan -= (float)target.x_error * 0.015f;
            g_tilt += (float)target.y_error * 0.015f;

            g_pan = clamp_f32(g_pan, GIMBAL_PAN_MIN_DEG, GIMBAL_PAN_MAX_DEG);
            g_tilt = clamp_f32(g_tilt, GIMBAL_TILT_MIN_DEG, GIMBAL_TILT_MAX_DEG);

            if (g_gimbal != 0 && g_gimbal->set_angle_deg != 0)
            {
                g_gimbal->set_angle_deg(g_pan, g_tilt);
            }
        }
    }

    /* 视觉丢失超过 1s，云台保持当前位置。你也可以改成自动回中。 */
    if ((now_ms - g_last_target_ms) > 1000U)
    {
        /* keep last angle */
    }
#else
    (void)now_ms;
#endif
}
