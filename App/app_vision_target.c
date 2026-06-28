#include "app_vision_target.h"
#include "app_config.h"
#include "vision_protocol.h"
#include "common_types.h"

static const gimbal_driver_t *g_gimbal = 0;
static float g_pan = GIMBAL_PAN_CENTER_DEG;
static float g_tilt = GIMBAL_TILT_CENTER_DEG;
static uint32_t g_last_target_ms = 0U;

void AppVisionTarget_Init(const gimbal_driver_t *gimbal_driver)
{
    VisionProtocol_Init();
    g_gimbal = gimbal_driver;

    if (g_gimbal != 0 && g_gimbal->init != 0)
    {
        g_gimbal->init();
    }
}

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
