#include "gimbal_servo.h"
#include "bsp_servo.h"
#include "app_config.h"
#include "common_types.h"

static float g_pan = GIMBAL_PAN_CENTER_DEG;
static float g_tilt = GIMBAL_TILT_CENTER_DEG;

void GimbalServo_Init(void)
{
    BSP_Servo_Init();
    GimbalServo_Center();
}

void GimbalServo_SetAngleDeg(float pan_deg, float tilt_deg)
{
    g_pan = clamp_f32(pan_deg, GIMBAL_PAN_MIN_DEG, GIMBAL_PAN_MAX_DEG);
    g_tilt = clamp_f32(tilt_deg, GIMBAL_TILT_MIN_DEG, GIMBAL_TILT_MAX_DEG);

    BSP_Servo_SetAngleDeg(1U, g_pan);
    BSP_Servo_SetAngleDeg(2U, g_tilt);
}

void GimbalServo_Center(void)
{
    GimbalServo_SetAngleDeg(GIMBAL_PAN_CENTER_DEG, GIMBAL_TILT_CENTER_DEG);
}

static const gimbal_driver_t g_gimbal_servo_driver =
{
    GimbalServo_Init,
    GimbalServo_SetAngleDeg,
    GimbalServo_Center
};

const gimbal_driver_t *GimbalServo_GetDriver(void)
{
    return &g_gimbal_servo_driver;
}
