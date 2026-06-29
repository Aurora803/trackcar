/**
 * @file gimbal_servo.c
 * @brief 舵机云台适配层实现。
 * @layer Components
 *
 * 本层负责角度限幅并调用 BSP_Servo 输出。当前工程默认关闭云台输出，
 * 因为现有电机 PWM、调试串口和循迹引脚已经占用关键资源。
 */
#include "gimbal_servo.h"
#include "bsp_servo.h"
#include "app_config.h"
#include "common_types.h"

static float g_pan = GIMBAL_PAN_CENTER_DEG;
static float g_tilt = GIMBAL_TILT_CENTER_DEG;

/**
 * @brief 初始化底层舵机输出并回中。
 */
void GimbalServo_Init(void)
{
    BSP_Servo_Init();
    GimbalServo_Center();
}

/**
 * @brief 设置云台角度并做软件限幅。
 */
void GimbalServo_SetAngleDeg(float pan_deg, float tilt_deg)
{
    g_pan = clamp_f32(pan_deg, GIMBAL_PAN_MIN_DEG, GIMBAL_PAN_MAX_DEG);
    g_tilt = clamp_f32(tilt_deg, GIMBAL_TILT_MIN_DEG, GIMBAL_TILT_MAX_DEG);

    BSP_Servo_SetAngleDeg(1U, g_pan);
    BSP_Servo_SetAngleDeg(2U, g_tilt);
}

/**
 * @brief 将云台移动到配置的中心角度。
 */
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

/**
 * @brief 返回云台驱动接口对象。
 */
const gimbal_driver_t *GimbalServo_GetDriver(void)
{
    return &g_gimbal_servo_driver;
}
