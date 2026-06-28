#include "app_robot.h"
#include "app_config.h"
#include "bsp_gpio.h"
#include "bsp_systick.h"
#include "bsp_uart.h"
#include "tb6612_motor.h"
#include "yahboom_tracker8_io.h"
#include "chassis.h"
#include "app_line_follow.h"
#if APP_ENABLE_VISION_TARGET
#include "app_vision_target.h"
#if APP_ENABLE_GIMBAL_SERVO
#include "gimbal_servo.h"
#endif
#endif
#include <stdio.h>

static robot_mode_t g_mode = ROBOT_MODE_LINE_FOLLOW;
static uint32_t g_last_control_ms = 0U;
static uint32_t g_last_telemetry_ms = 0U;
static uint32_t g_last_led_ms = 0U;

static void telemetry_output(void)
{
    line_follow_debug_t dbg = AppLineFollow_GetDebug();
    chassis_state_t ch = Chassis_GetState();

    printf("M=%d S=%d RAW=0x%02X ERR=%d LPWM=%d RPWM=%d LE=%d RE=%d C=%u DIR=%d SUM=%u\r\n",
           (int)g_mode,
           (int)dbg.state,
           dbg.tracker.raw_bits,
           dbg.tracker.position_error,
           ch.left_pwm,
           ch.right_pwm,
           ch.left_encoder_delta,
           ch.right_encoder_delta,
           (unsigned int)dbg.corner_count,
           (int)dbg.corner_dir,
           (unsigned int)dbg.corner_encoder_sum);
}

void AppRobot_Init(void)
{
    BSP_GPIO_InitAll();
    BSP_SysTick_Init();
    BSP_UART_Init();

    printf("\r\n[BOOT] STM32F103 rectangle line car demo\r\n");

    Chassis_Init(TB6612_GetDriver());
    AppLineFollow_Init(YahboomTracker8IO_GetDriver());
#if APP_ENABLE_VISION_TARGET
#if APP_ENABLE_GIMBAL_SERVO
    AppVisionTarget_Init(GimbalServo_GetDriver());
#else
    /* 允许先接入视觉协议但不输出云台 PWM，便于分阶段调试串口协议。 */
    AppVisionTarget_Init(0);
#endif
#endif

    g_mode = ROBOT_MODE_LINE_FOLLOW;
    g_last_control_ms = BSP_GetTickMs();
    g_last_telemetry_ms = g_last_control_ms;
    g_last_led_ms = g_last_control_ms;
}

void AppRobot_Task(void)
{
    uint32_t now = BSP_GetTickMs();

    if ((now - g_last_control_ms) >= APP_CONTROL_PERIOD_MS)
    {
        uint32_t dt = now - g_last_control_ms;
        g_last_control_ms = now;

        /* 编码器采样与控制周期同步。当前仍是 PWM 开环循迹，
         * 但后续接速度闭环时不会再遇到 50ms 遥测周期数据滞后的问题。
         */
        Chassis_UpdateEncoder();

        switch (g_mode)
        {
        case ROBOT_MODE_LINE_FOLLOW:
            AppLineFollow_Update(dt);
            break;

        case ROBOT_MODE_TARGET_TRACK:
            /* 目标跟踪模式预留给 OpenMV/树莓派 + 云台。
             * 当前默认配置未启用视觉/云台时，底盘保持空转停止，避免误以为会继续循迹。
             */
            Chassis_StopCoast();
#if APP_ENABLE_VISION_TARGET
            AppVisionTarget_Update(now);
#endif
            break;

        case ROBOT_MODE_STOP:
        default:
            Chassis_StopCoast();
            break;
        }
    }

    if ((now - g_last_telemetry_ms) >= APP_TELEMETRY_PERIOD_MS)
    {
        g_last_telemetry_ms = now;
        telemetry_output();
    }

    if ((now - g_last_led_ms) >= 500U)
    {
        g_last_led_ms = now;
        BSP_LED_Toggle();
    }
}

void AppRobot_SetMode(robot_mode_t mode)
{
    g_mode = mode;
    if (g_mode == ROBOT_MODE_STOP)
    {
        Chassis_StopCoast();
    }
}

robot_mode_t AppRobot_GetMode(void)
{
    return g_mode;
}
