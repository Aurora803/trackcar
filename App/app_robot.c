/**
 * @file app_robot.c
 * @brief 应用层总调度实现。
 * @layer App
 *
 * 本模块负责把 BSP 初始化、底盘驱动、循迹任务、视觉预留任务串起来。
 * 循迹状态机和 PID 不在这里展开，避免 main.c 或总调度层堆积控制细节。
 */
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
#include <stdint.h>
#include <stdio.h>

static robot_mode_t g_mode = ROBOT_MODE_LINE_FOLLOW;
static uint32_t g_last_control_ms = 0U;
static uint32_t g_last_telemetry_ms = 0U;
static uint32_t g_last_led_ms = 0U;
static int32_t g_telemetry_left_encoder_accum = 0;
static int32_t g_telemetry_right_encoder_accum = 0;

/**
 * @brief 输出当前调试遥测。
 *
 * 当前 printf 通过 USART2 阻塞发送。50ms 输出一行适合调试，但正式高速闭环
 * 时应降低频率或改为非阻塞发送，避免串口占用主循环时间。
 */
static void telemetry_output(void)
{
    line_follow_debug_t dbg = AppLineFollow_GetDebug();
    chassis_state_t ch = Chassis_GetState();

    printf("M=%d S=%d RAW=0x%02X ERR=%d LPWM=%d RPWM=%d LE=%ld RE=%ld C=%u DIR=%d SUM=%u\r\n",
           (int)g_mode,
           (int)dbg.state,
           dbg.tracker.raw_bits,
           dbg.tracker.position_error,
           ch.left_pwm,
           ch.right_pwm,
           (long)g_telemetry_left_encoder_accum,
           (long)g_telemetry_right_encoder_accum,
           (unsigned int)dbg.corner_count,
           (int)dbg.corner_dir,
           (unsigned int)dbg.corner_encoder_sum);

    g_telemetry_left_encoder_accum = 0;
    g_telemetry_right_encoder_accum = 0;
}

/**
 * @brief 初始化机器人应用。
 *
 * 初始化顺序：
 * 1. GPIO/SysTick/UART BSP；
 * 2. 底盘层绑定 TB6612 motor_driver_t；
 * 3. 循迹层绑定 Yahboom 8 路 tracker8_driver_t；
 * 4. 可选视觉/云台按宏开关预留。
 */
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

/**
 * @brief 机器人主循环任务。
 *
 * 该函数应在 while(1) 中尽可能频繁调用。实际控制周期由
 * APP_CONTROL_PERIOD_MS 决定，中断只提供毫秒 tick，不直接跑控制算法。
 */
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
        {
            chassis_state_t ch = Chassis_GetState();
            g_telemetry_left_encoder_accum += ch.left_encoder_delta;
            g_telemetry_right_encoder_accum += ch.right_encoder_delta;
        }

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

/**
 * @brief 切换机器人模式。
 *
 * 当前只在 STOP 模式切入时立即空转停止；其他模式的状态恢复由各自任务处理。
 */
void AppRobot_SetMode(robot_mode_t mode)
{
    g_mode = mode;
    if (g_mode == ROBOT_MODE_STOP)
    {
        Chassis_StopCoast();
    }
}

/**
 * @brief 返回当前机器人模式。
 */
robot_mode_t AppRobot_GetMode(void)
{
    return g_mode;
}
