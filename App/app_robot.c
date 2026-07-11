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
#include <string.h>

#if APP_ENABLE_BLUETOOTH_CONTROL && APP_ENABLE_VISION_TARGET
#error "Bluetooth control and vision protocol cannot share the USART2 RX buffer at the same time."
#endif

#define BLUETOOTH_CMD_BUFFER_SIZE 16U

static robot_mode_t g_mode = ROBOT_MODE_STOP;
static uint32_t g_last_control_ms = 0U;
static uint32_t g_last_telemetry_ms = 0U;
static uint32_t g_last_led_ms = 0U;
static int32_t g_telemetry_left_encoder_accum = 0;
static int32_t g_telemetry_right_encoder_accum = 0;
static uint32_t g_control_max_dt_ms = 0U;
static uint32_t g_control_overrun_count = 0U;
static char g_bluetooth_cmd_buffer[BLUETOOTH_CMD_BUFFER_SIZE];
static uint8_t g_bluetooth_cmd_len = 0U;

#if APP_ENABLE_MOTOR_SPEED_TEST_DEMO
typedef enum
{
    MOTOR_TEST_PHASE_WAIT = 0,
    MOTOR_TEST_PHASE_BOTH,
    MOTOR_TEST_PHASE_STOP_AFTER_BOTH,
    MOTOR_TEST_PHASE_LEFT,
    MOTOR_TEST_PHASE_STOP_AFTER_LEFT,
    MOTOR_TEST_PHASE_RIGHT,
    MOTOR_TEST_PHASE_STOP_AFTER_RIGHT
} motor_test_phase_t;

static motor_test_phase_t g_motor_test_phase = MOTOR_TEST_PHASE_WAIT;
static uint32_t g_motor_test_phase_start_ms = 0U;
static uint32_t g_motor_test_cycle = 0U;
static int16_t g_motor_test_left_cmd = 0;
static int16_t g_motor_test_right_cmd = 0;
static int32_t g_motor_test_left_abs_sum = 0;
static int32_t g_motor_test_right_abs_sum = 0;
static int32_t g_telemetry_left_encoder_abs_accum = 0;
static int32_t g_telemetry_right_encoder_abs_accum = 0;

static int32_t abs_i32_local(int32_t value)
{
    return (value < 0) ? -value : value;
}

static const char *motor_test_phase_name(motor_test_phase_t phase)
{
    switch (phase)
    {
    case MOTOR_TEST_PHASE_WAIT:
        return "WAIT";
    case MOTOR_TEST_PHASE_BOTH:
        return "BOTH";
    case MOTOR_TEST_PHASE_STOP_AFTER_BOTH:
        return "STOP_BOTH";
    case MOTOR_TEST_PHASE_LEFT:
        return "LEFT";
    case MOTOR_TEST_PHASE_STOP_AFTER_LEFT:
        return "STOP_LEFT";
    case MOTOR_TEST_PHASE_RIGHT:
        return "RIGHT";
    case MOTOR_TEST_PHASE_STOP_AFTER_RIGHT:
        return "STOP_RIGHT";
    default:
        return "UNKNOWN";
    }
}

static uint32_t motor_test_phase_duration_ms(motor_test_phase_t phase)
{
    switch (phase)
    {
    case MOTOR_TEST_PHASE_WAIT:
        return MOTOR_TEST_START_DELAY_MS;
    case MOTOR_TEST_PHASE_BOTH:
    case MOTOR_TEST_PHASE_LEFT:
    case MOTOR_TEST_PHASE_RIGHT:
        return MOTOR_TEST_RUN_MS;
    default:
        return MOTOR_TEST_STOP_MS;
    }
}

static void motor_test_enter_phase(motor_test_phase_t phase, uint32_t now)
{
    g_motor_test_phase = phase;
    g_motor_test_phase_start_ms = now;
    g_motor_test_left_abs_sum = 0;
    g_motor_test_right_abs_sum = 0;

    switch (phase)
    {
    case MOTOR_TEST_PHASE_BOTH:
        g_motor_test_cycle++;
        g_motor_test_left_cmd = MOTOR_TEST_PWM;
        g_motor_test_right_cmd = MOTOR_TEST_PWM;
        Chassis_SetPWM(g_motor_test_left_cmd, g_motor_test_right_cmd);
        break;

    case MOTOR_TEST_PHASE_LEFT:
        g_motor_test_left_cmd = MOTOR_TEST_PWM;
        g_motor_test_right_cmd = 0;
        Chassis_SetPWM(g_motor_test_left_cmd, g_motor_test_right_cmd);
        break;

    case MOTOR_TEST_PHASE_RIGHT:
        g_motor_test_left_cmd = 0;
        g_motor_test_right_cmd = MOTOR_TEST_PWM;
        Chassis_SetPWM(g_motor_test_left_cmd, g_motor_test_right_cmd);
        break;

    default:
        g_motor_test_left_cmd = 0;
        g_motor_test_right_cmd = 0;
        Chassis_StopCoast();
        break;
    }

    printf("MT_PHASE CYCLE=%lu PH=%s LPWM=%d RPWM=%d\r\n",
           (unsigned long)g_motor_test_cycle,
           motor_test_phase_name(g_motor_test_phase),
           (int)g_motor_test_left_cmd,
           (int)g_motor_test_right_cmd);
}

static motor_test_phase_t motor_test_next_phase(motor_test_phase_t phase)
{
    switch (phase)
    {
    case MOTOR_TEST_PHASE_WAIT:
        return MOTOR_TEST_PHASE_BOTH;
    case MOTOR_TEST_PHASE_BOTH:
        return MOTOR_TEST_PHASE_STOP_AFTER_BOTH;
    case MOTOR_TEST_PHASE_STOP_AFTER_BOTH:
        return MOTOR_TEST_PHASE_LEFT;
    case MOTOR_TEST_PHASE_LEFT:
        return MOTOR_TEST_PHASE_STOP_AFTER_LEFT;
    case MOTOR_TEST_PHASE_STOP_AFTER_LEFT:
        return MOTOR_TEST_PHASE_RIGHT;
    case MOTOR_TEST_PHASE_RIGHT:
        return MOTOR_TEST_PHASE_STOP_AFTER_RIGHT;
    case MOTOR_TEST_PHASE_STOP_AFTER_RIGHT:
    default:
        return MOTOR_TEST_PHASE_BOTH;
    }
}

static void motor_test_update(uint32_t now)
{
    uint32_t elapsed = now - g_motor_test_phase_start_ms;

    if (elapsed >= motor_test_phase_duration_ms(g_motor_test_phase))
    {
        motor_test_enter_phase(motor_test_next_phase(g_motor_test_phase), now);
    }
}
#endif

#if APP_ENABLE_BLUETOOTH_CONTROL
static void bluetooth_execute_command(const char *command)
{
    if (strcmp(command, "START") == 0 || strcmp(command, "GO") == 0 || strcmp(command, "1") == 0)
    {
        AppRobot_SetMode(ROBOT_MODE_LINE_FOLLOW);
        BSP_DebugUART_SendString("ACK START\r\n");
    }
    else if (strcmp(command, "STOP") == 0 || strcmp(command, "0") == 0)
    {
        AppRobot_SetMode(ROBOT_MODE_STOP);
        BSP_DebugUART_SendString("ACK STOP\r\n");
    }
    else
    {
        BSP_DebugUART_SendString("ERR CMD USE START/STOP OR 1/0\r\n");
    }
}

static void bluetooth_command_poll(void)
{
    char ch;

    while (BSP_DebugUART_ReadCharNonBlocking(&ch))
    {
        if ((ch == '1' || ch == '0') && g_bluetooth_cmd_len == 0U)
        {
            g_bluetooth_cmd_buffer[0] = ch;
            g_bluetooth_cmd_buffer[1] = '\0';
            bluetooth_execute_command(g_bluetooth_cmd_buffer);
            continue;
        }

        if (ch == '\r' || ch == '\n')
        {
            if (g_bluetooth_cmd_len > 0U)
            {
                g_bluetooth_cmd_buffer[g_bluetooth_cmd_len] = '\0';
                bluetooth_execute_command(g_bluetooth_cmd_buffer);
                g_bluetooth_cmd_len = 0U;
            }
            continue;
        }

        if (ch >= 'a' && ch <= 'z')
        {
            ch = (char)(ch - ('a' - 'A'));
        }

        if (ch == ' ' && g_bluetooth_cmd_len == 0U)
        {
            continue;
        }

        if (g_bluetooth_cmd_len < (BLUETOOTH_CMD_BUFFER_SIZE - 1U))
        {
            g_bluetooth_cmd_buffer[g_bluetooth_cmd_len++] = ch;
        }
        else
        {
            g_bluetooth_cmd_len = 0U;
            BSP_DebugUART_SendString("ERR CMD TOO LONG\r\n");
        }
    }
}
#endif

/**
 * @brief 输出当前调试遥测。
 *
 * 当前 printf 通过 USART2 TXE 中断队列非阻塞发送。遥测周期为 500ms，
 * 保留完整字段的同时避免 9600 波特率发送过程占用主控制循环。
 */
static void telemetry_output(void)
{
    chassis_state_t ch = Chassis_GetState();
    line_follow_debug_t dbg;
#if APP_ENABLE_MOTOR_SPEED_TEST_DEMO
    int32_t diff_percent;

    if (g_mode == ROBOT_MODE_MOTOR_TEST)
    {
        if (g_motor_test_right_abs_sum > 0)
        {
            diff_percent = ((g_motor_test_left_abs_sum - g_motor_test_right_abs_sum) * 100L) /
                           g_motor_test_right_abs_sum;
        }
        else
        {
            diff_percent = 0;
        }

        printf("MT CYCLE=%lu PH=%s T=%lu LPWM=%d RPWM=%d LE=%ld RE=%ld LA=%ld RA=%ld LSUM=%ld RSUM=%ld DIFF=%ld%% DT=%lu OV=%lu TD=%lu\r\n",
               (unsigned long)g_motor_test_cycle,
               motor_test_phase_name(g_motor_test_phase),
               (unsigned long)(BSP_GetTickMs() - g_motor_test_phase_start_ms),
               (int)g_motor_test_left_cmd,
               (int)g_motor_test_right_cmd,
               (long)g_telemetry_left_encoder_accum,
               (long)g_telemetry_right_encoder_accum,
               (long)g_telemetry_left_encoder_abs_accum,
               (long)g_telemetry_right_encoder_abs_accum,
               (long)g_motor_test_left_abs_sum,
               (long)g_motor_test_right_abs_sum,
               (long)diff_percent,
               (unsigned long)g_control_max_dt_ms,
               (unsigned long)g_control_overrun_count,
               (unsigned long)BSP_DebugUART_GetTxDroppedCount());

        g_telemetry_left_encoder_accum = 0;
        g_telemetry_right_encoder_accum = 0;
        g_telemetry_left_encoder_abs_accum = 0;
        g_telemetry_right_encoder_abs_accum = 0;
        g_control_max_dt_ms = 0U;
        g_control_overrun_count = 0U;
        return;
    }
#endif

    dbg = AppLineFollow_GetDebug();

    printf("M=%d S=%d RAW=0x%02X ERR=%d LPWM=%d RPWM=%d LE=%ld RE=%ld C=%u DIR=%d SUM=%u DT=%lu OV=%lu F=%u TD=%lu ARM=%u RM=%lu RC=%lu ST=%lu RCM=%lu RLM=%lu TR=%u\r\n",
           (int)g_mode,
           (int)dbg.state,
           (unsigned int)dbg.tracker.raw_bits,
           (int)dbg.tracker.position_error,
           (int)ch.left_pwm,
           (int)ch.right_pwm,
           (long)g_telemetry_left_encoder_accum,
           (long)g_telemetry_right_encoder_accum,
           (unsigned int)dbg.corner_count,
           (int)dbg.corner_dir,
           (unsigned int)dbg.corner_encoder_sum,
           (unsigned long)g_control_max_dt_ms,
           (unsigned long)g_control_overrun_count,
           (unsigned int)dbg.sensor_fault,
           (unsigned long)BSP_DebugUART_GetTxDroppedCount(),
           (unsigned int)dbg.corner_armed,
           (unsigned long)dbg.corner_rearm_ms,
           (unsigned long)dbg.corner_rearm_center_ms,
           (unsigned long)dbg.state_time_ms,
           (unsigned long)dbg.recover_center_ms,
           (unsigned long)dbg.recover_lost_time_ms,
           (unsigned int)dbg.transition_reason);

    g_telemetry_left_encoder_accum = 0;
    g_telemetry_right_encoder_accum = 0;
    g_control_max_dt_ms = 0U;
    g_control_overrun_count = 0U;
#if APP_ENABLE_MOTOR_SPEED_TEST_DEMO
    g_telemetry_left_encoder_abs_accum = 0;
    g_telemetry_right_encoder_abs_accum = 0;
#endif
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

#if APP_ENABLE_MOTOR_SPEED_TEST_DEMO
    printf("\r\n[BOOT] STM32F103 motor speed test demo\r\n");
    printf("[BOOT] Lift the car. Set APP_ENABLE_MOTOR_SPEED_TEST_DEMO=0 to return line follow.\r\n");
#else
    printf("\r\n[BOOT] STM32F103 rectangle line car TELEMETRY V3\r\n");
#endif

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

    g_last_control_ms = BSP_GetTickMs();
    g_last_telemetry_ms = g_last_control_ms;
    g_last_led_ms = g_last_control_ms;
    g_control_max_dt_ms = 0U;
    g_control_overrun_count = 0U;

#if APP_ENABLE_MOTOR_SPEED_TEST_DEMO
    g_mode = ROBOT_MODE_MOTOR_TEST;
    motor_test_enter_phase(MOTOR_TEST_PHASE_WAIT, g_last_control_ms);
#else
#if APP_ENABLE_BLUETOOTH_CONTROL
    g_mode = ROBOT_MODE_STOP;
    g_bluetooth_cmd_len = 0U;
    Chassis_StopCoast();
    BSP_DebugUART_SendString("READY STOP CMD=START/STOP OR 1/0\r\n");
#else
    g_mode = ROBOT_MODE_LINE_FOLLOW;
#endif
#endif
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

#if APP_ENABLE_BLUETOOTH_CONTROL
    /* 每次主循环都处理命令，STOP 不需要等待下一个 10ms 控制周期。 */
    bluetooth_command_poll();
#endif

    if ((now - g_last_control_ms) >= APP_CONTROL_PERIOD_MS)
    {
        uint32_t dt = now - g_last_control_ms;
        g_last_control_ms = now;

        if (dt > g_control_max_dt_ms)
        {
            g_control_max_dt_ms = dt;
        }
        if (dt >= APP_CONTROL_OVERRUN_WARN_MS)
        {
            g_control_overrun_count++;
        }

        /* 编码器采样与控制周期同步。当前仍是 PWM 开环循迹，
         * 遥测只读取控制周期累积值，不改变编码器采样节拍。
         */
        Chassis_UpdateEncoder();
        {
            chassis_state_t ch = Chassis_GetState();
            g_telemetry_left_encoder_accum += ch.left_encoder_delta;
            g_telemetry_right_encoder_accum += ch.right_encoder_delta;
#if APP_ENABLE_MOTOR_SPEED_TEST_DEMO
            g_telemetry_left_encoder_abs_accum += abs_i32_local((int32_t)ch.left_encoder_delta);
            g_telemetry_right_encoder_abs_accum += abs_i32_local((int32_t)ch.right_encoder_delta);
            if (g_mode == ROBOT_MODE_MOTOR_TEST)
            {
                g_motor_test_left_abs_sum += abs_i32_local((int32_t)ch.left_encoder_delta);
                g_motor_test_right_abs_sum += abs_i32_local((int32_t)ch.right_encoder_delta);
            }
#endif
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

        case ROBOT_MODE_MOTOR_TEST:
#if APP_ENABLE_MOTOR_SPEED_TEST_DEMO
            motor_test_update(now);
#else
            Chassis_StopCoast();
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
 * STOP 立即空转停车；从其他模式进入循迹时先复位状态机，再等待 START 稳定期。
 */
void AppRobot_SetMode(robot_mode_t mode)
{
    if (mode == ROBOT_MODE_STOP)
    {
        g_mode = ROBOT_MODE_STOP;
        Chassis_StopCoast();
        return;
    }

    if (mode == ROBOT_MODE_LINE_FOLLOW && g_mode != ROBOT_MODE_LINE_FOLLOW)
    {
        AppLineFollow_Reset();
    }

    g_mode = mode;
}

/**
 * @brief 返回当前机器人模式。
 */
robot_mode_t AppRobot_GetMode(void)
{
    return g_mode;
}
