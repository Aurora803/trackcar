/**
 * @file app_robot.c
 * @brief 应用层总调度实现。
 * @layer App
 *
 * 本模块负责把 BSP 初始化、底盘驱动和循迹任务串起来。
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
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BLUETOOTH_CMD_BUFFER_SIZE 16U

/* 以下时间戳均来自同一个 1 ms tick。使用无符号减法比较间隔，
 * 因而即使 tick 在约 49.7 天后回绕，周期调度仍然正确。
 */
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

#if APP_ENABLE_BLUETOOTH_CONTROL
/**
 * @brief 执行已完成且已规范化为大写的蓝牙命令。
 *
 * 单字符 1/0 在收到时立即执行；START、STOP 和 GO 由换行符提交。
 * 命令执行仍在主循环中完成，不会在 USART 中断中改变电机状态。
 */
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
            /* 手机蓝牙串口常把按钮配置成不带换行的单字符，因此不能等行结束符。 */
            g_bluetooth_cmd_buffer[0] = ch;
            g_bluetooth_cmd_buffer[1] = '\0';
            bluetooth_execute_command(g_bluetooth_cmd_buffer);
            continue;
        }

        if (ch == '\r' || ch == '\n')
        {
            /* CRLF 会在 CR 提交后留下一个空 LF；空缓冲时直接忽略即可。 */
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
            /* 溢出后丢弃当前整条命令，避免截断的前缀被误当作有效命令执行。 */
            g_bluetooth_cmd_len = 0U;
            BSP_DebugUART_SendString("ERR CMD TOO LONG\r\n");
        }
    }
}
#endif

/**
 * @brief 输出当前调试遥测。
 *
 * 当前 printf 通过 USART2 TXE 中断队列非阻塞发送。遥测周期由
 * APP_TELEMETRY_PERIOD_MS 配置，
 * 保留完整字段的同时避免 9600 波特率发送过程占用主控制循环。
 */
static void telemetry_output(void)
{
    chassis_state_t ch = Chassis_GetState();
    line_follow_debug_t dbg;

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
}

/**
 * @brief 初始化机器人应用。
 *
 * 初始化顺序：
 * 1. GPIO/SysTick/UART BSP；
 * 2. 底盘层绑定 TB6612 motor_driver_t；
 * 3. 循迹层绑定 Yahboom 8 路 tracker8_driver_t；
 */
void AppRobot_Init(void)
{
    BSP_GPIO_InitAll();
    BSP_SysTick_Init();
    BSP_UART_Init();

    printf("\r\n[BOOT] STM32F103 rectangle line car TELEMETRY V3\r\n");

    Chassis_Init(TB6612_GetDriver());
    AppLineFollow_Init(YahboomTracker8IO_GetDriver());

    g_last_control_ms = BSP_GetTickMs();
    g_last_telemetry_ms = g_last_control_ms;
    g_last_led_ms = g_last_control_ms;
    g_control_max_dt_ms = 0U;
    g_control_overrun_count = 0U;

#if APP_ENABLE_BLUETOOTH_CONTROL
    g_mode = ROBOT_MODE_STOP;
    g_bluetooth_cmd_len = 0U;
    Chassis_StopCoast();
    BSP_DebugUART_SendString("READY STOP CMD=START/STOP OR 1/0\r\n");
#else
    g_mode = ROBOT_MODE_LINE_FOLLOW;
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

        /* 不补跑遗漏的多个控制周期：使用真实 dt 运行一次，防止主循环恢复后
         * 连续执行多次已过期的电机命令。DT/OV 同时保留给遥测定位卡顿来源。
         */
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
        }

        switch (g_mode)
        {
        case ROBOT_MODE_LINE_FOLLOW:
            AppLineFollow_Update(dt);
            break;

        case ROBOT_MODE_STOP:
        default:
            Chassis_StopCoast();
            break;
        }
    }

    if ((now - g_last_telemetry_ms) >= APP_TELEMETRY_PERIOD_MS)
    {
        /* 遥测也只补发最新快照，不积压历史日志，避免低波特率进一步拖慢控制。 */
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
        /* 每次重新启动循迹都清除 PID、转角冷却和丢线历史，避免停车前的
         * 方向偏差影响下一次起步。
         */
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
