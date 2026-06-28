#include "app_line_follow.h"
#include "app_config.h"
#include "pid.h"
#include "chassis.h"
#include "common_types.h"

static const tracker8_driver_t *g_tracker = 0;
static pid_t g_line_pid;
static line_follow_debug_t g_debug;

void AppLineFollow_Init(const tracker8_driver_t *tracker_driver)
{
    g_tracker = tracker_driver;
    if (g_tracker != 0 && g_tracker->init != 0)
    {
        g_tracker->init();
    }

    PID_Init(&g_line_pid,
             LINE_PID_KP,
             LINE_PID_KI,
             LINE_PID_KD,
             -LINE_PID_OUT_LIMIT,
             LINE_PID_OUT_LIMIT,
             -LINE_PID_INTEGRAL_LIMIT,
             LINE_PID_INTEGRAL_LIMIT);

    g_debug.left_pwm = 0;
    g_debug.right_pwm = 0;
    g_debug.correction = 0;
}

void AppLineFollow_Update(uint32_t dt_ms)
{
    tracker8_sample_t sample;
    float correction_f;
    int16_t correction;
    int16_t left;
    int16_t right;

    if (g_tracker == 0 || g_tracker->read == 0)
    {
        Chassis_StopCoast();
        return;
    }

    sample = g_tracker->read();
    g_debug.tracker = sample;

    if (sample.status == TRACKER_STATUS_LOST)
    {
        PID_Reset(&g_line_pid);
        if (sample.last_valid_error >= 0)
        {
            left = TRACKER_LOST_SEARCH_PWM;
            right = (int16_t)(-TRACKER_LOST_SEARCH_PWM);
        }
        else
        {
            left = (int16_t)(-TRACKER_LOST_SEARCH_PWM);
            right = TRACKER_LOST_SEARCH_PWM;
        }
        correction = 0;
    }
    else
    {
        correction_f = PID_Update(&g_line_pid, 0.0f, (float)sample.position_error, (float)dt_ms / 1000.0f);
        correction = (int16_t)correction_f;

        /* error 左负右正。小车偏左时 correction 为正，左轮加速右轮减速，向右修正。 */
        left = (int16_t)(LINE_BASE_PWM + correction);
        right = (int16_t)(LINE_BASE_PWM - correction);

        left = clamp_i16(left, -LINE_PWM_LIMIT, LINE_PWM_LIMIT);
        right = clamp_i16(right, -LINE_PWM_LIMIT, LINE_PWM_LIMIT);
    }

    g_debug.left_pwm = left;
    g_debug.right_pwm = right;
    g_debug.correction = correction;

    Chassis_SetPWM(left, right);
}

line_follow_debug_t AppLineFollow_GetDebug(void)
{
    return g_debug;
}
