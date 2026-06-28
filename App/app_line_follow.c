#include "app_line_follow.h"
#include "app_config.h"
#include "pid.h"
#include "chassis.h"
#include "common_types.h"

static const tracker8_driver_t *g_tracker = 0;
static pid_t g_line_pid;
static line_follow_debug_t g_debug;

static line_follow_state_t g_state = LINE_STATE_START;
static uint32_t g_state_time_ms = 0U;
static uint32_t g_lost_time_ms = 0U;
static uint32_t g_reacquire_time_ms = 0U;
static int8_t g_corner_dir = 0;
static int32_t g_corner_encoder_sum = 0;
static uint16_t g_corner_count = 0U;
static int8_t g_corner_candidate_dir = 0;
static uint8_t g_corner_candidate_count = 0U;

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint8_t count_bits4(uint8_t value)
{
    uint8_t count = 0U;
    uint8_t i;

    for (i = 0U; i < 4U; ++i)
    {
        if ((value & (uint8_t)(1U << i)) != 0U)
        {
            count++;
        }
    }

    return count;
}

static uint8_t tracker_is_valid(const tracker8_sample_t *sample)
{
    return (sample->status == TRACKER_STATUS_OK || sample->status == TRACKER_STATUS_CROSS) ? 1U : 0U;
}

static uint8_t tracker_center_found(const tracker8_sample_t *sample)
{
    uint8_t center_bits;

    if (sample->status != TRACKER_STATUS_OK)
    {
        return 0U;
    }

    center_bits = (uint8_t)(sample->raw_bits & 0x18U); /* S4/S5 */
    if (center_bits != 0U && abs_i32((int32_t)sample->position_error) <= LINE_RECOVER_ERROR_THRESHOLD)
    {
        return 1U;
    }

    return 0U;
}

static void reset_corner_debounce(void)
{
    g_corner_candidate_dir = 0;
    g_corner_candidate_count = 0U;
}

static int8_t detect_corner_dir(const tracker8_sample_t *sample)
{
#if !APP_RECTANGLE_TRACK_ONLY
    /* 非矩形赛道先不自动进入 90 度转角状态，避免普通岔线/噪声触发专用逻辑。 */
    (void)sample;
    return 0;
#else
    uint8_t left_count;
    uint8_t right_count;

    if (!tracker_is_valid(sample))
    {
        if (abs_i32((int32_t)sample->last_valid_error) >= LINE_CORNER_ERROR_THRESHOLD)
        {
            return (sample->last_valid_error > 0) ? 1 : -1;
        }
        return 0;
    }

    left_count = count_bits4((uint8_t)(sample->raw_bits & 0x0FU));
    right_count = count_bits4((uint8_t)((sample->raw_bits >> 4) & 0x0FU));

    /* 矩形赛道直角弯：一侧 3~4 路连续压线，另一侧很少压线。 */
    if (left_count >= 3U && right_count <= 1U)
    {
        return -1;
    }
    if (right_count >= 3U && left_count <= 1U)
    {
        return 1;
    }

    /* 宽横线/全黑线：矩形闭环没有岔路，按默认方向转 90° 弯。 */
    if (sample->status == TRACKER_STATUS_CROSS)
    {
        return (RECT_DEFAULT_CORNER_DIR >= 0) ? 1 : -1;
    }

    return 0;
#endif
}

static uint8_t corner_dir_confirmed(int8_t dir)
{
    if (dir == 0)
    {
        reset_corner_debounce();
        return 0U;
    }

    if (g_corner_candidate_dir == dir)
    {
        if (g_corner_candidate_count < 255U)
        {
            g_corner_candidate_count++;
        }
    }
    else
    {
        g_corner_candidate_dir = dir;
        g_corner_candidate_count = 1U;
    }

    /* 单帧边缘反光或杂线只会成为候选，不会立刻让小车进入转角状态。 */
    return (g_corner_candidate_count >= LINE_CORNER_DEBOUNCE_COUNT) ? 1U : 0U;
}

static void enter_state(line_follow_state_t next_state)
{
    g_state = next_state;
    g_state_time_ms = 0U;
    g_lost_time_ms = 0U;
    g_reacquire_time_ms = 0U;
    reset_corner_debounce();

    if (next_state == LINE_STATE_FOLLOW || next_state == LINE_STATE_RECOVER)
    {
        PID_Reset(&g_line_pid);
    }
}

static void enter_corner(int8_t dir)
{
    if (dir == 0)
    {
        dir = (RECT_DEFAULT_CORNER_DIR >= 0) ? 1 : -1;
    }

    g_corner_dir = dir;
    g_corner_encoder_sum = 0;
    PID_Reset(&g_line_pid);
    enter_state(LINE_STATE_CORNER);
}

static int16_t calc_dynamic_base_pwm(int16_t position_error)
{
    int32_t e = abs_i32((int32_t)position_error);

    if (e <= 150)
    {
        return LINE_BASE_PWM_FAST;
    }
    if (e <= 550)
    {
        return LINE_BASE_PWM_MID;
    }
    return LINE_BASE_PWM_SLOW;
}

static void apply_pwm(int16_t left, int16_t right, int16_t correction)
{
    left = clamp_i16(left, -LINE_PWM_LIMIT, LINE_PWM_LIMIT);
    right = clamp_i16(right, -LINE_PWM_LIMIT, LINE_PWM_LIMIT);

    g_debug.left_pwm = left;
    g_debug.right_pwm = right;
    g_debug.correction = correction;
    Chassis_SetPWM(left, right);
}

static void apply_line_pid(const tracker8_sample_t *sample, uint32_t dt_ms, int16_t base_pwm)
{
    float correction_f;
    int16_t correction;
    int16_t left;
    int16_t right;

    /* position_error: 左负右正。这里用 setpoint=position_error, measurement=0，
     * 让 correction 也保持左负右正：线在右侧时 correction>0，左轮更快、右轮更慢，车向右修正。
     */
    correction_f = PID_Update(&g_line_pid,
                              (float)sample->position_error,
                              0.0f,
                              (float)dt_ms / 1000.0f);
    correction = (int16_t)correction_f;

    left = (int16_t)(base_pwm + correction);
    right = (int16_t)(base_pwm - correction);
    apply_pwm(left, right, correction);
}

static void handle_start(const tracker8_sample_t *sample, uint32_t dt_ms)
{
    (void)sample;
    g_state_time_ms += dt_ms;
    apply_pwm(0, 0, 0);

    if (g_state_time_ms >= LINE_START_STABLE_MS)
    {
        enter_state(LINE_STATE_FOLLOW);
    }
}

static void handle_follow(const tracker8_sample_t *sample, uint32_t dt_ms)
{
    int8_t corner_dir;
    int16_t base_pwm;

    corner_dir = detect_corner_dir(sample);
    if (corner_dir != 0)
    {
        if (corner_dir_confirmed(corner_dir))
        {
            enter_corner(corner_dir);
            return;
        }
    }
    else
    {
        reset_corner_debounce();
    }

    if (!tracker_is_valid(sample))
    {
        g_lost_time_ms += dt_ms;
        if (g_lost_time_ms >= LINE_BLIND_ENTER_MS)
        {
            enter_state(LINE_STATE_BLIND);
            return;
        }

        apply_line_pid(sample, dt_ms, LINE_BASE_PWM_SLOW);
        return;
    }

    g_lost_time_ms = 0U;
    base_pwm = calc_dynamic_base_pwm(sample->position_error);
    apply_line_pid(sample, dt_ms, base_pwm);
}

static void handle_blind(const tracker8_sample_t *sample, uint32_t dt_ms)
{
    int16_t turn;
    int16_t left;
    int16_t right;

    g_state_time_ms += dt_ms;

    if (tracker_is_valid(sample))
    {
        g_reacquire_time_ms += dt_ms;
        if (g_reacquire_time_ms >= LINE_BLIND_REACQUIRE_MS)
        {
            enter_state(LINE_STATE_RECOVER);
            return;
        }
    }
    else
    {
        g_reacquire_time_ms = 0U;
    }

    if (g_state_time_ms >= LINE_BLIND_TIMEOUT_MS)
    {
        enter_state(LINE_STATE_LOST);
        return;
    }

    turn = (sample->last_valid_error >= 0) ? LINE_BLIND_TURN_PWM : (int16_t)(-LINE_BLIND_TURN_PWM);
    left = (int16_t)(LINE_BLIND_BASE_PWM + turn);
    right = (int16_t)(LINE_BLIND_BASE_PWM - turn);
    apply_pwm(left, right, turn);
}

static void handle_corner(const tracker8_sample_t *sample, uint32_t dt_ms)
{
    chassis_state_t ch;
    int16_t left;
    int16_t right;
    uint8_t reached_encoder;
    uint8_t found_center;

    g_state_time_ms += dt_ms;
    ch = Chassis_GetState();

    if (g_corner_dir < 0)
    {
        g_corner_encoder_sum += abs_i32((int32_t)ch.right_encoder_delta);
        left = LINE_CORNER_INNER_PWM;
        right = LINE_CORNER_OUTER_PWM;
    }
    else
    {
        g_corner_encoder_sum += abs_i32((int32_t)ch.left_encoder_delta);
        left = LINE_CORNER_OUTER_PWM;
        right = LINE_CORNER_INNER_PWM;
    }

    apply_pwm(left, right, 0);

    reached_encoder = (g_corner_encoder_sum >= LINE_CORNER_ENCODER_TARGET) ? 1U : 0U;
    found_center = tracker_center_found(sample);

    if (g_state_time_ms >= LINE_CORNER_MIN_MS && (reached_encoder || found_center))
    {
        if (g_corner_count < 65535U)
        {
            g_corner_count++;
        }
        enter_state(LINE_STATE_RECOVER);
        return;
    }

    if (g_state_time_ms >= LINE_CORNER_TIMEOUT_MS)
    {
        enter_state(LINE_STATE_LOST);
    }
}

static void handle_recover(const tracker8_sample_t *sample, uint32_t dt_ms)
{
    g_state_time_ms += dt_ms;

    if (!tracker_is_valid(sample))
    {
        g_lost_time_ms += dt_ms;
        if (g_lost_time_ms >= LINE_BLIND_ENTER_MS)
        {
            enter_state(LINE_STATE_BLIND);
            return;
        }
        apply_pwm(LINE_RECOVER_PWM, LINE_RECOVER_PWM, 0);
        return;
    }

    g_lost_time_ms = 0U;
    apply_line_pid(sample, dt_ms, LINE_RECOVER_PWM);

    if (g_state_time_ms >= LINE_RECOVER_MS)
    {
        enter_state(LINE_STATE_FOLLOW);
    }
}

static void handle_lost(const tracker8_sample_t *sample)
{
    Chassis_StopCoast();
    g_debug.left_pwm = 0;
    g_debug.right_pwm = 0;
    g_debug.correction = 0;

    if (tracker_is_valid(sample))
    {
        enter_state(LINE_STATE_RECOVER);
    }
}

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
    g_debug.state = LINE_STATE_START;
    g_debug.corner_dir = 0;
    g_debug.corner_count = 0U;
    g_debug.corner_encoder_sum = 0U;
    g_debug.state_time_ms = 0U;

    g_state = LINE_STATE_START;
    g_state_time_ms = 0U;
    g_lost_time_ms = 0U;
    g_reacquire_time_ms = 0U;
    g_corner_dir = 0;
    g_corner_encoder_sum = 0;
    g_corner_count = 0U;
    reset_corner_debounce();
}

void AppLineFollow_Update(uint32_t dt_ms)
{
    tracker8_sample_t sample;

    if (g_tracker == 0 || g_tracker->read == 0)
    {
        Chassis_StopCoast();
        return;
    }

    sample = g_tracker->read();
    g_debug.tracker = sample;

    switch (g_state)
    {
    case LINE_STATE_START:
        handle_start(&sample, dt_ms);
        break;

    case LINE_STATE_FOLLOW:
        handle_follow(&sample, dt_ms);
        break;

    case LINE_STATE_BLIND:
        handle_blind(&sample, dt_ms);
        break;

    case LINE_STATE_CORNER:
        handle_corner(&sample, dt_ms);
        break;

    case LINE_STATE_RECOVER:
        handle_recover(&sample, dt_ms);
        break;

    case LINE_STATE_LOST:
    default:
        handle_lost(&sample);
        break;
    }

    g_debug.state = g_state;
    g_debug.corner_dir = g_corner_dir;
    g_debug.corner_count = g_corner_count;
    g_debug.corner_encoder_sum = (uint16_t)clamp_i16(g_corner_encoder_sum, 0, 32767);
    g_debug.state_time_ms = g_state_time_ms;
}

line_follow_debug_t AppLineFollow_GetDebug(void)
{
    return g_debug;
}
