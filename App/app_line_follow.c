/**
 * @file app_line_follow.c
 * @brief 8 路红外循迹状态机和 PID 差速控制。
 * @layer App
 *
 * 当前实现面向闭合矩形赛道：正常循迹、丢线搜索、直角弯、恢复和停车等待
 * 都在本文件内完成。后续若加入速度闭环或更多赛道类型，建议先保留接口，
 * 再逐步拆分传感器判定、状态机、控制器和调试输出。
 */
#include "app_line_follow.h"
#include "app_config.h"
#include "pid.h"
#include "chassis.h"
#include "common_types.h"
#include <stdio.h>

/* 传感器驱动由 AppRobot_Init 注入，便于后续替换不同循迹模块。 */
static const tracker8_driver_t *g_tracker = 0;
static pid_t g_line_pid;
static line_follow_debug_t g_debug;

/* 状态机私有状态，只在主循环控制周期内访问，不与中断共享。 */
static line_follow_state_t g_state = LINE_STATE_START;
static uint32_t g_state_time_ms = 0U;
static uint32_t g_lost_time_ms = 0U;
static uint32_t g_reacquire_time_ms = 0U;
static int8_t g_corner_dir = 0;
static int32_t g_corner_encoder_sum = 0;
static uint16_t g_corner_count = 0U;
static int8_t g_corner_candidate_dir = 0;
static uint8_t g_corner_candidate_count = 0U;
static uint32_t g_recover_lost_time_ms = 0U;
static uint32_t g_corner_rearm_ms = 0U;
static uint32_t g_corner_rearm_center_ms = 0U;
static uint32_t g_corner_center_search_ms = 0U;
static uint32_t g_corner_exit_confirm_ms = 0U;
static uint8_t g_corner_armed = 1U;
static int8_t g_blind_search_dir = 0;
static uint32_t g_sensor_all_inactive_ms = 0U;
static uint32_t g_sensor_all_active_ms = 0U;
static uint32_t g_sensor_healthy_ms = 0U;
static uint32_t g_recover_center_ms = 0U;
static line_sensor_fault_t g_sensor_fault = LINE_SENSOR_FAULT_NONE;

typedef enum
{
    LINE_TRANSITION_NONE = 0,
    LINE_TRANSITION_START_TO_FOLLOW = 1,
    LINE_TRANSITION_FOLLOW_TO_CORNER = 2,
    LINE_TRANSITION_FOLLOW_LOST_TO_BLIND = 3,
    LINE_TRANSITION_CORNER_ENCODER_EXIT = 4,
    LINE_TRANSITION_CORNER_CENTER_EXIT = 5,
    LINE_TRANSITION_CORNER_TIMEOUT_EXIT = 6,
    LINE_TRANSITION_CORNER_TO_BLIND = 7,
    LINE_TRANSITION_BLIND_REACQUIRE_TO_RECOVER = 8,
    LINE_TRANSITION_BLIND_TIMEOUT_TO_LOST = 9,
    LINE_TRANSITION_RECOVER_STABLE_TO_FOLLOW = 10,
    LINE_TRANSITION_RECOVER_LOST_TO_BLIND = 11,
    LINE_TRANSITION_LOST_REACQUIRE_TO_RECOVER = 12,
    LINE_TRANSITION_SENSOR_FAULT_TO_LOST = 13
} line_transition_reason_t;

static uint8_t g_transition_reason = (uint8_t)LINE_TRANSITION_NONE;
#if LINE_ENABLE_TRANSITION_TRACE
static tracker8_sample_t g_transition_sample;
#endif

static int32_t abs_i32(int32_t value)
{
    return (value < 0) ? -value : value;
}

/* 连续采样证据最多按一个名义控制周期累加，避免一次调度延迟等价于多帧确认。 */
static uint32_t add_sample_confirm_ms(uint32_t current, uint32_t dt_ms, uint32_t limit)
{
    uint32_t step = (dt_ms > APP_CONTROL_PERIOD_MS) ? APP_CONTROL_PERIOD_MS : dt_ms;

    if (current >= limit)
    {
        return limit;
    }
    if (step >= (limit - current))
    {
        return limit;
    }
    return current + step;
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

    center_bits = (uint8_t)(sample->raw_bits & 0x18U); /* X4/X5 */
    if (center_bits != 0U && abs_i32((int32_t)sample->position_error) <= LINE_RECOVER_ERROR_THRESHOLD)
    {
        return 1U;
    }

    return 0U;
}

/**
 * @brief 识别持续全高/全低和底层无效采样，并在恢复稳定后自动清除故障。
 */
static void update_sensor_health(tracker8_sample_t *sample, uint32_t dt_ms)
{
    if (sample->status == TRACKER_STATUS_INVALID)
    {
        g_sensor_fault = LINE_SENSOR_FAULT_DRIVER_INVALID;
        g_sensor_healthy_ms = 0U;
    }
    else if (sample->raw_bits == 0x00U)
    {
        g_sensor_all_inactive_ms = add_sample_confirm_ms(g_sensor_all_inactive_ms,
                                                         dt_ms,
                                                         TRACKER_ALL_INACTIVE_FAULT_MS);
        g_sensor_all_active_ms = 0U;
        g_sensor_healthy_ms = 0U;
        if (g_sensor_fault == LINE_SENSOR_FAULT_NONE &&
            g_sensor_all_inactive_ms >= TRACKER_ALL_INACTIVE_FAULT_MS)
        {
            g_sensor_fault = LINE_SENSOR_FAULT_ALL_INACTIVE;
        }
    }
    else if (sample->raw_bits == 0xFFU)
    {
        g_sensor_all_active_ms = add_sample_confirm_ms(g_sensor_all_active_ms,
                                                       dt_ms,
                                                       TRACKER_ALL_ACTIVE_FAULT_MS);
        g_sensor_all_inactive_ms = 0U;
        g_sensor_healthy_ms = 0U;
        if (g_sensor_fault == LINE_SENSOR_FAULT_NONE &&
            g_sensor_all_active_ms >= TRACKER_ALL_ACTIVE_FAULT_MS)
        {
            g_sensor_fault = LINE_SENSOR_FAULT_ALL_ACTIVE;
        }
    }
    else
    {
        g_sensor_all_inactive_ms = 0U;
        g_sensor_all_active_ms = 0U;
        if (g_sensor_fault != LINE_SENSOR_FAULT_NONE)
        {
            g_sensor_healthy_ms = add_sample_confirm_ms(g_sensor_healthy_ms,
                                                        dt_ms,
                                                        TRACKER_FAULT_CLEAR_MS);
            if (g_sensor_healthy_ms >= TRACKER_FAULT_CLEAR_MS)
            {
                g_sensor_fault = LINE_SENSOR_FAULT_NONE;
                g_sensor_healthy_ms = 0U;
            }
        }
    }

    if (g_sensor_fault != LINE_SENSOR_FAULT_NONE)
    {
        sample->status = TRACKER_STATUS_INVALID;
    }
}

/* 直角弯检测去抖：要求连续多帧同方向特征，降低反光/杂线误触发概率。 */
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
        /* 丢线本身不能直接判定为直角弯。否则普通偏出赛道/传感器瞬断也会进入 CORNER，
         * 你的串口里 S=3 且 RAW=0x00 就是这种风险。矩形直角只在有效压线形态下识别。
         */
        return 0;
    }

    left_count = count_bits4((uint8_t)(sample->raw_bits & 0x0FU));
    right_count = count_bits4((uint8_t)((sample->raw_bits >> 4) & 0x0FU));

    /* 矩形赛道直角弯：一侧 3~4 路连续压线，另一侧很少压线。 */
    if (RECT_DEFAULT_CORNER_DIR < 0)
    {
        if (left_count >= 3U && right_count <= 1U)
        {
            return -1;
        }
    }
    else
    {
        if (right_count >= 3U && left_count <= 1U)
        {
            return 1;
        }
    }

#if RECT_ENABLE_CROSS_CORNER
    /* 宽横线/全黑线：只有确认赛道直角处会稳定出现“多路同时触发”时才打开。
     * 默认关闭，避免传感器过低、黑线过宽或反光导致直线误进 CORNER。
     */
    if (sample->status == TRACKER_STATUS_CROSS)
    {
        return (RECT_DEFAULT_CORNER_DIR >= 0) ? 1 : -1;
    }
#endif

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

/**
 * @brief 切换循迹状态并清理状态内计时器。
 *
 * FOLLOW/RECOVER 重新启用 PID 时复位积分和 D 项历史，避免上个状态的误差
 * 直接带入新的差速输出。
 */
static void enter_state(line_follow_state_t next_state)
{
#if LINE_ENABLE_TRANSITION_TRACE
    line_follow_state_t old_state = g_state;
    uint32_t old_state_time_ms = g_state_time_ms;
    uint8_t old_raw_bits = g_transition_sample.raw_bits;
    int16_t old_position_error = g_transition_sample.position_error;
    uint16_t old_corner_encoder_sum = (uint16_t)g_corner_encoder_sum;
    uint32_t old_recover_center_ms = g_recover_center_ms;
    uint32_t old_recover_lost_time_ms = g_recover_lost_time_ms;
    uint8_t old_corner_armed = g_corner_armed;
    uint32_t old_corner_rearm_ms = g_corner_rearm_ms;
#endif

    g_state = next_state;
    g_state_time_ms = 0U;
    g_lost_time_ms = 0U;
    g_reacquire_time_ms = 0U;
    reset_corner_debounce();

    if (next_state != LINE_STATE_BLIND)
    {
        g_blind_search_dir = 0;
    }

    if (next_state == LINE_STATE_FOLLOW || next_state == LINE_STATE_RECOVER)
    {
        PID_Reset(&g_line_pid);
    }

#if LINE_ENABLE_TRANSITION_TRACE
    if (old_state != next_state)
    {
        printf("EV FROM=%u TO=%u TR=%u OST=%lu RAW=0x%02X ERR=%d SUM=%u RCM=%lu RLM=%lu ARM=%u RM=%lu\r\n",
               (unsigned int)old_state,
               (unsigned int)next_state,
               (unsigned int)g_transition_reason,
               (unsigned long)old_state_time_ms,
               (unsigned int)old_raw_bits,
               (int)old_position_error,
               (unsigned int)old_corner_encoder_sum,
               (unsigned long)old_recover_center_ms,
               (unsigned long)old_recover_lost_time_ms,
               (unsigned int)old_corner_armed,
               (unsigned long)old_corner_rearm_ms);
    }
#endif
}

/**
 * @brief 进入直角弯状态。
 * @param dir -1 左转，+1 右转，0 时使用 RECT_DEFAULT_CORNER_DIR。
 */
static void enter_corner(int8_t dir)
{
    if (dir == 0)
    {
        dir = (RECT_DEFAULT_CORNER_DIR >= 0) ? 1 : -1;
    }

    g_corner_dir = dir;
    g_corner_encoder_sum = 0;
    g_corner_center_search_ms = 0U;
    g_corner_exit_confirm_ms = 0U;
    PID_Reset(&g_line_pid);
    g_transition_reason = (uint8_t)LINE_TRANSITION_FOLLOW_TO_CORNER;
    enter_state(LINE_STATE_CORNER);
}

/**
 * @brief 根据循迹误差动态选择基础 PWM。
 *
 * 误差小使用较高速度，误差大自动降速，避免直线和弯道使用同一速度导致过弯冲出。
 */
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

/**
 * @brief 下发左右轮 PWM 并更新调试字段。
 *
 * LINE_PWM_LIMIT 是循迹应用层限幅；Chassis_SetPWM 内的 CHASSIS_PWM_LIMIT
 * 是更宽的底盘安全限幅，保留双层限制以隔离应用调参与底盘保护。
 */
static void apply_pwm(int16_t left, int16_t right, int16_t correction)
{
    left = clamp_i16(left, -LINE_PWM_LIMIT, LINE_PWM_LIMIT);
    right = clamp_i16(right, -LINE_PWM_LIMIT, LINE_PWM_LIMIT);

    g_debug.left_pwm = left;
    g_debug.right_pwm = right;
    g_debug.correction = correction;
    Chassis_SetPWM(left, right);
}

/**
 * @brief 对有效循迹误差执行 PID 差速控制。
 *
 * PID 输出 correction 后，左轮 = base + correction，右轮 = base - correction。
 * 当前 correction 正负约定依赖 tracker8_if 中“左负右正”的误差定义。
 */
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

/**
 * @brief RECOVER 专用 PID 输出，仅限制修正量，不改变 PID 计算和转向极性。
 */
static void apply_recover_pid(const tracker8_sample_t *sample, uint32_t dt_ms)
{
    float correction_f;
    int16_t correction;
    int16_t left;
    int16_t right;

    correction_f = PID_Update(&g_line_pid,
                              (float)sample->position_error,
                              0.0f,
                              (float)dt_ms / 1000.0f);
    correction = clamp_i16((int16_t)correction_f,
                           -LINE_RECOVER_CORRECTION_LIMIT,
                           LINE_RECOVER_CORRECTION_LIMIT);

    left = (int16_t)(LINE_RECOVER_PWM + correction);
    right = (int16_t)(LINE_RECOVER_PWM - correction);
    apply_pwm(left, right, correction);
}

/**
 * @brief START 状态：上电后保持电机停止，等待传感器和电源稳定。
 */
static void handle_start(const tracker8_sample_t *sample, uint32_t dt_ms)
{
    (void)sample;
    g_state_time_ms += dt_ms;
    apply_pwm(0, 0, 0);

    if (g_state_time_ms >= LINE_START_STABLE_MS)
    {
        g_transition_reason = (uint8_t)LINE_TRANSITION_START_TO_FOLLOW;
        enter_state(LINE_STATE_FOLLOW);
    }
}

/**
 * @brief FOLLOW 状态：正常循迹、直角候选识别和丢线入口。
 */
static void handle_follow(const tracker8_sample_t *sample, uint32_t dt_ms)
{
    int8_t corner_dir;
    int16_t base_pwm;

    corner_dir = (g_corner_armed != 0U) ? detect_corner_dir(sample) : 0;
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
            g_transition_reason = (uint8_t)LINE_TRANSITION_FOLLOW_LOST_TO_BLIND;
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

/**
 * @brief BLIND 状态：按最后一次有效误差低速偏转找线。
 *
 * 电机动作不使用反转轮，只用左右轮速度差寻找黑线，避免丢线时大幅甩头。
 */
static void handle_blind(const tracker8_sample_t *sample, uint32_t dt_ms)
{
    int16_t left;
    int16_t right;

    g_state_time_ms += dt_ms;

    if (g_blind_search_dir == 0)
    {
        if (sample->last_valid_error > 0)
        {
            g_blind_search_dir = 1;
        }
        else if (sample->last_valid_error < 0)
        {
            g_blind_search_dir = -1;
        }
        else
        {
            g_blind_search_dir = (RECT_DEFAULT_CORNER_DIR >= 0) ? 1 : -1;
        }
    }

    if (tracker_is_valid(sample))
    {
        g_reacquire_time_ms = add_sample_confirm_ms(g_reacquire_time_ms,
                                                    dt_ms,
                                                    LINE_BLIND_REACQUIRE_MS);
        if (g_reacquire_time_ms >= LINE_BLIND_REACQUIRE_MS)
        {
            g_transition_reason = (uint8_t)LINE_TRANSITION_BLIND_REACQUIRE_TO_RECOVER;
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
        g_transition_reason = (uint8_t)LINE_TRANSITION_BLIND_TIMEOUT_TO_LOST;
        enter_state(LINE_STATE_LOST);
        return;
    }

    /* 丢线搜索先不用反转轮，避免 LPWM=-140/RPWM=500 这种猛甩头。
     * 搜索方向在进入 BLIND 后锁定，避免边缘传感器抖动造成反复换向。
     */
    if (g_blind_search_dir >= 0)
    {
        left = LINE_BLIND_TURN_PWM;
        right = LINE_BLIND_BASE_PWM;
    }
    else
    {
        left = LINE_BLIND_BASE_PWM;
        right = LINE_BLIND_TURN_PWM;
    }
    apply_pwm(left, right, 0);
}

/**
 * @brief CORNER 状态：矩形赛道直角弯处理。
 *
 * 进入条件来自 FOLLOW 中的直角特征去抖。退出条件由
 * LINE_CORNER_USE_ENCODER 决定：当前固定左转使用右编码器累计值退出；
 * 传感器出口需要连续 LINE_CORNER_EXIT_CONFIRM_MS 确认。
 */
static void handle_corner(const tracker8_sample_t *sample, uint32_t dt_ms)
{
    chassis_state_t ch;
    int16_t left;
    int16_t right;
    uint8_t reached_encoder = 0U;
    uint8_t reached_time = 0U;
    uint8_t found_center = 0U;
    uint8_t found_line = 0U;
    uint8_t sensor_exit_candidate = 0U;
    uint8_t sensor_exit_confirmed = 0U;
    uint8_t align_phase = 0U;
    uint8_t center_search_timeout = 0U;

    g_state_time_ms += dt_ms;
    ch = Chassis_GetState();

    if (g_corner_dir < 0)
    {
        g_corner_encoder_sum += abs_i32((int32_t)ch.right_encoder_delta);
    }
    else
    {
        g_corner_encoder_sum += abs_i32((int32_t)ch.left_encoder_delta);
    }

#if LINE_CORNER_USE_ENCODER
    reached_encoder = (g_corner_encoder_sum >= LINE_CORNER_ENCODER_TARGET) ? 1U : 0U;
    if (g_corner_encoder_sum >= LINE_CORNER_CENTER_ENABLE_ENCODER)
    {
        found_center = tracker_center_found(sample);
    }
    if (reached_encoder)
    {
        found_line = tracker_is_valid(sample);
    }
    if (reached_encoder && !found_line)
    {
        g_corner_center_search_ms += dt_ms;
        center_search_timeout = (g_corner_center_search_ms >= LINE_CORNER_CENTER_SEARCH_MS) ? 1U : 0U;
    }
#else
    /* 编码器还没调通时，先用固定时间退出直角弯，避免 SUM=0 时卡死或直接 LOST。 */
    reached_time = (g_state_time_ms >= LINE_CORNER_TIME_MS) ? 1U : 0U;
    if (g_state_time_ms >= LINE_CORNER_MIN_MS)
    {
        found_center = tracker_center_found(sample);
    }
#endif

    sensor_exit_candidate = (found_center || (reached_encoder && found_line)) ? 1U : 0U;
    if (sensor_exit_candidate)
    {
        g_corner_exit_confirm_ms = add_sample_confirm_ms(g_corner_exit_confirm_ms,
                                                         dt_ms,
                                                         LINE_CORNER_EXIT_CONFIRM_MS);
    }
    else
    {
        g_corner_exit_confirm_ms = 0U;
    }
    sensor_exit_confirmed = (g_corner_exit_confirm_ms >= LINE_CORNER_EXIT_CONFIRM_MS) ? 1U : 0U;
    align_phase = (reached_encoder || found_center) ? 1U : 0U;

    if (g_corner_dir < 0)
    {
        left = align_phase ? LINE_CORNER_ALIGN_INNER_PWM : LINE_CORNER_INNER_PWM;
        right = align_phase ? LINE_CORNER_ALIGN_OUTER_PWM : LINE_CORNER_OUTER_PWM;
    }
    else
    {
        left = align_phase ? LINE_CORNER_ALIGN_OUTER_PWM : LINE_CORNER_OUTER_PWM;
        right = align_phase ? LINE_CORNER_ALIGN_INNER_PWM : LINE_CORNER_INNER_PWM;
    }

    apply_pwm(left, right, 0);

    if (g_state_time_ms >= LINE_CORNER_MIN_MS && (reached_time || sensor_exit_confirmed))
    {
        if (g_corner_count < 65535U)
        {
            g_corner_count++;
        }
        g_corner_rearm_ms = LINE_CORNER_REARM_MS;
        g_corner_rearm_center_ms = 0U;
        g_corner_armed = 0U;
        if (found_center)
        {
            g_transition_reason = (uint8_t)LINE_TRANSITION_CORNER_CENTER_EXIT;
        }
        else if (reached_encoder)
        {
            g_transition_reason = (uint8_t)LINE_TRANSITION_CORNER_ENCODER_EXIT;
        }
        else
        {
            g_transition_reason = (uint8_t)LINE_TRANSITION_CORNER_TIMEOUT_EXIT;
        }
        enter_state(LINE_STATE_RECOVER);
        return;
    }

    if (g_state_time_ms >= LINE_CORNER_MIN_MS && center_search_timeout)
    {
        if (g_corner_count < 65535U)
        {
            g_corner_count++;
        }
        g_corner_rearm_ms = LINE_CORNER_REARM_MS;
        g_corner_rearm_center_ms = 0U;
        g_corner_armed = 0U;
        g_transition_reason = (uint8_t)LINE_TRANSITION_CORNER_TO_BLIND;
        enter_state(LINE_STATE_BLIND);
        return;
    }

    if (g_state_time_ms >= LINE_CORNER_TIMEOUT_MS)
    {
        g_transition_reason = (uint8_t)LINE_TRANSITION_CORNER_TIMEOUT_EXIT;
        enter_state(LINE_STATE_BLIND);
    }
}

/**
 * @brief RECOVER 状态：重新看到线后的低速平滑恢复。
 */
static void handle_recover(const tracker8_sample_t *sample, uint32_t dt_ms)
{
    g_state_time_ms += dt_ms;

    if (!tracker_is_valid(sample))
    {
        g_recover_lost_time_ms =
            add_sample_confirm_ms(g_recover_lost_time_ms,
                                  dt_ms,
                                  LINE_RECOVER_LOST_CONFIRM_MS);

        if (g_recover_lost_time_ms >= LINE_RECOVER_LOST_CONFIRM_MS)
        {
            g_transition_reason = (uint8_t)LINE_TRANSITION_RECOVER_LOST_TO_BLIND;
            enter_state(LINE_STATE_BLIND);
            return;
        }

        /*
         * 允许 1～2 个控制周期的瞬时丢线。
         * 此时不要继续使用上一次误差猛打方向。
         */
        PID_Reset(&g_line_pid);

        apply_pwm(LINE_RECOVER_LOST_PWM,
                  LINE_RECOVER_LOST_PWM,
                  0);
        return;
    }

    g_recover_lost_time_ms = 0U;
    g_lost_time_ms = 0U;

    apply_recover_pid(sample, dt_ms);

/*
 * 必须在 RECOVER 中运行至少 LINE_RECOVER_MS，
 * 并且黑线连续稳定在中央一段时间，才能回到 FOLLOW。
 */
    if (sample->status == TRACKER_STATUS_OK &&
        (sample->raw_bits & 0x18U) != 0U &&
        abs_i32((int32_t)sample->position_error) <= LINE_RECOVER_CENTER_ERROR_MAX)
    {
        g_recover_center_ms =
            add_sample_confirm_ms(g_recover_center_ms,
                                dt_ms,
                                LINE_RECOVER_CENTER_CONFIRM_MS);
    }
    else
    {
        g_recover_center_ms = 0U;
    }

    if (g_state_time_ms >= LINE_RECOVER_MS &&
        g_recover_center_ms >= LINE_RECOVER_CENTER_CONFIRM_MS)
    {
        g_transition_reason = (uint8_t)LINE_TRANSITION_RECOVER_STABLE_TO_FOLLOW;
        enter_state(LINE_STATE_FOLLOW);
    }
}

/**
 * @brief LOST 状态：长时间找不到线后停车，等待稳定重新识别到黑线。
 */
static void handle_lost(const tracker8_sample_t *sample, uint32_t dt_ms)
{
    Chassis_StopCoast();
    g_debug.left_pwm = 0;
    g_debug.right_pwm = 0;
    g_debug.correction = 0;

    /* LOST 后不能一看到单帧有效就立刻 RECOVER，否则传感器抖一下会反复启动/停车。
     * 要求连续稳定看到线 LINE_BLIND_REACQUIRE_MS 后，再进入恢复状态。
     */
    if (tracker_is_valid(sample))
    {
        g_reacquire_time_ms = add_sample_confirm_ms(g_reacquire_time_ms,
                                                    dt_ms,
                                                    LINE_BLIND_REACQUIRE_MS);
        if (g_reacquire_time_ms >= LINE_BLIND_REACQUIRE_MS)
        {
            g_transition_reason = (uint8_t)LINE_TRANSITION_LOST_REACQUIRE_TO_RECOVER;
            enter_state(LINE_STATE_RECOVER);
        }
    }
    else
    {
        g_reacquire_time_ms = 0U;
    }
}

/**
 * @brief 停车并把循迹运行状态复位到 START。
 */
void AppLineFollow_Reset(void)
{
    Chassis_StopCoast();
    PID_Reset(&g_line_pid);

    g_debug.tracker.raw_bits = 0U;
    g_debug.tracker.active_count = 0U;
    g_debug.tracker.position_error = 0;
    g_debug.tracker.last_valid_error = 0;
    g_debug.tracker.status = TRACKER_STATUS_INVALID;
    g_debug.left_pwm = 0;
    g_debug.right_pwm = 0;
    g_debug.correction = 0;
    g_debug.state = LINE_STATE_START;
    g_debug.corner_dir = 0;
    g_debug.corner_count = 0U;
    g_debug.corner_encoder_sum = 0U;
    g_debug.sensor_fault = LINE_SENSOR_FAULT_NONE;
    g_debug.state_time_ms = 0U;

    g_state = LINE_STATE_START;
    g_state_time_ms = 0U;
    g_lost_time_ms = 0U;
    g_reacquire_time_ms = 0U;
    g_corner_dir = 0;
    g_corner_encoder_sum = 0;
    g_corner_count = 0U;
    g_corner_rearm_ms = 0U;
    g_corner_rearm_center_ms = 0U;
    g_corner_center_search_ms = 0U;
    g_corner_exit_confirm_ms = 0U;
    g_corner_armed = 1U;
    g_sensor_all_inactive_ms = 0U;
    g_sensor_all_active_ms = 0U;
    g_sensor_healthy_ms = 0U;
    g_sensor_fault = LINE_SENSOR_FAULT_NONE;
    g_transition_reason = (uint8_t)LINE_TRANSITION_NONE;
    reset_corner_debounce();
}

/**
 * @brief 初始化循迹状态机、传感器驱动和 PID。
 */
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
    AppLineFollow_Reset();
}

/**
 * @brief 执行一次循迹控制周期。
 *
 * 本函数由 AppRobot_Task 按 APP_CONTROL_PERIOD_MS 调用。函数内读取一次传感器，
 * 再根据当前状态机分派到对应处理函数。
 */
void AppLineFollow_Update(uint32_t dt_ms)
{
    tracker8_sample_t sample;

    if (g_tracker == 0 || g_tracker->read == 0)
    {
        Chassis_StopCoast();
        return;
    }

    sample = g_tracker->read();
    update_sensor_health(&sample, dt_ms);
#if LINE_ENABLE_TRANSITION_TRACE
    g_transition_sample = sample;
#endif
    g_debug.tracker = sample;

    if (g_sensor_fault != LINE_SENSOR_FAULT_NONE)
    {
        if (g_state != LINE_STATE_LOST)
        {
            g_transition_reason = (uint8_t)LINE_TRANSITION_SENSOR_FAULT_TO_LOST;
            enter_state(LINE_STATE_LOST);
        }
        Chassis_StopCoast();
        g_debug.left_pwm = 0;
        g_debug.right_pwm = 0;
        g_debug.correction = 0;
        g_debug.state = g_state;
        g_debug.sensor_fault = g_sensor_fault;
        g_debug.state_time_ms = g_state_time_ms;
        g_debug.corner_armed = g_corner_armed;
        g_debug.corner_rearm_ms = g_corner_rearm_ms;
        g_debug.corner_rearm_center_ms = g_corner_rearm_center_ms;
        g_debug.recover_center_ms = g_recover_center_ms;
        g_debug.recover_lost_time_ms = g_recover_lost_time_ms;
        g_debug.transition_reason = g_transition_reason;
        return;
    }

    /*
 * 直角完成后，必须连续处于正常 FOLLOW 状态一段时间，
 * 才允许识别下一个直角。
 *
 * 在 CORNER、RECOVER、BLIND、LOST 中不计算重新使能时间；
 * 一旦离开 FOLLOW，重新开始计时。
 */
    if (g_corner_armed == 0U)
    {
        if (g_state != LINE_STATE_FOLLOW)
        {
            /*
            * 尚未稳定进入下一段直线，保持直角检测关闭。
            */
            g_corner_rearm_ms = LINE_CORNER_REARM_MS;
            g_corner_rearm_center_ms = 0U;
        }
        else
        {
            /*
            * 只有连续处于 FOLLOW 时，才递减重新使能延时。
            */
            if (g_corner_rearm_ms > 0U)
            {
                if (dt_ms >= g_corner_rearm_ms)
                {
                    g_corner_rearm_ms = 0U;
                }
                else
                {
                    g_corner_rearm_ms -= dt_ms;
                }

                g_corner_rearm_center_ms = 0U;
            }
            else if (tracker_center_found(&sample))
            {
                /*
                * FOLLOW 延时结束后，还需中心线连续稳定。
                */
                g_corner_rearm_center_ms += dt_ms;

                if (g_corner_rearm_center_ms >=
                    LINE_CORNER_REARM_CENTER_MS)
                {
                    g_corner_armed = 1U;
                    g_corner_rearm_center_ms = 0U;
                }
            }
            else
            {
                g_corner_rearm_center_ms = 0U;
            }
        }
    }

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
        handle_lost(&sample, dt_ms);
        break;
    }

    g_debug.state = g_state;
    g_debug.corner_dir = g_corner_dir;
    g_debug.corner_count = g_corner_count;
    g_debug.corner_encoder_sum = (uint16_t)clamp_i16(g_corner_encoder_sum, 0, 32767);
    g_debug.sensor_fault = g_sensor_fault;
    g_debug.state_time_ms = g_state_time_ms;
    g_debug.corner_armed = g_corner_armed;
    g_debug.corner_rearm_ms = g_corner_rearm_ms;
    g_debug.corner_rearm_center_ms = g_corner_rearm_center_ms;
    g_debug.recover_center_ms = g_recover_center_ms;
    g_debug.recover_lost_time_ms = g_recover_lost_time_ms;
    g_debug.transition_reason = g_transition_reason;
}

/**
 * @brief 返回当前循迹调试快照。
 */
line_follow_debug_t AppLineFollow_GetDebug(void)
{
    return g_debug;
}
