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

    center_bits = (uint8_t)(sample->raw_bits & 0x18U); /* X4/X5 */
    if (center_bits != 0U && abs_i32((int32_t)sample->position_error) <= LINE_RECOVER_ERROR_THRESHOLD)
    {
        return 1U;
    }

    return 0U;
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
    if ((left_count >= 3U && right_count <= 1U) ||
        (right_count >= 3U && left_count <= 1U))
    {
        return (RECT_DEFAULT_CORNER_DIR >= 0) ? 1 : -1;
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
    PID_Reset(&g_line_pid);
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
 * 是底盘安全限幅。当前两者相同，本轮保持双层限幅以避免改变既有输出路径。
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
 * @brief START 状态：上电后保持电机停止，等待传感器和电源稳定。
 */
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

/**
 * @brief FOLLOW 状态：正常循迹、直角候选识别和丢线入口。
 */
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

    /* 丢线搜索先不用反转轮，避免 LPWM=-140/RPWM=500 这种猛甩头。
     * last_valid_error 左负右正：线最后在右侧就向右找，最后在左侧就向左找。
     */
    if (sample->last_valid_error >= 0)
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
 * LINE_CORNER_USE_ENCODER 决定：当前配置为时间退出，编码器恢复后可改为计数退出。
 */
static void handle_corner(const tracker8_sample_t *sample, uint32_t dt_ms)
{
    chassis_state_t ch;
    int16_t left;
    int16_t right;
    uint8_t reached_encoder = 0U;
    uint8_t reached_time = 0U;
    uint8_t found_center = 0U;

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

#if LINE_CORNER_USE_ENCODER
    reached_encoder = (g_corner_encoder_sum >= LINE_CORNER_ENCODER_TARGET) ? 1U : 0U;
    if (g_corner_encoder_sum >= LINE_CORNER_CENTER_ENABLE_ENCODER)
    {
        found_center = tracker_center_found(sample);
    }
#else
    /* 编码器还没调通时，先用固定时间退出直角弯，避免 SUM=0 时卡死或直接 LOST。 */
    reached_time = (g_state_time_ms >= LINE_CORNER_TIME_MS) ? 1U : 0U;
    if (g_state_time_ms >= LINE_CORNER_MIN_MS)
    {
        found_center = tracker_center_found(sample);
    }
#endif

    if (g_state_time_ms >= LINE_CORNER_MIN_MS && (reached_encoder || reached_time || found_center))
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
        /* RECOVER 的含义是“已经重新看见线后的平滑恢复”。
         * RAW=0 时继续直行会把车带离赛道，因此立即退回 BLIND 找线。
         */
        enter_state(LINE_STATE_BLIND);
        return;
    }

    g_lost_time_ms = 0U;
    apply_line_pid(sample, dt_ms, LINE_RECOVER_PWM);

    if (g_state_time_ms >= LINE_RECOVER_MS)
    {
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
        g_reacquire_time_ms += dt_ms;
        if (g_reacquire_time_ms >= LINE_BLIND_REACQUIRE_MS)
        {
            enter_state(LINE_STATE_RECOVER);
        }
    }
    else
    {
        g_reacquire_time_ms = 0U;
    }
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
        handle_lost(&sample, dt_ms);
        break;
    }

    g_debug.state = g_state;
    g_debug.corner_dir = g_corner_dir;
    g_debug.corner_count = g_corner_count;
    g_debug.corner_encoder_sum = (uint16_t)clamp_i16(g_corner_encoder_sum, 0, 32767);
    g_debug.state_time_ms = g_state_time_ms;
}

/**
 * @brief 返回当前循迹调试快照。
 */
line_follow_debug_t AppLineFollow_GetDebug(void)
{
    return g_debug;
}
