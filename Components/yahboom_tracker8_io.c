/**
 * @file yahboom_tracker8_io.c
 * @brief Yahboom 8 路循迹模块 GPIO 采样实现。
 * @layer Components
 *
 * 本文件只负责从 GPIO 读 8 路传感器、按 TRACKER_BLACK_ACTIVE_LOW 转换有效电平、
 * 按权重计算位置误差。直角弯、丢线恢复和 PID 控制不在这里实现。
 */
#include "yahboom_tracker8_io.h"
#include "bsp_gpio.h"
#include "app_config.h"

static int16_t g_last_valid_error = 0;

/* X1~X8 从车头视角左到右排列，bit0 对应最左侧 X1。 */
static const gpio_pin_t g_tracker_pins[TRACKER_SENSOR_COUNT] =
{
    {GPIOB, GPIO_Pin_11},  /* X1：最左 */
    {GPIOB, GPIO_Pin_10},  /* X2 */
    {GPIOB, GPIO_Pin_1},   /* X3 */
    {GPIOB, GPIO_Pin_0},   /* X4 */
    {GPIOA, GPIO_Pin_7},   /* X5 */
    {GPIOA, GPIO_Pin_6},   /* X6 */
    {GPIOA, GPIO_Pin_5},   /* X7 */
    {GPIOA, GPIO_Pin_4}    /* X8：最右 */
};

/* 误差权重：左侧为负，右侧为正。权重大小决定纠偏响应强度。 */
static const int16_t g_tracker_weights[TRACKER_SENSOR_COUNT] =
{
    TRACKER_WEIGHT_0,
    TRACKER_WEIGHT_1,
    TRACKER_WEIGHT_2,
    TRACKER_WEIGHT_3,
    TRACKER_WEIGHT_4,
    TRACKER_WEIGHT_5,
    TRACKER_WEIGHT_6,
    TRACKER_WEIGHT_7
};

/**
 * @brief 初始化 8 路循迹输入 GPIO。
 *
 * 当前 Yahboom 8 路循迹输入为 X1 PB11、X2 PB10、X3 PB1、X4 PB0、
 * X5 PA7、X6 PA6、X7 PA5、X8 PA4，均按上拉输入配置。黑线是否为低电平有效
 * 不在 GPIO 初始化阶段决定，而由 YahboomTracker8IO_Read() 中的
 * TRACKER_BLACK_ACTIVE_LOW 宏统一转换。
 */
static void tracker_gpio_init(void)
{
    GPIO_InitTypeDef gpio;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    gpio.GPIO_Mode = GPIO_Mode_IPU;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;

    gpio.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &gpio);

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_Init(GPIOB, &gpio);
}

/**
 * @brief 初始化 8 路循迹输入 GPIO 和最近一次有效误差。
 *
 * 本函数现在负责初始化 8 路循迹输入 GPIO，使传感器驱动不再依赖
 * BSP_GPIO_InitAll() 的调用顺序。黑线有效电平由 TRACKER_BLACK_ACTIVE_LOW 决定：
 * 当前配置为 1，即 GPIO 读到低电平时认为检测到黑线。
 */
void YahboomTracker8IO_Init(void)
{
    tracker_gpio_init();
    g_last_valid_error = 0;
}

/**
 * @brief 读取 8 路 GPIO 并计算循迹误差。
 *
 * 当没有任何传感器检测到黑线时返回 TRACKER_STATUS_LOST，并保留 last_valid_error，
 * 供 BLIND 状态判断应该向哪一侧找线。
 */
tracker8_sample_t YahboomTracker8IO_Read(void)
{
    tracker8_sample_t sample;
    int32_t weighted_sum = 0;
    uint8_t i;

    sample.raw_bits = 0U;
    sample.active_count = 0U;
    sample.position_error = g_last_valid_error;
    sample.last_valid_error = g_last_valid_error;
    sample.status = TRACKER_STATUS_LOST;

    for (i = 0U; i < TRACKER_SENSOR_COUNT; ++i)
    {
        uint8_t level = BSP_GPIO_Read(g_tracker_pins[i]);
        uint8_t active;

#if TRACKER_BLACK_ACTIVE_LOW
        active = (level == 0U) ? 1U : 0U;
#else
        active = (level != 0U) ? 1U : 0U;
#endif
        if (active)
        {
            sample.raw_bits |= (uint8_t)(1U << i);
            sample.active_count++;
            weighted_sum += g_tracker_weights[i];
        }
    }

    if (sample.active_count == 0U)
    {
        sample.position_error = g_last_valid_error;
        sample.status = TRACKER_STATUS_LOST;
    }
    else
    {
        sample.position_error = (int16_t)(weighted_sum / sample.active_count);
        g_last_valid_error = sample.position_error;
        sample.last_valid_error = g_last_valid_error;

        if (sample.active_count >= 6U)
        {
            sample.status = TRACKER_STATUS_CROSS;
        }
        else
        {
            sample.status = TRACKER_STATUS_OK;
        }
    }

    return sample;
}

static const tracker8_driver_t g_tracker_driver =
{
    YahboomTracker8IO_Init,
    YahboomTracker8IO_Read
};

/**
 * @brief 返回 Yahboom 8 路传感器的 tracker8_driver_t 适配对象。
 */
const tracker8_driver_t *YahboomTracker8IO_GetDriver(void)
{
    return &g_tracker_driver;
}
