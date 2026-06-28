#include "yahboom_tracker8_io.h"
#include "bsp_gpio.h"
#include "app_config.h"

static int16_t g_last_valid_error = 0;

static const gpio_pin_t g_tracker_pins[TRACKER_SENSOR_COUNT] =
{
    {GPIOA, GPIO_Pin_2},   /* S1 */
    {GPIOA, GPIO_Pin_3},   /* S2 */
    {GPIOA, GPIO_Pin_4},   /* S3 */
    {GPIOA, GPIO_Pin_5},   /* S4 */
    {GPIOB, GPIO_Pin_8},   /* S5 */
    {GPIOB, GPIO_Pin_9},   /* S6 */
    {GPIOB, GPIO_Pin_10},  /* S7 */
    {GPIOB, GPIO_Pin_11}   /* S8 */
};

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

void YahboomTracker8IO_Init(void)
{
    g_last_valid_error = 0;
}

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

const tracker8_driver_t *YahboomTracker8IO_GetDriver(void)
{
    return &g_tracker_driver;
}
