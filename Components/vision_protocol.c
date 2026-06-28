#include "vision_protocol.h"
#include "bsp_uart.h"
#include <string.h>
#include <stdlib.h>

#define VISION_LINE_BUF_SIZE 48U

static char g_line_buf[VISION_LINE_BUF_SIZE];
static uint8_t g_line_len = 0U;

void VisionProtocol_Init(void)
{
    g_line_len = 0U;
    memset(g_line_buf, 0, sizeof(g_line_buf));
}

static uint8_t parse_line(const char *line, vision_target_t *out_target, uint32_t now_ms)
{
    char *endptr;
    long x;
    long y;
    long valid;
    const char *p = line;

    if (line == 0 || out_target == 0) return 0U;
    if (p[0] != '$' || p[1] != 'T' || p[2] != ',') return 0U;

    p += 3;
    x = strtol(p, &endptr, 10);
    if (*endptr != ',') return 0U;
    p = endptr + 1;

    y = strtol(p, &endptr, 10);
    if (*endptr != ',') return 0U;
    p = endptr + 1;

    valid = strtol(p, &endptr, 10);

    out_target->x_error = (int16_t)x;
    out_target->y_error = (int16_t)y;
    out_target->valid = (valid != 0) ? 1U : 0U;
    out_target->timestamp_ms = now_ms;
    return 1U;
}

uint8_t VisionProtocol_Poll(vision_target_t *out_target, uint32_t now_ms)
{
    char ch;

    while (BSP_VisionUART_ReadCharNonBlocking(&ch))
    {
        if (ch == '\n' || ch == '\r')
        {
            if (g_line_len > 0U)
            {
                g_line_buf[g_line_len] = '\0';
                g_line_len = 0U;
                if (parse_line(g_line_buf, out_target, now_ms))
                {
                    return 1U;
                }
            }
        }
        else
        {
            if (g_line_len < (VISION_LINE_BUF_SIZE - 1U))
            {
                g_line_buf[g_line_len++] = ch;
            }
            else
            {
                g_line_len = 0U;
            }
        }
    }

    return 0U;
}
