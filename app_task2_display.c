/*
 * app_task2_display.c
 *
 * Display behavior:
 *   WAIT_KEY  : READY / PRESS K1 / 00.00
 *   LEAVING_A : STARTING / running time
 *   RUNNING   : RUNNING / running time
 *   FINISHED  : FINISHED or TIMEOUT / frozen final time
 *
 * Only framebuffer pages 3 and 4 are refreshed for the running digits.
 * A full refresh is used only when the state changes.
 */

#include "app_task2_display.h"
#include "app_oled.h"
#include "app_task2.h"
#include "app_irtracking.h"
#include "app_timebase.h"

#include <stdint.h>
#include <stdio.h>

#define TASK2_DISPLAY_REFRESH_MS        100U
#define TASK2_TIME_FIRST_PAGE             3U
#define TASK2_TIME_PAGE_COUNT              2U
#define TASK2_BIG_TIME_X                  34U
#define TASK2_BIG_TIME_Y                  24U
#define TASK2_BIG_SCALE                    2U
#define TASK2_BIG_ADVANCE                 12U

static bool s_display_online = false;
static Task2_State_t s_last_state = (Task2_State_t)0xFFU;
static uint32_t s_last_refresh_ms = 0U;
static uint32_t s_last_centiseconds = 0xFFFFFFFFU;

static const uint8_t s_big_digits[10][5] = {
    {0x3EU, 0x51U, 0x49U, 0x45U, 0x3EU},
    {0x00U, 0x42U, 0x7FU, 0x40U, 0x00U},
    {0x42U, 0x61U, 0x51U, 0x49U, 0x46U},
    {0x21U, 0x41U, 0x45U, 0x4BU, 0x31U},
    {0x18U, 0x14U, 0x12U, 0x7FU, 0x10U},
    {0x27U, 0x45U, 0x45U, 0x45U, 0x39U},
    {0x3CU, 0x4AU, 0x49U, 0x49U, 0x30U},
    {0x01U, 0x71U, 0x09U, 0x05U, 0x03U},
    {0x36U, 0x49U, 0x49U, 0x49U, 0x36U},
    {0x06U, 0x49U, 0x49U, 0x29U, 0x1EU}
};

static const uint8_t s_big_period[5] = {
    0x00U, 0x60U, 0x60U, 0x00U, 0x00U
};

static uint32_t Task2_DisplayTimeMs(void)
{
    if (g_task2_state == TASK2_STATE_FINISHED)
    {
        return g_task2_finish_ms;
    }

    if ((g_task2_state == TASK2_STATE_LEAVING_A) ||
        (g_task2_state == TASK2_STATE_RUNNING))
    {
        return g_task2_elapsed_ms;
    }

    return 0U;
}

static void Task2_ClearBigTimeArea(void)
{
    uint8_t x;
    uint8_t y;

    for (y = TASK2_BIG_TIME_Y;
         y < (TASK2_BIG_TIME_Y + 14U);
         y++)
    {
        for (x = TASK2_BIG_TIME_X;
             x < (TASK2_BIG_TIME_X + 60U);
             x++)
        {
            OLED_DrawPixel(x, y, false);
        }
    }
}

static void Task2_DrawBigGlyph(uint8_t x,
                               uint8_t y,
                               const uint8_t glyph[5])
{
    uint8_t column;
    uint8_t row;
    uint8_t dx;
    uint8_t dy;

    for (column = 0U; column < 5U; column++)
    {
        for (row = 0U; row < 7U; row++)
        {
            bool on = ((glyph[column] >> row) & 0x01U) != 0U;

            for (dx = 0U; dx < TASK2_BIG_SCALE; dx++)
            {
                for (dy = 0U; dy < TASK2_BIG_SCALE; dy++)
                {
                    OLED_DrawPixel(
                        (uint8_t)(x + column * TASK2_BIG_SCALE + dx),
                        (uint8_t)(y + row * TASK2_BIG_SCALE + dy),
                        on);
                }
            }
        }
    }
}

static void Task2_DrawBigTime(uint32_t elapsed_ms)
{
    uint32_t centiseconds = elapsed_ms / 10U;
    uint32_t seconds;
    uint32_t hundredths;
    uint8_t x = TASK2_BIG_TIME_X;
    uint8_t index;
    uint8_t characters[5];

    /* Task 2 has a 23 s safety timeout; 99.99 is only a display guard. */
    if (centiseconds > 9999U)
    {
        centiseconds = 9999U;
    }

    seconds = centiseconds / 100U;
    hundredths = centiseconds % 100U;

    characters[0] = (uint8_t)(seconds / 10U);
    characters[1] = (uint8_t)(seconds % 10U);
    characters[2] = 0xFFU; /* decimal point */
    characters[3] = (uint8_t)(hundredths / 10U);
    characters[4] = (uint8_t)(hundredths % 10U);

    Task2_ClearBigTimeArea();

    for (index = 0U; index < 5U; index++)
    {
        if (characters[index] == 0xFFU)
        {
            Task2_DrawBigGlyph(x, TASK2_BIG_TIME_Y, s_big_period);
        }
        else
        {
            Task2_DrawBigGlyph(x,
                               TASK2_BIG_TIME_Y,
                               s_big_digits[characters[index]]);
        }

        x = (uint8_t)(x + TASK2_BIG_ADVANCE);
    }
}

static void Task2_DrawStatePage(void)
{
    uint32_t display_ms = Task2_DisplayTimeMs();
    char ir_buf[12];

    OLED_Clear();
    OLED_DrawRectangle(0U, 0U, OLED_WIDTH, OLED_HEIGHT, true);
    OLED_ShowString(28U, 0U, "TASK 2 TIMER");

    switch (g_task2_state)
    {
        case TASK2_STATE_WAIT_KEY:
            OLED_ShowString(46U, 1U, "READY");
            OLED_ShowString(37U, 7U, "PRESS K1");
            break;

        case TASK2_STATE_LEAVING_A:
            OLED_ShowString(40U, 1U, "STARTING");
            OLED_ShowString(40U, 7U, "TIME  SEC");
            break;

        case TASK2_STATE_RUNNING:
            OLED_ShowString(43U, 1U, "RUNNING");
            OLED_ShowString(40U, 7U, "TIME  SEC");
            break;

        case TASK2_STATE_FINISHED:
        default:
            if (g_task2_stop_reason == TASK2_STOP_A_MARKER)
            {
                OLED_ShowString(40U, 1U, "FINISHED");
                OLED_ShowString(31U, 7U, "STOP A MARKER");
            }
            else
            {
                OLED_ShowString(43U, 1U, "TIMEOUT");
                OLED_ShowString(34U, 7U, "SAFETY STOP");
            }
            break;
    }

    /* IR diagnostic: frame count on page 2 */
    snprintf(ir_buf, sizeof(ir_buf), "IR:%04lu",
             (unsigned long)g_ir_scan_count);
    OLED_ShowString(44U, 2U, ir_buf);

    Task2_DrawBigTime(display_ms);
}

bool Task2_DisplayInit(void)
{
    s_display_online = OLED_Init();
    s_last_state = (Task2_State_t)0xFFU;
    s_last_refresh_ms = App_Millis();
    s_last_centiseconds = 0xFFFFFFFFU;

    if (!s_display_online)
    {
        return false;
    }

    Task2_DrawStatePage();

    if (!OLED_Refresh())
    {
        s_display_online = false;
        return false;
    }

    s_last_state = g_task2_state;
    s_last_centiseconds = Task2_DisplayTimeMs() / 10U;
    return true;
}

void Task2_DisplayProcess(void)
{
    uint32_t now_ms;
    uint32_t centiseconds;

    if (!s_display_online)
    {
        return;
    }

    now_ms = App_Millis();

    if (g_task2_state != s_last_state)
    {
        Task2_DrawStatePage();

        if (!OLED_Refresh())
        {
            s_display_online = false;
            return;
        }

        s_last_state = g_task2_state;
        s_last_centiseconds = Task2_DisplayTimeMs() / 10U;
        s_last_refresh_ms = now_ms;
        return;
    }

    if ((g_task2_state != TASK2_STATE_LEAVING_A) &&
        (g_task2_state != TASK2_STATE_RUNNING))
    {
        return;
    }

    if ((uint32_t)(now_ms - s_last_refresh_ms) <
        TASK2_DISPLAY_REFRESH_MS)
    {
        return;
    }

    centiseconds = Task2_DisplayTimeMs() / 10U;

    if (centiseconds != s_last_centiseconds)
    {
        Task2_DrawBigTime(Task2_DisplayTimeMs());

        if (!OLED_RefreshPages(TASK2_TIME_FIRST_PAGE,
                               TASK2_TIME_PAGE_COUNT))
        {
            s_display_online = false;
            return;
        }

        s_last_centiseconds = centiseconds;
    }

    /* Also refresh IR diagnostic count every cycle so it updates live. */
    {
        char ir_buf[12];

        OLED_ShowString(44U, 2U, "       ");
        snprintf(ir_buf, sizeof(ir_buf), "IR:%04lu",
                 (unsigned long)g_ir_scan_count);
        OLED_ShowString(44U, 2U, ir_buf);

        if (!OLED_RefreshPages(2U, 1U))
        {
            s_display_online = false;
            return;
        }
    }

    s_last_refresh_ms = now_ms;
}

bool Task2_DisplayIsOnline(void)
{
    return s_display_online;
}

