/*
 * app_irtracking.c - 8-channel GPIO-multiplexed IR tracking implementation
 *
 * Hardware connection:
 *   IR AD0 -> MSPM0 PA14
 *   IR AD1 -> MSPM0 PA15
 *   IR AD2 -> MSPM0 PA16
 *   IR OUT -> MSPM0 PA17
 *   IR 5V  -> 5V
 *   IR GND -> GND
 *
 * Module truth table:
 *   AD2 AD1 AD0 = 000 -> CH1
 *   AD2 AD1 AD0 = 001 -> CH2
 *   ...
 *   AD2 AD1 AD0 = 111 -> CH8
 *
 * SysConfig performs IOMUX and direction initialization. The scan timing and
 * active level follow the manufacturer MSPM0 reference implementation.
 */

#include "app_irtracking.h"
#include "app_motor.h"

/* ========== SysConfig-generated hardware mapping ========== */
#define IR_HW_PORT      IR_GPIO_PORT
#define IR_AD0_PIN      IR_GPIO_IR_AD0_PIN
#define IR_AD1_PIN      IR_GPIO_IR_AD1_PIN
#define IR_AD2_PIN      IR_GPIO_IR_AD2_PIN
#define IR_OUT_PIN      IR_GPIO_IR_OUT_PIN

/* ========== Right-angle corner parameters ========== */
#if (IR_H_OVAL_TRACK_MODE == 0U)
#define CORNER_OUTER_SPEED   400
#define CORNER_INNER_SPEED  (-400)
#define CORNER_SENSOR_MS      15U
#define CORNER_TIMEOUT_MS    800U
#endif

/* Keep only 40% of PID output near the center. */
#define CENTER_ERROR_RANGE       2
#define CENTER_OUTPUT_PERCENT   40

int pid_output_IRR = 0;
u8  trun_flag = 0;

volatile uint8_t  g_ir_raw_data    = 0xFFU;
volatile uint8_t  g_ir_line_data   = 0xFFU;
volatile uint8_t  g_ir_black_mask  = 0x00U;
volatile uint8_t  g_ir_black_count = 0U;
volatile int8_t   g_ir_error       = 0;
volatile uint32_t g_ir_scan_count  = 0U;
volatile uint8_t  g_ir_last_channel = 0U;

static int32_t IRTrack_Integral = 0;
static int8_t  error_last = 0;
static int8_t  s_line_error = 0;
static int16_t s_ir_base_speed = IR_SPEED_FAST;

static void IR_ResetPIDState(void);
static int32_t IR_ApplyIMUAssist(int32_t ir_output, int8_t error);
#if (IR_H_OVAL_TRACK_MODE == 0U)
static void IR_CornerTurnLeft(void);
static void IR_CornerTurnRight(void);
#endif

static void IR_WriteAddressPin(uint32_t pin, bool high)
{
    if (high)
    {
        DL_GPIO_setPins(IR_HW_PORT, pin);
    }
    else
    {
        DL_GPIO_clearPins(IR_HW_PORT, pin);
    }
}

void IR_SelectChannel(uint8_t channel)
{
    channel &= 0x07U;

    IR_WriteAddressPin(IR_AD0_PIN, (channel & 0x01U) != 0U);
    IR_WriteAddressPin(IR_AD1_PIN, (channel & 0x02U) != 0U);
    IR_WriteAddressPin(IR_AD2_PIN, (channel & 0x04U) != 0U);

    g_ir_last_channel = channel;

    /* Allow the analog switch/comparator output to settle. */
    delay_cycles(IR_MUX_SETTLE_CYCLES);
}

uint8_t IR_ReadChannelRaw(uint8_t channel)
{
    uint8_t high_count = 0U;
    uint8_t sample;

    IR_SelectChannel(channel);

    for (sample = 0U; sample < IR_GPIO_SAMPLE_COUNT; sample++)
    {
        if (DL_GPIO_readPins(IR_HW_PORT, IR_OUT_PIN) != 0U)
        {
            high_count++;
        }

        if ((sample + 1U) < IR_GPIO_SAMPLE_COUNT)
        {
            delay_cycles(IR_GPIO_SAMPLE_GAP_CYCLES);
        }
    }

    return (high_count > (IR_GPIO_SAMPLE_COUNT / 2U)) ? 1U : 0U;
}

uint8_t IR_ReadAllRaw(void)
{
    uint8_t data = 0U;
    uint8_t logical_index;

    for (logical_index = 0U; logical_index < 8U; logical_index++)
    {
        uint8_t physical_channel;
        uint8_t raw_level;
        uint8_t bit_position = (uint8_t)(7U - logical_index);

#if IR_CH1_IS_LEFTMOST
        physical_channel = logical_index;
#else
        physical_channel = (uint8_t)(7U - logical_index);
#endif

        raw_level = IR_ReadChannelRaw(physical_channel);

        if (raw_level != 0U)
        {
            data |= (uint8_t)(1U << bit_position);
        }
    }

    g_ir_raw_data = data;
    g_ir_scan_count++;
    return data;
}

bool deal_IRdata(u8 *x1, u8 *x2, u8 *x3, u8 *x4,
                 u8 *x5, u8 *x6, u8 *x7, u8 *x8)
{
    uint8_t raw_data;
    uint8_t line_data = 0U;
    uint8_t bit_position;

    if ((x1 == 0) || (x2 == 0) || (x3 == 0) || (x4 == 0) ||
        (x5 == 0) || (x6 == 0) || (x7 == 0) || (x8 == 0))
    {
        return false;
    }

    raw_data = IR_ReadAllRaw();

    /* Normalize to the old program's convention: 0 = black, 1 = white. */
    for (bit_position = 0U; bit_position < 8U; bit_position++)
    {
        uint8_t raw_level = (uint8_t)((raw_data >> bit_position) & 0x01U);
        uint8_t normalized_level =
            (raw_level == IR_BLACK_LEVEL) ? 0U : 1U;

        line_data |= (uint8_t)(normalized_level << bit_position);
    }

    g_ir_line_data  = line_data;
    g_ir_black_mask = (uint8_t)(~line_data);

    *x1 = (u8)((line_data >> 7) & 0x01U);
    *x2 = (u8)((line_data >> 6) & 0x01U);
    *x3 = (u8)((line_data >> 5) & 0x01U);
    *x4 = (u8)((line_data >> 4) & 0x01U);
    *x5 = (u8)((line_data >> 3) & 0x01U);
    *x6 = (u8)((line_data >> 2) & 0x01U);
    *x7 = (u8)((line_data >> 1) & 0x01U);
    *x8 = (u8)((line_data >> 0) & 0x01U);

    return true;
}

static void IR_ResetPIDState(void)
{
    IRTrack_Integral = 0;
    error_last = 0;
    pid_output_IRR = 0;
}

void IR_ResetController(void)
{
    IR_ResetPIDState();
    s_line_error = 0;
    g_ir_error = 0;
}

void IR_SetBaseSpeed(int16_t speed)
{
    if (speed < 0)
    {
        speed = 0;
    }
    else if (speed > 1000)
    {
        speed = 1000;
    }

    s_ir_base_speed = speed;
}

int16_t IR_GetBaseSpeed(void)
{
    return s_ir_base_speed;
}

int32_t PID_IR_Calc(int8_t actual_value)
{
    int32_t output;
    int8_t error = actual_value;
    int8_t deriv;

    IRTrack_Integral += (int32_t)error;

    if (IRTrack_Integral > 5000)
    {
        IRTrack_Integral = 5000;
    }
    else if (IRTrack_Integral < -5000)
    {
        IRTrack_Integral = -5000;
    }

    deriv = (int8_t)(error - error_last);

    output = (int32_t)error * (int32_t)IRTrack_Trun_KP
           + (int32_t)IRTrack_Trun_KI * IRTrack_Integral
           + (int32_t)deriv * (int32_t)IRTrack_Trun_KD;

    error_last = error;
    return output;
}

static int32_t IR_ApplyIMUAssist(int32_t ir_output, int8_t error)
{
    (void)error;
    return ir_output;
}

#if (IR_H_OVAL_TRACK_MODE == 0U)
static void IR_CornerTurnLeft(void)
{
    u8 x1, x2, x3, x4, x5, x6, x7, x8;
    uint32_t elapsed = 0U;

    Contrl_Speed(CORNER_INNER_SPEED, CORNER_INNER_SPEED,
                 CORNER_OUTER_SPEED, CORNER_OUTER_SPEED);

    while (elapsed < CORNER_TIMEOUT_MS)
    {
        delay_ms(CORNER_SENSOR_MS);
        elapsed += CORNER_SENSOR_MS;

        if (!deal_IRdata(&x1, &x2, &x3, &x4,
                         &x5, &x6, &x7, &x8))
        {
            Contrl_Speed(0, 0, 0, 0);
            IR_ResetPIDState();
            return;
        }

        /* Exit when the extreme-left pattern has eased and the line has
         * re-entered the inner sensor area. */
        if (!(x1 == 0U && x2 == 0U && x3 == 0U && x4 == 0U))
        {
            if (x2 == 0U || x3 == 0U || x4 == 0U ||
                x5 == 0U || x6 == 0U || x7 == 0U)
            {
                break;
            }
        }
    }
}

static void IR_CornerTurnRight(void)
{
    u8 x1, x2, x3, x4, x5, x6, x7, x8;
    uint32_t elapsed = 0U;

    Contrl_Speed(CORNER_OUTER_SPEED, CORNER_OUTER_SPEED,
                 CORNER_INNER_SPEED, CORNER_INNER_SPEED);

    while (elapsed < CORNER_TIMEOUT_MS)
    {
        delay_ms(CORNER_SENSOR_MS);
        elapsed += CORNER_SENSOR_MS;

        if (!deal_IRdata(&x1, &x2, &x3, &x4,
                         &x5, &x6, &x7, &x8))
        {
            Contrl_Speed(0, 0, 0, 0);
            IR_ResetPIDState();
            return;
        }

        /* Exit when the extreme-right pattern has eased and the line has
         * re-entered the inner sensor area. */
        if (!(x5 == 0U && x6 == 0U && x7 == 0U && x8 == 0U))
        {
            if (x2 == 0U || x3 == 0U || x4 == 0U ||
                x5 == 0U || x6 == 0U || x7 == 0U)
            {
                break;
            }
        }
    }
}
#endif

void LineWalking(void)
{
    static u8 x1, x2, x3, x4, x5, x6, x7, x8;
    u8 black_count;
    int16_t actual_speed;
    int8_t abs_error;

    if (!deal_IRdata(&x1, &x2, &x3, &x4,
                     &x5, &x6, &x7, &x8))
    {
        Contrl_Speed(0, 0, 0, 0);
        IR_ResetPIDState();
        return;
    }

    black_count = (u8)((x1 == 0U) + (x2 == 0U) + (x3 == 0U) + (x4 == 0U)
                     + (x5 == 0U) + (x6 == 0U) + (x7 == 0U) + (x8 == 0U));
    g_ir_black_count = black_count;

#if (IR_H_OVAL_TRACK_MODE == 0U)
    if (x1 == 0U && x2 == 0U && x3 == 0U && x4 == 0U &&
        x5 == 0U && x6 == 1U && x7 == 1U && x8 == 1U)
    {
        s_line_error = -15;
        IR_CornerTurnLeft();
        return;
    }
    else if (x1 == 1U && x2 == 1U && x3 == 1U && x4 == 0U &&
             x5 == 0U && x6 == 0U && x7 == 0U && x8 == 0U)
    {
        s_line_error = 15;
        IR_CornerTurnRight();
        return;
    }
    else if (x1 == 0U && x2 == 0U && x3 == 0U && x4 == 0U &&
             x5 == 0U && x6 == 0U && x7 == 1U && x8 == 1U)
    {
        s_line_error = -15;
        IR_CornerTurnLeft();
        return;
    }
    else if (x1 == 1U && x2 == 1U && x3 == 0U && x4 == 0U &&
             x5 == 0U && x6 == 0U && x7 == 0U && x8 == 0U)
    {
        s_line_error = 15;
        IR_CornerTurnRight();
        return;
    }
    else if (x1 == 0U && x8 == 1U && black_count >= 4U)
    {
        s_line_error = -15;
        IR_CornerTurnLeft();
        return;
    }
    else if (x1 == 1U && x8 == 0U && black_count >= 4U)
    {
        s_line_error = 15;
        IR_CornerTurnRight();
        return;
    }
    else
#endif
    {
        int count = 0;
        float sum = 0.0f;
        u8 sensors[8] = {x1, x2, x3, x4, x5, x6, x7, x8};
        int i;

        for (i = 0; i < 8; i++)
        {
            if (sensors[i] == 0U)
            {
                count++;
                sum += (float)i;
            }
        }

        if (count > 0 && count < 8)
        {
            float centroid = sum / (float)count;
            s_line_error = (int8_t)((centroid - 3.5f) * 5.0f);
        }
        /*
         * count == 0: all white / line lost, keep the previous error.
         * count == 8: all black, keep the previous error.
         */
    }

    if (s_line_error > 20)
    {
        s_line_error = 20;
    }
    else if (s_line_error < -20)
    {
        s_line_error = -20;
    }

    g_ir_error = s_line_error;
    pid_output_IRR = PID_IR_Calc(s_line_error);

    if (s_line_error >= -CENTER_ERROR_RANGE &&
        s_line_error <= CENTER_ERROR_RANGE)
    {
        pid_output_IRR =
            (pid_output_IRR * CENTER_OUTPUT_PERCENT) / 100;
    }

    pid_output_IRR =
        (int)IR_ApplyIMUAssist(pid_output_IRR, s_line_error);

    actual_speed = s_ir_base_speed;
    abs_error = (s_line_error >= 0) ?
                s_line_error : (int8_t)(-s_line_error);

    if ((abs_error >= IR_CURVE_ERROR_THRESHOLD) &&
        (actual_speed > IR_SPEED_CURVE))
    {
        actual_speed = IR_SPEED_CURVE;
    }

    Motion_Car_Control(actual_speed, 0, pid_output_IRR);
}