/*
 * app_irtracking.c - UART-based 8-channel IR tracking module
 *
 * Protocol:
 *   MCU -> Module:  $0,0,1#
 *   Module -> MCU:  $D,x1:0,x2:0,x3:0,x4:0,x5:0,x6:0,x7:0,x8:0#
 *
 *   Module value convention: 0 = white, 1 = black.
 *   This driver inverts to the legacy convention: 0 = black, 1 = white.
 *
 * Hardware:
 *   MCU TX (PA21) -> Module RX
 *   MCU RX (PA22) -> Module TX
 *   UART2 peripheral, SysConfig instance name "UART_IR"
 */

#include "app_irtracking.h"
#include "app_motor.h"
#include "app_imu.h"
#include "ti_msp_dl_config.h"

#include <string.h>

/* SysConfig instance name must be UART_IR. */
#ifndef IR_UART_INST
#define IR_UART_INST       UART_IR_INST
#endif

#ifndef IR_UART_IRQN
#define IR_UART_IRQN       UART_IR_INST_INT_IRQN
#endif

#define IR_RX_BUF_SIZE     256U
#define IR_RX_BUF_MASK     ((IR_RX_BUF_SIZE) - 1U)
#define IR_FRAME_BUF_SIZE   64U

#if ((IR_RX_BUF_SIZE & (IR_RX_BUF_SIZE - 1U)) != 0U)
#error "IR_RX_BUF_SIZE must be a power of two"
#endif

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
volatile uint8_t  g_ir_last_channel = 0xFFU;

/* ========== UART receive ring buffer ========== */
static volatile uint8_t  s_rx_buf[IR_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;

/* ========== Frame assembly ========== */
static char    s_frame_buf[IR_FRAME_BUF_SIZE];
static uint8_t s_frame_idx = 0U;

/* ========== Latest parsed sensor values ========== */
/* Module convention: 0 = white, 1 = black; x1 at index 0. */
static volatile uint8_t  s_ir_raw_values[8];
static volatile bool     s_ir_frame_valid = false;
static volatile uint16_t s_ir_frame_age_ms = 0xFFFFU;

/*
 * Frame ID: incremented each time a valid $D frame is parsed.
 * LineWalking reads this to detect fresh data and avoid
 * re-running PID on the same sensor values.
 */
volatile uint32_t g_ir_frame_id = 0U;

/* ========== PID and tracking state ========== */
static int32_t IRTrack_Integral = 0;
static int8_t  error_last = 0;
static int8_t  s_line_error = 0;
static int16_t s_ir_base_speed = IR_SPEED_FAST;

/* ========== Forward declarations ========== */
static void IR_ResetPIDState(void);
static int32_t IR_ApplyIMUAssist(int32_t ir_output, int8_t error);
#if (IR_H_OVAL_TRACK_MODE == 0U)
static void IR_CornerTurnLeft(void);
static void IR_CornerTurnRight(void);
#endif

/* UART helpers */
static void IR_RxPushFromISR(uint8_t byte);
static bool IR_RxPop(uint8_t *byte);
static void IR_SendString(const char *str);
static void IR_ParseByte(uint8_t byte);
static void IR_ParseFrame(const char *frame);

/* ========== Initialization ========== */

void IR_Init(void)
{
    uint8_t i;

    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_frame_idx = 0U;
    s_ir_frame_valid = false;
    s_ir_frame_age_ms = 0xFFFFU;

    for (i = 0U; i < 8U; i++)
    {
        s_ir_raw_values[i] = 0U;
    }

    g_ir_raw_data    = 0xFFU;
    g_ir_line_data   = 0xFFU;
    g_ir_black_mask  = 0x00U;
    g_ir_black_count = 0U;
    g_ir_error       = 0;
    g_ir_scan_count  = 0U;
    g_ir_last_channel = 0xFFU;

    IR_ResetController();

    /* Flush any stale bytes from the RX FIFO. */
    while (!DL_UART_Main_isRXFIFOEmpty(IR_UART_INST))
    {
        (void)DL_UART_Main_receiveData(IR_UART_INST);
    }

    /* Enable RX interrupt. SysConfig already configures the UART peripheral. */
    DL_UART_Main_enableInterrupt(IR_UART_INST, DL_UART_MAIN_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(IR_UART_IRQN);
    NVIC_EnableIRQ(IR_UART_IRQN);
    __enable_irq();

    /* Request digital data stream from the module. */
    IR_SendString("$0,0,1#");
}

/* ========== Main-loop processing ========== */

void IR_Process(void)
{
    uint8_t byte;

    while (IR_RxPop(&byte))
    {
        IR_ParseByte(byte);
    }
}

void IR_Tick(uint16_t elapsed_ms)
{
    if (s_ir_frame_age_ms < 0xFFFFU)
    {
        uint32_t age = (uint32_t)s_ir_frame_age_ms + (uint32_t)elapsed_ms;
        s_ir_frame_age_ms = (age > 0xFFFFU) ? 0xFFFFU : (uint16_t)age;
    }
}

/* ========== UART send / receive ========== */

static void IR_SendString(const char *str)
{
    while (*str != '\0')
    {
        DL_UART_transmitDataBlocking(IR_UART_INST, (uint8_t)*str);
        str++;
    }
}

static void IR_RxPushFromISR(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) & IR_RX_BUF_MASK);

    if (next == s_rx_tail)
    {
        return; /* Buffer full, drop byte. */
    }

    s_rx_buf[s_rx_head] = byte;
    s_rx_head = next;
}

static bool IR_RxPop(uint8_t *byte)
{
    if (s_rx_tail == s_rx_head)
    {
        return false;
    }

    *byte = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) & IR_RX_BUF_MASK);
    return true;
}

/* ========== Frame parser ========== */

static void IR_ParseByte(uint8_t byte)
{
    char c = (char)byte;

    /* '$' always starts a new frame, discarding any partial previous one. */
    if (c == '$')
    {
        s_frame_idx = 0U;
        s_frame_buf[0] = c;
        s_frame_idx = 1U;
        return;
    }

    /* Ignore bytes received before the first '$'. */
    if (s_frame_idx == 0U)
    {
        return;
    }

    /* Append to frame buffer if space permits. */
    if (s_frame_idx < (IR_FRAME_BUF_SIZE - 1U))
    {
        s_frame_buf[s_frame_idx] = c;
        s_frame_idx++;
    }

    /* '#', '\n', or '\r' terminates a frame. */
    if ((c == '#') || (c == '\n') || (c == '\r'))
    {
        s_frame_buf[s_frame_idx] = '\0';
        IR_ParseFrame(s_frame_buf);
        s_frame_idx = 0U;
    }
}

/*
 * Parse a digital-data frame: $D,x1:V,x2:V,x3:V,x4:V,x5:V,x6:V,x7:V,x8:V#
 * V is '0' (white) or '1' (black).
 *
 * The parser is deliberately strict: any format deviation causes the frame
 * to be silently dropped, preventing corrupted data from steering the car.
 */
static void IR_ParseFrame(const char *frame)
{
    uint8_t values[8];
    int     field;
    const char *p = frame;

    /* Must start with "$D,". */
    if ((p[0] != '$') || (p[1] != 'D') || (p[2] != ','))
    {
        return;
    }

    p += 3; /* Skip "$D," */

    for (field = 0; field < 8; field++)
    {
        /* Expect "x" */
        if (*p != 'x')
        {
            return;
        }
        p++;

        /* Skip sensor number digit(s) */
        while ((*p >= '0') && (*p <= '9'))
        {
            p++;
        }

        /* Expect ":" */
        if (*p != ':')
        {
            return;
        }
        p++;

        /* Parse value: '0' or '1' */
        if (*p == '0')
        {
            values[field] = 0U;
        }
        else if (*p == '1')
        {
            values[field] = 1U;
        }
        else
        {
            return;
        }
        p++;

        /* Expect ',' (fields 0-6) or '#' (field 7) */
        if (field < 7)
        {
            if (*p != ',')
            {
                return;
            }
            p++;
        }
    }

    /* Store parsed values atomically. */
    for (field = 0; field < 8; field++)
    {
        s_ir_raw_values[field] = values[field];
    }

    s_ir_frame_valid = true;
    s_ir_frame_age_ms = 0U;
    g_ir_scan_count++;
    g_ir_frame_id++;
}

/* ========== deal_IRdata (reads cached UART frame) ========== */

bool deal_IRdata(u8 *x1, u8 *x2, u8 *x3, u8 *x4,
                 u8 *x5, u8 *x6, u8 *x7, u8 *x8)
{
    uint8_t raw_data  = 0U;
    uint8_t line_data = 0U;
    uint8_t i;

    if ((x1 == 0) || (x2 == 0) || (x3 == 0) || (x4 == 0) ||
        (x5 == 0) || (x6 == 0) || (x7 == 0) || (x8 == 0))
    {
        return false;
    }

    /* Safety: no valid frame within the timeout window. */
    if (!s_ir_frame_valid ||
        (s_ir_frame_age_ms > (uint16_t)IR_FRAME_TIMEOUT_MS))
    {
        return false;
    }

    /*
     * Build raw_data and line_data from the cached module values.
     *
     * Module convention: 1 = black
     * Legacy convention:  0 = black
     *
     * raw_data:  bit = 1 when module reports black (matches g_ir_black_mask)
     * line_data: bit = 0 when module reports black (legacy)
     */
    for (i = 0U; i < 8U; i++)
    {
        uint8_t sensor_idx;
        uint8_t bit_pos = (uint8_t)(7U - i); /* x1 at bit7 */

#if IR_SENSOR_REVERSE
        sensor_idx = (uint8_t)(7U - i);
#else
        sensor_idx = i;
#endif

        if (s_ir_raw_values[sensor_idx] != 0U)
        {
            /* Module says black: set bit in raw_data, clear bit in line_data. */
            raw_data |= (uint8_t)(1U << bit_pos);
            /* line_data bit stays 0 → legacy "0 = black" */
        }
        else
        {
            /* Module says white: set bit in line_data. */
            line_data |= (uint8_t)(1U << bit_pos);
        }
    }

    g_ir_raw_data   = raw_data;
    g_ir_line_data  = line_data;
    g_ir_black_mask = raw_data; /* 1 = black */

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

/* ========== PID controller ========== */

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

/* ========== IMU angular-rate damping ========== */

static int32_t IR_ApplyIMUAssist(int32_t ir_output, int8_t error)
{
#if IMU_ASSIST_ENABLE
    int8_t abs_error;
    float yaw_rate;
    float gain;
    int32_t damping;

    if (!IMU_IsOnline() || !IMU_IsCalibrated())
    {
        return ir_output;
    }

    abs_error = (error >= 0) ? error : (int8_t)(-error);
    gain = (abs_error <= IMU_DAMP_FULL_GAIN_ERROR) ?
           IMU_DAMP_GAIN_CENTER : IMU_DAMP_GAIN_TURN;

    yaw_rate = IMU_GetGyroZDps() * IMU_GYRO_Z_SIGN;
    damping = (int32_t)(yaw_rate * gain);

    if (damping > IMU_DAMP_OUTPUT_LIMIT)
    {
        damping = IMU_DAMP_OUTPUT_LIMIT;
    }
    else if (damping < -IMU_DAMP_OUTPUT_LIMIT)
    {
        damping = -IMU_DAMP_OUTPUT_LIMIT;
    }

    return ir_output - damping;
#else
    (void)error;
    return ir_output;
#endif
}

/* ========== Right-angle corner handling (disabled in oval-track mode) ========== */

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

/* ========== LineWalking — centroid-based PID tracking ========== */

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

/* ========== UART interrupt handler ========== */

void UART_IR_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(IR_UART_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(IR_UART_INST))
            {
                IR_RxPushFromISR(
                    (uint8_t)DL_UART_Main_receiveData(IR_UART_INST));
            }
            break;

        default:
            break;
    }
}
