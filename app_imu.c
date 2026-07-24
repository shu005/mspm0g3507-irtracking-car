/*
 * app_imu.c - UART IMU driver, frame parser and gyro-Z processing
 */

#include "app_imu.h"

/* SysConfig instance name must be UART_IMU. */
#ifndef IMU_UART_INST
#define IMU_UART_INST       UART_IMU_INST
#endif

#ifndef IMU_UART_IRQN
#define IMU_UART_IRQN       UART_IMU_INST_INT_IRQN
#endif

#define IMU_HEAD_1                  0x7EU
#define IMU_HEAD_2                  0x23U
#define IMU_RX_BUFFER_SIZE          512U
#define IMU_RX_BUFFER_MASK          (IMU_RX_BUFFER_SIZE - 1U)
#define IMU_MAX_FRAME_SIZE          64U
#define IMU_RAW_FRAME_LENGTH        0x17U
#define IMU_EULER_FRAME_LENGTH      0x11U

#define IMU_ACCEL_SCALE_G           (16.0f / 32767.0f)
#define IMU_GYRO_SCALE_DPS          (2000.0f / 32767.0f)
#define IMU_MAG_SCALE               (800.0f / 32767.0f)

#if ((IMU_RX_BUFFER_SIZE & (IMU_RX_BUFFER_SIZE - 1U)) != 0U)
#error "IMU_RX_BUFFER_SIZE must be a power of two"
#endif

IMU_Data_t g_imu_data;
IMU_Diagnostics_t g_imu_diag;

static volatile uint8_t  s_rx_buffer[IMU_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;

static uint8_t  s_frame[IMU_MAX_FRAME_SIZE];
static uint8_t  s_frame_index = 0U;
static uint8_t  s_expected_length = 0U;
static uint16_t s_raw_age_ms = 0xFFFFU;

static uint16_t s_calibration_target = 0U;
static uint16_t s_calibration_count = 0U;
static float    s_calibration_sum_dps = 0.0f;
static bool     s_filter_initialized = false;

static void IMU_RxPushFromISR(uint8_t byte);
static bool IMU_RxPop(uint8_t *byte);
static void IMU_ParseByte(uint8_t byte);
static void IMU_HandleFrame(const uint8_t *frame, uint8_t length);
static void IMU_HandleRawFrame(const uint8_t *frame);
static void IMU_HandleEulerFrame(const uint8_t *frame);
static int16_t IMU_ReadS16LE(const uint8_t *data);
static float IMU_ReadFloatLE(const uint8_t *data);
static void IMU_SendCommand7(uint8_t function, uint8_t parameter1,
                             uint8_t parameter2);

void IMU_Init(void)
{
    memset(&g_imu_data, 0, sizeof(g_imu_data));
    memset(&g_imu_diag, 0, sizeof(g_imu_diag));

    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_frame_index = 0U;
    s_expected_length = 0U;
    s_raw_age_ms = 0xFFFFU;
    s_filter_initialized = false;

    /* Remove stale bytes that may already be present after power-up. */
    while (!DL_UART_Main_isRXFIFOEmpty(IMU_UART_INST))
    {
        (void)DL_UART_Main_receiveData(IMU_UART_INST);
    }

    /* SysConfig already enables Receive, but enabling it here makes the
     * module self-contained and harmlessly repeats the same setting. */
    DL_UART_Main_enableInterrupt(IMU_UART_INST, DL_UART_MAIN_INTERRUPT_RX);

    NVIC_ClearPendingIRQ(IMU_UART_IRQN);
    NVIC_EnableIRQ(IMU_UART_IRQN);
    __enable_irq();
}

void IMU_Process(void)
{
    uint8_t byte;

    while (IMU_RxPop(&byte))
    {
        IMU_ParseByte(byte);
    }
}

void IMU_Tick(uint16_t elapsed_ms)
{
    if (s_raw_age_ms < 0xFFFFU)
    {
        uint32_t age = (uint32_t)s_raw_age_ms + (uint32_t)elapsed_ms;
        s_raw_age_ms = (age > 0xFFFFU) ? 0xFFFFU : (uint16_t)age;
    }
}

void IMU_StartGyroCalibration(uint16_t sample_count)
{
    if (sample_count == 0U)
    {
        sample_count = 1U;
    }

    s_calibration_target = sample_count;
    s_calibration_count = 0U;
    s_calibration_sum_dps = 0.0f;
    s_filter_initialized = false;

    g_imu_data.gyro_z_bias_dps = 0.0f;
    g_imu_data.gyro_z_filtered_dps = 0.0f;
    g_imu_data.calibrated = false;
    g_imu_data.calibrating = true;
}

bool IMU_IsOnline(void)
{
    return g_imu_data.raw_valid &&
           (s_raw_age_ms <= (uint16_t)IMU_ONLINE_TIMEOUT_MS);
}

bool IMU_IsCalibrated(void)
{
    return g_imu_data.calibrated;
}

float IMU_GetGyroZDps(void)
{
    if (!IMU_IsOnline() || !g_imu_data.calibrated)
    {
        return 0.0f;
    }

    return g_imu_data.gyro_z_filtered_dps;
}

float IMU_GetYawDeg(void)
{
    return g_imu_data.euler_valid ? g_imu_data.yaw_deg : 0.0f;
}

const IMU_Data_t *IMU_GetData(void)
{
    return &g_imu_data;
}

void IMU_SetOutputFrequency(uint8_t frequency_hz)
{
    if (frequency_hz < 10U)
    {
        frequency_hz = 10U;
    }
    else if (frequency_hz > 100U)
    {
        frequency_hz = 100U;
    }

    IMU_SendCommand7(0x60U, frequency_hz, 0x5FU);
}

void IMU_StartModuleCalibration(void)
{
    IMU_SendCommand7(0x70U, 0x01U, 0x5FU);
}

static void IMU_RxPushFromISR(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) & IMU_RX_BUFFER_MASK);

    if (next == s_rx_tail)
    {
        g_imu_diag.ring_overflows++;
        return;
    }

    s_rx_buffer[s_rx_head] = byte;
    s_rx_head = next;
    g_imu_diag.rx_bytes++;
}

static bool IMU_RxPop(uint8_t *byte)
{
    if (s_rx_tail == s_rx_head)
    {
        return false;
    }

    *byte = s_rx_buffer[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) & IMU_RX_BUFFER_MASK);
    return true;
}

static void IMU_ParseByte(uint8_t byte)
{
    if (s_frame_index == 0U)
    {
        if (byte == IMU_HEAD_1)
        {
            s_frame[0] = byte;
            s_frame_index = 1U;
        }
        return;
    }

    if (s_frame_index == 1U)
    {
        if (byte == IMU_HEAD_2)
        {
            s_frame[1] = byte;
            s_frame_index = 2U;
        }
        else if (byte == IMU_HEAD_1)
        {
            /* Treat this byte as a possible new first header byte. */
            s_frame[0] = byte;
            s_frame_index = 1U;
        }
        else
        {
            s_frame_index = 0U;
        }
        return;
    }

    if (s_frame_index == 2U)
    {
        if ((byte < 5U) || (byte > IMU_MAX_FRAME_SIZE))
        {
            g_imu_diag.length_errors++;
            s_frame_index = 0U;
            s_expected_length = 0U;
            return;
        }

        s_frame[2] = byte;
        s_expected_length = byte;
        s_frame_index = 3U;
        return;
    }

    if (s_frame_index >= IMU_MAX_FRAME_SIZE)
    {
        g_imu_diag.length_errors++;
        s_frame_index = 0U;
        s_expected_length = 0U;
        return;
    }

    s_frame[s_frame_index++] = byte;

    if (s_frame_index == s_expected_length)
    {
        uint8_t checksum = 0U;
        uint8_t i;

        for (i = 0U; i < (uint8_t)(s_expected_length - 1U); i++)
        {
            checksum = (uint8_t)(checksum + s_frame[i]);
        }

        if (checksum == s_frame[s_expected_length - 1U])
        {
            g_imu_diag.valid_frames++;
            IMU_HandleFrame(s_frame, s_expected_length);
        }
        else
        {
            g_imu_diag.checksum_errors++;
        }

        s_frame_index = 0U;
        s_expected_length = 0U;
    }
}

static void IMU_HandleFrame(const uint8_t *frame, uint8_t length)
{
    uint8_t function = frame[3];

    if ((function == IMU_FUNC_RAW_DATA) &&
        (length == IMU_RAW_FRAME_LENGTH))
    {
        IMU_HandleRawFrame(frame);
    }
    else if ((function == IMU_FUNC_EULER_ANGLE) &&
             (length == IMU_EULER_FRAME_LENGTH))
    {
        IMU_HandleEulerFrame(frame);
    }
    else
    {
        /* Other automatic-return frames are valid but not needed for
         * line-following control, so they are deliberately ignored. */
    }
}

static void IMU_HandleRawFrame(const uint8_t *frame)
{
    float gyro_z_dps;
    float corrected_z;

    g_imu_data.accel_raw[0] = IMU_ReadS16LE(&frame[4]);
    g_imu_data.accel_raw[1] = IMU_ReadS16LE(&frame[6]);
    g_imu_data.accel_raw[2] = IMU_ReadS16LE(&frame[8]);

    g_imu_data.gyro_raw[0] = IMU_ReadS16LE(&frame[10]);
    g_imu_data.gyro_raw[1] = IMU_ReadS16LE(&frame[12]);
    g_imu_data.gyro_raw[2] = IMU_ReadS16LE(&frame[14]);

    g_imu_data.mag_raw[0] = IMU_ReadS16LE(&frame[16]);
    g_imu_data.mag_raw[1] = IMU_ReadS16LE(&frame[18]);
    g_imu_data.mag_raw[2] = IMU_ReadS16LE(&frame[20]);

    g_imu_data.accel_g[0] = (float)g_imu_data.accel_raw[0] * IMU_ACCEL_SCALE_G;
    g_imu_data.accel_g[1] = (float)g_imu_data.accel_raw[1] * IMU_ACCEL_SCALE_G;
    g_imu_data.accel_g[2] = (float)g_imu_data.accel_raw[2] * IMU_ACCEL_SCALE_G;

    g_imu_data.gyro_dps[0] = (float)g_imu_data.gyro_raw[0] * IMU_GYRO_SCALE_DPS;
    g_imu_data.gyro_dps[1] = (float)g_imu_data.gyro_raw[1] * IMU_GYRO_SCALE_DPS;
    g_imu_data.gyro_dps[2] = (float)g_imu_data.gyro_raw[2] * IMU_GYRO_SCALE_DPS;

    g_imu_data.mag_scaled[0] = (float)g_imu_data.mag_raw[0] * IMU_MAG_SCALE;
    g_imu_data.mag_scaled[1] = (float)g_imu_data.mag_raw[1] * IMU_MAG_SCALE;
    g_imu_data.mag_scaled[2] = (float)g_imu_data.mag_raw[2] * IMU_MAG_SCALE;

    g_imu_data.raw_valid = true;
    s_raw_age_ms = 0U;

    gyro_z_dps = g_imu_data.gyro_dps[2];

    if (g_imu_data.calibrating)
    {
        s_calibration_sum_dps += gyro_z_dps;
        s_calibration_count++;
        g_imu_data.gyro_z_filtered_dps = 0.0f;

        if (s_calibration_count >= s_calibration_target)
        {
            g_imu_data.gyro_z_bias_dps =
                s_calibration_sum_dps / (float)s_calibration_count;
            g_imu_data.calibrating = false;
            g_imu_data.calibrated = true;
            g_imu_data.gyro_z_filtered_dps = 0.0f;
            s_filter_initialized = true;
        }
        return;
    }

    if (!g_imu_data.calibrated)
    {
        return;
    }

    corrected_z = gyro_z_dps - g_imu_data.gyro_z_bias_dps;

    if (!s_filter_initialized)
    {
        g_imu_data.gyro_z_filtered_dps = corrected_z;
        s_filter_initialized = true;
    }
    else
    {
        g_imu_data.gyro_z_filtered_dps +=
            IMU_GYRO_FILTER_ALPHA *
            (corrected_z - g_imu_data.gyro_z_filtered_dps);
    }
}

static void IMU_HandleEulerFrame(const uint8_t *frame)
{
    g_imu_data.roll_deg = IMU_ReadFloatLE(&frame[4]);
    g_imu_data.pitch_deg = IMU_ReadFloatLE(&frame[8]);
    g_imu_data.yaw_deg = IMU_ReadFloatLE(&frame[12]);
    g_imu_data.euler_valid = true;
}

static int16_t IMU_ReadS16LE(const uint8_t *data)
{
    uint16_t value = (uint16_t)data[0] |
                     ((uint16_t)data[1] << 8);
    return (int16_t)value;
}

static float IMU_ReadFloatLE(const uint8_t *data)
{
    uint32_t raw = (uint32_t)data[0] |
                   ((uint32_t)data[1] << 8) |
                   ((uint32_t)data[2] << 16) |
                   ((uint32_t)data[3] << 24);
    float value;

    memcpy(&value, &raw, sizeof(value));
    return value;
}

static void IMU_SendCommand7(uint8_t function, uint8_t parameter1,
                             uint8_t parameter2)
{
    uint8_t packet[7];
    uint8_t checksum = 0U;
    uint8_t i;

    packet[0] = IMU_HEAD_1;
    packet[1] = IMU_HEAD_2;
    packet[2] = 0x07U;
    packet[3] = function;
    packet[4] = parameter1;
    packet[5] = parameter2;

    for (i = 0U; i < 6U; i++)
    {
        checksum = (uint8_t)(checksum + packet[i]);
    }
    packet[6] = checksum;

    for (i = 0U; i < 7U; i++)
    {
        DL_UART_transmitDataBlocking(IMU_UART_INST, packet[i]);
    }
}

/* SysConfig maps this symbolic handler name to the physical UART3 ISR. */
void UART_IMU_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(IMU_UART_INST))
    {
        case DL_UART_MAIN_IIDX_RX:
            while (!DL_UART_Main_isRXFIFOEmpty(IMU_UART_INST))
            {
                IMU_RxPushFromISR(
                    (uint8_t)DL_UART_Main_receiveData(IMU_UART_INST));
            }
            break;

        default:
            break;
    }
}