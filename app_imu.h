/*
 * app_imu.h - UART IMU driver and parser
 *
 * Hardware used by this project:
 *   MSPM0 UART3 RX: PA25 <- IMU TX
 *   MSPM0 UART3 TX: PA26 -> IMU RX
 *   Baud rate: 115200, 8-N-1
 *
 * Protocol:
 *   0x7E 0x23 length function payload checksum
 *   length is the total frame length, including checksum.
 *   checksum is the low 8 bits of the sum from header1 to the
 *   byte immediately before checksum.
 */

#ifndef APP_IMU_H
#define APP_IMU_H

#include "app_common.h"
#include <stdbool.h>

/* IMU automatic-return function codes used by this driver. */
#define IMU_FUNC_RAW_DATA       0x04U
#define IMU_FUNC_EULER_ANGLE    0x26U

/* Data timeout. The module defaults to 25 Hz, so 250 ms allows
 * several consecutive frames to be missed before declaring offline. */
#define IMU_ONLINE_TIMEOUT_MS   250U

/* Software low-pass coefficient for corrected gyro Z.
 * Bigger = faster response and more noise; smaller = smoother and more lag. */
#define IMU_GYRO_FILTER_ALPHA   0.35f

typedef struct
{
    int16_t accel_raw[3];
    int16_t gyro_raw[3];
    int16_t mag_raw[3];

    float accel_g[3];
    float gyro_dps[3];
    float mag_scaled[3];

    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    float gyro_z_bias_dps;
    float gyro_z_filtered_dps;

    bool raw_valid;
    bool euler_valid;
    bool calibrated;
    bool calibrating;
} IMU_Data_t;

typedef struct
{
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t checksum_errors;
    uint32_t length_errors;
    uint32_t ring_overflows;
} IMU_Diagnostics_t;

/* Exposed for CCS Expressions / Watch debugging. */
extern IMU_Data_t g_imu_data;
extern IMU_Diagnostics_t g_imu_diag;

/* Enable UART interrupt and reset parser/buffers. */
void IMU_Init(void);

/* Parse bytes accumulated by the UART ISR. Call frequently from main loop. */
void IMU_Process(void);

/* Advance data age. Call with the actual elapsed milliseconds. */
void IMU_Tick(uint16_t elapsed_ms);

/* Start stationary software calibration of gyro Z bias. */
void IMU_StartGyroCalibration(uint16_t sample_count);

/* State and data accessors. */
bool IMU_IsOnline(void);
bool IMU_IsCalibrated(void);
float IMU_GetGyroZDps(void);
float IMU_GetYawDeg(void);
const IMU_Data_t *IMU_GetData(void);

/* Optional commands supported by the supplied protocol table. */
void IMU_SetOutputFrequency(uint8_t frequency_hz);
void IMU_StartModuleCalibration(void);

#endif /* APP_IMU_H */