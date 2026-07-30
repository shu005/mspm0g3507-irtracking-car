/*
 * app_irtracking.h - UART-based 8-channel IR tracking module interface
 *
 * Protocol (module UART, 115200 bps 8N1):
 *   MCU -> Module:  $0,0,1#          request digital data
 *   Module -> MCU:  $D,x1:V,x2:V,...,x8:V#
 *   V = 0 (white) / 1 (black)
 *
 * Hardware:
 *   MCU TX (PA21) -> Module RX
 *   MCU RX (PA22) -> Module TX
 *   UART2 peripheral, instance name "UART_IR" in SysConfig
 */

#ifndef APP_IRTRACKING_H
#define APP_IRTRACKING_H

#include "app_common.h"
#include <stdbool.h>

/* ========== Line tracking PID parameters ========== */
#define IRTrack_Trun_KP   (280)
#define IRTrack_Trun_KI   (0)
#define IRTrack_Trun_KD   (150)

/*
 * H题要求2使用椭圆环线，不包含直角弯。
 * 1 = 关闭原工程的直角弯阻塞控制，使用连续质心循迹。
 */
#define IR_H_OVAL_TRACK_MODE           1U

/* H题要求2的三档速度，单位沿用电机驱动板的速度指令。 */
#define IR_SPEED_FAST                 310
#define IR_SPEED_CURVE                300
#define IR_SPEED_APPROACH             260
#define IR_CURVE_ERROR_THRESHOLD        6
#define IR_CURVE_YAW_RATE_DPS          18.0f

/* 兼容工程内旧名称。 */
#define IRR_SPEED           IR_SPEED_FAST
#define IR_CONTROL_LOOP_MS   10U

/*
 * IR_SENSOR_REVERSE: 1 = reverse the physical x1..x8 order.
 * Set to 1 if the car steers opposite to the line direction.
 * Set to 0 for normal order (x1 = leftmost sensor).
 */
#define IR_SENSOR_REVERSE              1U

/* ========== UART receive configuration ========== */
/*
 * Frame timeout: if no valid frame arrives within this period the sensor is
 * considered offline and deal_IRdata() returns false, causing LineWalking()
 * to stop the car safely.
 */
#define IR_FRAME_TIMEOUT_MS           500U

/* ========== IMU angular-rate damping parameters ========== */
/* 1 = enabled; 0 = pure IR tracking. */
#define IMU_ASSIST_ENABLE              1

/* Motion_Car_Control uses positive Vz for a right turn.
 * With a common flat installation, raw gyro Z is often negative on a
 * right turn, so -1 converts it to the motor-control convention.
 * If oscillation becomes worse after enabling IMU, change to +1.0f. */
#define IMU_GYRO_Z_SIGN              (-1.0f)

#define IMU_DAMP_GAIN_CENTER           5.0f
#define IMU_DAMP_GAIN_TURN             2.0f
#define IMU_DAMP_FULL_GAIN_ERROR       6
#define IMU_DAMP_OUTPUT_LIMIT          800

/* ========== Debug variables for CCS Expressions / Watch ========== */
/* Raw 8-bit from latest UART frame, X1 at bit7, 1 = black. */
extern volatile uint8_t g_ir_raw_data;

/* Normalized legacy format: 0 = black, 1 = white; X1 at bit7. */
extern volatile uint8_t g_ir_line_data;

/* Convenience mask: 1 = black, 0 = white; X1 at bit7. */
extern volatile uint8_t g_ir_black_mask;

/* 当前一帧检测到黑色的传感器数量，范围0~8。 */
extern volatile uint8_t g_ir_black_count;

/* 当前质心循迹误差，负数偏左，正数偏右。 */
extern volatile int8_t g_ir_error;

/* Total number of valid UART frames received. */
extern volatile uint32_t g_ir_scan_count;

/* Monotonically increasing ID for each new frame. Used to detect fresh data. */
extern volatile uint32_t g_ir_frame_id;

/* Always 0xFF for UART (no physical channel multiplexing). */
extern volatile uint8_t g_ir_last_channel;

extern int pid_output_IRR;
extern u8 trun_flag;

/* ========== Public API ========== */

/* Initialize UART RX, flush FIFO, send $0,0,1# to start data stream. */
void IR_Init(void);

/* Call every main-loop iteration to parse incoming UART data. */
void IR_Process(void);

/* Called from the 10ms tick handler to age the frame watchdog. */
void IR_Tick(uint16_t elapsed_ms);

/*
 * Read latest UART frame and output normalized values.
 * Returns false when no valid frame has been received within the timeout.
 */
bool deal_IRdata(u8 *x1, u8 *x2, u8 *x3, u8 *x4,
                 u8 *x5, u8 *x6, u8 *x7, u8 *x8);

int32_t PID_IR_Calc(int8_t actual_value);

/* 设置/读取当前循迹基础速度。 */
void IR_SetBaseSpeed(int16_t speed);
int16_t IR_GetBaseSpeed(void);

/* 每次从A点重新开始前清空PID和历史误差。 */
void IR_ResetController(void);

void LineWalking(void);

#endif /* APP_IRTRACKING_H */
