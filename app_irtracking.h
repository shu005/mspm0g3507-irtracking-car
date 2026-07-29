/*
 * app_irtracking.h - 8-channel GPIO-multiplexed IR tracking module interface
 *
 * Hardware:
 *   AD0 -> PA14
 *   AD1 -> PA15
 *   AD2 -> PA16
 *   OUT -> PA17
 *
 * The module uses AD2:AD0 to select CH1~CH8 and exposes the selected
 * channel as a digital level on OUT. EN is pulled low on the module and
 * does not need an MCU pin.
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

/* ========== Multiplexed grayscale module configuration ========== */
/*
 * The manufacturer's MSPM0 car example defines ACTIVE_LEVEL as 1:
 * a sensor detecting the black line returns a high level.
 * deal_IRdata() normalizes this to the old control convention:
 * 0 = black and 1 = white.
 */
#define IR_BLACK_LEVEL                 1U

/*
 * Physical direction of the module after installation:
 *   1U: CH1/X1 is the leftmost sensor when looking in the car's forward direction.
 *   0U: CH1/X1 is the rightmost sensor; software reverses the channel order.
 */
#define IR_CH1_IS_LEFTMOST             1U

/*
 * The manufacturer's MSPM0 read example waits 50 us after changing AD2:AD0
 * before sampling OUT. At the current 32 MHz CPU clock this is 1600 cycles.
 */
#define IR_MUX_SETTLE_US              50U
#define IR_MUX_SETTLE_CYCLES          ((CPUCLK_FREQ / 1000000U) * IR_MUX_SETTLE_US)

/* Odd-number majority sampling suppresses a single transient on OUT. */
#define IR_GPIO_SAMPLE_COUNT           3U
#define IR_GPIO_SAMPLE_GAP_CYCLES     16U

/* ========== IMU angular-rate damping parameters ========== */
/* 1 = enabled; 0 = pure IR tracking. */
#define IMU_ASSIST_ENABLE              0

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
/* Electrical levels returned by the module, X1 at bit7 and X8 at bit0. */
extern volatile uint8_t g_ir_raw_data;

/* Normalized legacy format: 0 = black, 1 = white; X1 at bit7. */
extern volatile uint8_t g_ir_line_data;

/* Convenience mask: 1 = black, 0 = white; X1 at bit7. */
extern volatile uint8_t g_ir_black_mask;

/* 当前一帧检测到黑色的传感器数量，范围0~8。 */
extern volatile uint8_t g_ir_black_count;

/* 当前质心循迹误差，负数偏左，正数偏右。 */
extern volatile int8_t g_ir_error;

/* Total number of completed 8-channel scans. */
extern volatile uint32_t g_ir_scan_count;

/* Last physical multiplexer channel selected, range 0~7. */
extern volatile uint8_t g_ir_last_channel;

extern int pid_output_IRR;
extern u8 trun_flag;

/* Select physical CH1~CH8 using channel values 0~7. */
void IR_SelectChannel(uint8_t channel);

/* Read one physical channel as its raw electrical level (0 or 1). */
uint8_t IR_ReadChannelRaw(uint8_t channel);

/*
 * Scan all sensors and return raw electrical levels.
 * The returned bit order is always logical left-to-right:
 *   bit7 = X1/leftmost, ... bit0 = X8/rightmost.
 */
uint8_t IR_ReadAllRaw(void);

/*
 * Read and normalize all eight channels.
 * Outputs always use the legacy convention required by the existing
 * control logic: 0 = black, 1 = white.
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