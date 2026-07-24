/*
 * app_irtracking.c - 八路巡线模块实现
 * 8-channel IR line tracking implementation
 *
 * 接线:
 *   IR模块 SCL -> PA15
 *   IR模块 SDA -> PA16
 *   IR模块 5V  -> 5V
 *   IR模块 GND -> GND
 */

#include "app_irtracking.h"
#include "app_motor.h"
#include "app_imu.h"

/* ========== 直角弯四轮动力参数 ========== */
#define CORNER_OUTER_SPEED   500
#define CORNER_INNER_SPEED  (-700)
#define CORNER_SENSOR_MS      15
#define CORNER_TIMEOUT_MS    800

/* 中心附近仅保留原 PID 输出的 40%。 */
#define CENTER_ERROR_RANGE       2
#define CENTER_OUTPUT_PERCENT   40

int pid_output_IRR = 0;
u8  trun_flag = 0;

static int32_t IRTrack_Integral = 0;
static int8_t  error_last = 0;

#ifndef I2C_IR_INST
#define I2C_IR_INST  I2C_0_INST
#endif

static int32_t IR_ApplyIMUAssist(int32_t ir_output, int8_t error);
static void IR_CornerTurnLeft(void);
static void IR_CornerTurnRight(void);

uint8_t IRI2C_ReadByte(uint8_t reg)
{
    uint8_t data = 0;
    volatile uint32_t timeout;
    uint8_t regAddr = reg;

    DL_I2C_fillControllerTXFIFO(I2C_IR_INST, &regAddr, 1);
    DL_I2C_startControllerTransfer(I2C_IR_INST,
                                    (uint32_t)IR_I2C_DEVICE_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    timeout = 100000;
    while ((DL_I2C_getControllerStatus(I2C_IR_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0)
    {
        if (--timeout == 0)
        {
            return 0xFF;
        }
    }

    DL_I2C_startControllerTransfer(I2C_IR_INST,
                                    (uint32_t)IR_I2C_DEVICE_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_RX, 1);

    timeout = 100000;
    while (DL_I2C_isControllerRXFIFOEmpty(I2C_IR_INST))
    {
        if (--timeout == 0)
        {
            return 0xFF;
        }
    }

    data = DL_I2C_receiveControllerData(I2C_IR_INST);

    timeout = 100000;
    while ((DL_I2C_getControllerStatus(I2C_IR_INST) &
            DL_I2C_CONTROLLER_STATUS_IDLE) == 0)
    {
        if (--timeout == 0)
        {
            break;
        }
    }

    return data;
}

void deal_IRdata(u8 *x1, u8 *x2, u8 *x3, u8 *x4,
                 u8 *x5, u8 *x6, u8 *x7, u8 *x8)
{
    u8 IRbuf = IRI2C_ReadByte(IR_I2C_REG_DATA);

    *x1 = (IRbuf >> 7) & 0x01;
    *x2 = (IRbuf >> 6) & 0x01;
    *x3 = (IRbuf >> 5) & 0x01;
    *x4 = (IRbuf >> 4) & 0x01;
    *x5 = (IRbuf >> 3) & 0x01;
    *x6 = (IRbuf >> 2) & 0x01;
    *x7 = (IRbuf >> 1) & 0x01;
    *x8 = (IRbuf >> 0) & 0x01;
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
    if (IRTrack_Integral < -5000)
    {
        IRTrack_Integral = -5000;
    }

    deriv = error - error_last;

    output = (int32_t)error * (int32_t)IRTrack_Trun_KP
           + (int32_t)IRTrack_Trun_KI * IRTrack_Integral
           + (int32_t)deriv * (int32_t)IRTrack_Trun_KD;

    error_last = error;
    return output;
}

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

    /* Convert IMU sign to the same convention as motor Vz:
     * positive = right turn, negative = left turn. */
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

    /* Rate damping: oppose the car's current angular velocity.
     * It is most useful when the line error has returned toward zero but
     * the chassis is still rotating, which is exactly the overshoot stage. */
    return ir_output - damping;
#else
    (void)error;
    return ir_output;
#endif
}

static void IR_CornerTurnLeft(void)
{
    u8 x1, x2, x3, x4, x5, x6, x7, x8;
    uint32_t elapsed = 0;

    Contrl_Speed(CORNER_INNER_SPEED, CORNER_INNER_SPEED,
                 CORNER_OUTER_SPEED, CORNER_OUTER_SPEED);

    while (elapsed < CORNER_TIMEOUT_MS)
    {
        delay_ms(CORNER_SENSOR_MS);
        elapsed += CORNER_SENSOR_MS;

        IMU_Tick(CORNER_SENSOR_MS);
        IMU_Process();

        deal_IRdata(&x1, &x2, &x3, &x4, &x5, &x6, &x7, &x8);

        if (!(x1 == 0 && x2 == 0 && x3 == 0 && x4 == 0 &&
              x5 == 0 && x6 == 1 && x7 == 1 && x8 == 1))
        {
            if (x4 == 0 || x5 == 0)
            {
                break;
            }
        }
    }
}

static void IR_CornerTurnRight(void)
{
    u8 x1, x2, x3, x4, x5, x6, x7, x8;
    uint32_t elapsed = 0;

    Contrl_Speed(CORNER_OUTER_SPEED, CORNER_OUTER_SPEED,
                 CORNER_INNER_SPEED, CORNER_INNER_SPEED);

    while (elapsed < CORNER_TIMEOUT_MS)
    {
        delay_ms(CORNER_SENSOR_MS);
        elapsed += CORNER_SENSOR_MS;

        IMU_Tick(CORNER_SENSOR_MS);
        IMU_Process();

        deal_IRdata(&x1, &x2, &x3, &x4, &x5, &x6, &x7, &x8);

        if (!(x1 == 1 && x2 == 1 && x3 == 1 && x4 == 0 &&
              x5 == 0 && x6 == 0 && x7 == 0 && x8 == 0))
        {
            if (x4 == 0 || x5 == 0)
            {
                break;
            }
        }
    }
}

void LineWalking(void)
{
    static int8_t err = 0;
    static u8 x1, x2, x3, x4, x5, x6, x7, x8;
    u8 black_count;

    /* Parse all IMU bytes accumulated since the previous control cycle. */
    IMU_Process();

    deal_IRdata(&x1, &x2, &x3, &x4, &x5, &x6, &x7, &x8);

    black_count = (u8)((x1 == 0) + (x2 == 0) + (x3 == 0) + (x4 == 0)
                     + (x5 == 0) + (x6 == 0) + (x7 == 0) + (x8 == 0));

    if (x1 == 0 && x2 == 0 && x3 == 0 && x4 == 0 &&
        x5 == 0 && x6 == 1 && x7 == 1 && x8 == 1)
    {
        err = -15;
        IR_CornerTurnLeft();
        return;
    }
    else if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 0 &&
             x5 == 0 && x6 == 0 && x7 == 0 && x8 == 0)
    {
        err = 15;
        IR_CornerTurnRight();
        return;
    }
    else if (x1 == 0 && x2 == 0 && x3 == 0 && x4 == 0 &&
             x5 == 0 && x6 == 0 && x7 == 1 && x8 == 1)
    {
        err = -12;
    }
    else if (x1 == 1 && x2 == 1 && x3 == 0 && x4 == 0 &&
             x5 == 0 && x6 == 0 && x7 == 0 && x8 == 0)
    {
        err = 12;
    }
    else if (x1 == 0 && x8 == 1 && black_count >= 4)
    {
        err = -15;
        IR_CornerTurnLeft();
        return;
    }
    else if (x1 == 1 && x8 == 0 && black_count >= 4)
    {
        err = 15;
        IR_CornerTurnRight();
        return;
    }
    else
    {
        int count = 0;
        float sum = 0.0f;
        u8 sensors[8] = {x1, x2, x3, x4, x5, x6, x7, x8};
        int i;

        for (i = 0; i < 8; i++)
        {
            if (sensors[i] == 0)
            {
                count++;
                sum += (float)i;
            }
        }

        if (count > 0 && count < 8)
        {
            float centroid = sum / (float)count;
            err = (int8_t)((centroid - 3.5f) * 5.0f);
        }
        /* count == 0: lost line, retain the previous error.
         * count == 8: all black, retain the previous error. */
    }

    if (err > 20)
    {
        err = 20;
    }
    if (err < -20)
    {
        err = -20;
    }

    pid_output_IRR = PID_IR_Calc(err);

    if (err >= -CENTER_ERROR_RANGE && err <= CENTER_ERROR_RANGE)
    {
        pid_output_IRR =
            (pid_output_IRR * CENTER_OUTPUT_PERCENT) / 100;
    }

    pid_output_IRR = (int)IR_ApplyIMUAssist(pid_output_IRR, err);

    Motion_Car_Control(IRR_SPEED, 0, pid_output_IRR);
}