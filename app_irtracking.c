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

/* ========== I2C 安全参数 ========== */
/*
 * 轮询超时次数。该值不是毫秒，而是 while 循环的最大次数。
 * 用于防止红外模块掉线或总线异常时程序永久卡死。
 */
#define IR_I2C_TIMEOUT              100000U

/*
 * 普通循迹连续读取失败 2 次后停车。
 * 若希望任意一次失败都立即停车，可改为 1U。
 */
#define I2C_FAIL_STOP_COUNT              2U

/*
 * TI 官方 I2C 轮询示例要求：启动传输后先等待至少几个 I2C
 * 功能时钟周期，再读取 BUSY/IDLE 状态（I2C_ERR_13 规避）。
 * 当前工程为 32MHz，100 个 CPU 周期留有足够余量。
 */
#define IR_I2C_START_DELAY_CYCLES      100U

int pid_output_IRR = 0;
u8  trun_flag = 0;

/*
 * 可在 CCS Expressions / Watch 中观察：
 * g_ir_i2c_error_total：上电以来 I2C 读取失败总次数
 * g_ir_i2c_fail_streak：当前连续失败次数
 */
volatile uint32_t g_ir_i2c_error_total = 0U;
volatile uint8_t  g_ir_i2c_fail_streak = 0U;

static int32_t IRTrack_Integral = 0;
static int8_t  error_last = 0;

#ifndef I2C_IR_INST
#define I2C_IR_INST  I2C_0_INST
#endif

static bool IR_I2C_WaitIdle(void);
static void IR_I2C_AbortTransfer(void);
static void IR_ResetPIDState(void);
static void IR_RecordI2CFailure(bool stop_immediately);
static int32_t IR_ApplyIMUAssist(int32_t ir_output, int8_t error);
static void IR_CornerTurnLeft(void);
static void IR_CornerTurnRight(void);

/*
 * 等待 I2C 控制器和总线都回到空闲状态。
 * 等待期间同时检测控制器错误和仲裁丢失。
 */
static bool IR_I2C_WaitIdle(void)
{
    uint32_t timeout = IR_I2C_TIMEOUT;

    while (timeout > 0U)
    {
        uint32_t status = DL_I2C_getControllerStatus(I2C_IR_INST);

        if ((status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                       DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST)) != 0U)
        {
            return false;
        }

        if (((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) &&
            ((status & DL_I2C_CONTROLLER_STATUS_BUSY_BUS) == 0U))
        {
            return true;
        }

        timeout--;
    }

    return false;
}

/*
 * 中止当前控制器传输并清空 FIFO。
 * 这里只重置 I2C 传输状态，不重置整个 I2C 外设，
 * 因此不需要重新执行 SysConfig 初始化。
 */
static void IR_I2C_AbortTransfer(void)
{
    DL_I2C_resetControllerTransfer(I2C_IR_INST);
    DL_I2C_flushControllerTXFIFO(I2C_IR_INST);
    DL_I2C_flushControllerRXFIFO(I2C_IR_INST);
}

/*
 * 安全读取一个红外模块寄存器。
 *
 * 返回 true：
 *   通信成功，*data 为真实传感器数据；即使 *data == 0xFF，
 *   也表示合法的“八路全白”数据。
 *
 * 返回 false：
 *   通信超时、总线错误或仲裁丢失；此时禁止使用 *data。
 */
bool IRI2C_ReadByte(uint8_t reg, uint8_t *data)
{
    uint8_t reg_addr = reg;
    uint32_t timeout;
    uint32_t status;

    if (data == 0)
    {
        return false;
    }

    /*
     * 若上一次异常传输留下了无效 FIFO 数据，先清理。
     * 正常情况下控制器此时应为空闲。
     */
    if (!IR_I2C_WaitIdle())
    {
        goto read_failed;
    }

    DL_I2C_flushControllerTXFIFO(I2C_IR_INST);
    DL_I2C_flushControllerRXFIFO(I2C_IR_INST);

    if (DL_I2C_fillControllerTXFIFO(I2C_IR_INST, &reg_addr, 1U) != 1U)
    {
        goto read_failed;
    }

    /* 第一次传输：写入要读取的寄存器地址。 */
    DL_I2C_startControllerTransfer(
        I2C_IR_INST,
        (uint32_t)IR_I2C_DEVICE_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX,
        1U);

    delay_cycles(IR_I2C_START_DELAY_CYCLES);

    if (!IR_I2C_WaitIdle())
    {
        goto read_failed;
    }

    /* 第二次传输：从该寄存器读取 1 个字节。 */
    DL_I2C_startControllerTransfer(
        I2C_IR_INST,
        (uint32_t)IR_I2C_DEVICE_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX,
        1U);

    delay_cycles(IR_I2C_START_DELAY_CYCLES);

    timeout = IR_I2C_TIMEOUT;

    while (DL_I2C_isControllerRXFIFOEmpty(I2C_IR_INST))
    {
        status = DL_I2C_getControllerStatus(I2C_IR_INST);

        if ((status & (DL_I2C_CONTROLLER_STATUS_ERROR |
                       DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST)) != 0U)
        {
            goto read_failed;
        }

        if (timeout == 0U)
        {
            goto read_failed;
        }

        timeout--;
    }

    *data = DL_I2C_receiveControllerData(I2C_IR_INST);

    if (!IR_I2C_WaitIdle())
    {
        goto read_failed;
    }

    return true;

read_failed:
    IR_I2C_AbortTransfer();
    g_ir_i2c_error_total++;
    return false;
}

/*
 * 读取并拆分八路红外数据。
 * 只有 I2C 成功时才改写 x1~x8。
 */
bool deal_IRdata(u8 *x1, u8 *x2, u8 *x3, u8 *x4,
                 u8 *x5, u8 *x6, u8 *x7, u8 *x8)
{
    u8 IRbuf;

    if ((x1 == 0) || (x2 == 0) || (x3 == 0) || (x4 == 0) ||
        (x5 == 0) || (x6 == 0) || (x7 == 0) || (x8 == 0))
    {
        return false;
    }

    if (!IRI2C_ReadByte(IR_I2C_REG_DATA, &IRbuf))
    {
        return false;
    }

    *x1 = (IRbuf >> 7) & 0x01U;
    *x2 = (IRbuf >> 6) & 0x01U;
    *x3 = (IRbuf >> 5) & 0x01U;
    *x4 = (IRbuf >> 4) & 0x01U;
    *x5 = (IRbuf >> 3) & 0x01U;
    *x6 = (IRbuf >> 2) & 0x01U;
    *x7 = (IRbuf >> 1) & 0x01U;
    *x8 = (IRbuf >> 0) & 0x01U;

    return true;
}

static void IR_ResetPIDState(void)
{
    IRTrack_Integral = 0;
    error_last = 0;
    pid_output_IRR = 0;
}

/*
 * 记录一次 I2C 失败。
 *
 * 普通循迹：
 *   第一次偶发失败只放弃本次控制更新；
 *   连续达到 I2C_FAIL_STOP_COUNT 次后停车。
 *
 * 直角弯：
 *   正在使用较大差速强转，任何一次失败都立即停车。
 */
static void IR_RecordI2CFailure(bool stop_immediately)
{
    if (g_ir_i2c_fail_streak < 255U)
    {
        g_ir_i2c_fail_streak++;
    }

    if (stop_immediately ||
        (g_ir_i2c_fail_streak >= I2C_FAIL_STOP_COUNT))
    {
        Contrl_Speed(0, 0, 0, 0);
        IR_ResetPIDState();
    }
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

        if (!deal_IRdata(&x1, &x2, &x3, &x4,
                         &x5, &x6, &x7, &x8))
        {
            /*
             * 直角弯正在强转，读取失败一次就立即停车。
             * 保留失败计数，等待下一次 LineWalking 成功后清零。
             */
            IR_RecordI2CFailure(true);
            return;
        }

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

        if (!deal_IRdata(&x1, &x2, &x3, &x4,
                         &x5, &x6, &x7, &x8))
        {
            /*
             * 直角弯正在强转，读取失败一次就立即停车。
             * 保留失败计数，等待下一次 LineWalking 成功后清零。
             */
            IR_RecordI2CFailure(true);
            return;
        }

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
    bool recovered_from_i2c;

    /* Parse all IMU bytes accumulated since the previous control cycle. */
    IMU_Process();

    if (!deal_IRdata(&x1, &x2, &x3, &x4,
                     &x5, &x6, &x7, &x8))
    {
        /*
         * 读取失败时不使用旧的 x1~x8，不计算新的 PID。
         * 第一次偶发失败维持上一周期电机指令；
         * 连续失败达到阈值后四轮停车。
         */
        IR_RecordI2CFailure(false);
        return;
    }

    /*
     * 通信成功：
     * 记录是否刚从故障恢复，再清除连续失败计数。
     */
    recovered_from_i2c = (g_ir_i2c_fail_streak != 0U);
    g_ir_i2c_fail_streak = 0U;

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
        /*
         * count == 0：
         *   I2C 已确认读取成功，因此这是合法的 0xFF（全白丢线），
         *   保留上一次误差。
         *
         * count == 8：
         *   八路全黑，保留上一次误差。
         */
    }

    if (err > 20)
    {
        err = 20;
    }
    if (err < -20)
    {
        err = -20;
    }

    if (recovered_from_i2c)
    {
        /*
         * 通信恢复后的第一帧：
         * 清除积分，并用当前误差初始化微分历史，
         * 避免 KD 因故障前后的误差跳变产生瞬时冲击。
         */
        IRTrack_Integral = 0;
        error_last = err;
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