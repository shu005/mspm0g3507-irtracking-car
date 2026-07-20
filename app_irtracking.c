/*
 * app_irtracking.c - 八路巡线模块实现
 * 8-channel IR line tracking implementation
 *
 * 工作原理:
 *   八路 IR 传感器通过 I2C 返回一个字节, 8 个 bit 对应 8 个传感器状态
 *   0 = 检测到黑线 (灯亮), 1 = 白底 (灯灭)
 *   使用质心法计算偏移量, PID 控制小车转向
 *
 * 接线:
 *   IR模块 SCL -> PA15
 *   IR模块 SDA -> PA16
 *   IR模块 5V  -> 5V
 *   IR模块 GND -> GND
 */

#include "app_irtracking.h"
#include "app_motor.h"

/* ========== 全局变量 ========== */
int  pid_output_IRR = 0;       /* PID 输出 (传给电机控制) */
u8   trun_flag = 0;            /* 转向标志 (预留) */

/* ========== PID 变量 ========== */
static float IRTrack_Integral = 0;   /* 积分累积 */
static int8_t error_last = 0;        /* 上一次误差 */

/* ========== I2C 实例 ========== */
/*
 * I2C 实例由 SysConfig 生成
 * 如果 SysConfig 里 I2C 命名不同, 修改下面的宏
 */
#ifndef I2C_IR_INST
#define I2C_IR_INST  I2C_0_INST
#endif

/* ========== I2C 读取 IR 传感器 ========== */
/*
 * 从设备 0x12 的寄存器 0x30 读取一个字节
 * 返回: bit7=X1(最左), bit0=X8(最右), 0=检测到黑线
 *
 * I2C 协议: 先写寄存器地址, 再读数据
 *   START + DEV_ADDR(W) + REG_ADDR + RESTART + DEV_ADDR(R) + DATA + STOP
 */
uint8_t IRI2C_ReadByte(uint8_t reg)
{
    uint8_t data = 0;
    volatile uint32_t timeout;

    /* 第一步: 写寄存器地址到 TX FIFO, 启动 TX 传输 */
    uint8_t regAddr = reg;
    DL_I2C_fillControllerTXFIFO(I2C_IR_INST, &regAddr, 1);
    DL_I2C_startControllerTransfer(I2C_IR_INST, (uint32_t)IR_I2C_DEVICE_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    /* 等待 TX 完成 (总线空闲) */
    timeout = 100000;
    while ((DL_I2C_getControllerStatus(I2C_IR_INST)
            & DL_I2C_CONTROLLER_STATUS_IDLE) == 0) {
        if (--timeout == 0) return 0xFF;
    }

    /* 第二步: 读 1 字节数据 */
    DL_I2C_startControllerTransfer(I2C_IR_INST, (uint32_t)IR_I2C_DEVICE_ADDR,
                                    DL_I2C_CONTROLLER_DIRECTION_RX, 1);

    /* 等待 RX FIFO 有数据 */
    timeout = 100000;
    while (DL_I2C_isControllerRXFIFOEmpty(I2C_IR_INST)) {
        if (--timeout == 0) return 0xFF;
    }
    data = DL_I2C_receiveControllerData(I2C_IR_INST);

    /* 等待总线空闲 */
    timeout = 100000;
    while ((DL_I2C_getControllerStatus(I2C_IR_INST)
            & DL_I2C_CONTROLLER_STATUS_IDLE) == 0) {
        if (--timeout == 0) break;
    }

    return data;
}

/* ========== 解析传感器数据 ========== */
/*
 * 从一个字节中提取 8 个传感器的状态
 * x1 = bit7 (最左), x8 = bit0 (最右)
 * 0 = 检测到黑线, 1 = 白底
 */
void deal_IRdata(u8 *x1, u8 *x2, u8 *x3, u8 *x4,
                 u8 *x5, u8 *x6, u8 *x7, u8 *x8)
{
    u8 IRbuf = 0xFF;

    IRbuf = IRI2C_ReadByte(IR_I2C_REG_DATA);

    *x1 = (IRbuf >> 7) & 0x01;
    *x2 = (IRbuf >> 6) & 0x01;
    *x3 = (IRbuf >> 5) & 0x01;
    *x4 = (IRbuf >> 4) & 0x01;
    *x5 = (IRbuf >> 3) & 0x01;
    *x6 = (IRbuf >> 2) & 0x01;
    *x7 = (IRbuf >> 1) & 0x01;
    *x8 = (IRbuf >> 0) & 0x01;
}

/* ========== 位置式 PID 计算 ========== */
/*
 * 输入: 偏差值 (actual_value)
 * 输出: 转向修正量
 *
 * 调试建议:
 *   如果巡线效果不好, 先将 KI、KD 置 0
 *   然后慢慢增加 KP, 最后再尝试加 KD
 */
float PID_IR_Calc(int8_t actual_value)
{
    float IRTrackTurn = 0;
    int8_t error;

    error = actual_value;

    IRTrack_Integral += error;

    /* 积分限幅 (防止积分饱和) */
    if (IRTrack_Integral >  5000)  IRTrack_Integral =  5000;
    if (IRTrack_Integral < -5000)  IRTrack_Integral = -5000;

    /* 位置式 PID */
    IRTrackTurn = error * IRTrack_Trun_KP
                + IRTrack_Trun_KI * IRTrack_Integral
                + (error - error_last) * IRTrack_Trun_KD;

    error_last = error;

    return IRTrackTurn;
}

/* ========== 巡线主逻辑 ========== */
/*
 * 巡线策略:
 *   1. 首先判断是否为急转弯 (锐角/直角), 用硬编码模式匹配
 *   2. 一般情况用质心法计算偏差
 *   3. 丢线 (全白) 或 十字路口 (全黑) 保持上一个状态
 *
 * 传感器位置 (从车头看, X1 在最左, X8 在最右):
 *   X1  X2  X3  X4  |  X5  X6  X7  X8
 *   ← 左侧         中心        右侧 →
 *
 * 偏差符号约定:
 *   正 (+) → 线在右侧 → 车需右转 (左轮加速, 右轮减速)
 *   负 (-) → 线在左侧 → 车需左转 (右轮加速, 左轮减速)
 */
void LineWalking(void)
{
    static int8_t err = 0;
    static u8 x1, x2, x3, x4, x5, x6, x7, x8;

    /* 1. 读取传感器 */
    deal_IRdata(&x1, &x2, &x3, &x4, &x5, &x6, &x7, &x8);

    /* 2. 急转弯优先判断 (直角/锐角) */
    /*    线在最左侧 5 个传感器, 车偏右太多, 需急左转 */
    if      (x1 == 0 && x2 == 0 && x3 == 0 && x4 == 0
          && x5 == 0 && x6 == 1 && x7 == 1 && x8 == 1)   /* 0000 0111 */
    {
        err = -15;
        delay_ms(100);
    }
    /*    线在最右侧 4 个传感器, 车偏左太多, 需急右转 */
    else if (x1 == 1 && x2 == 1 && x3 == 1 && x4 == 0
          && x5 == 0 && x6 == 0 && x7 == 0 && x8 == 0)   /* 1110 0000 */
    {
        err = 15;
        delay_ms(100);
    }
    /*    线在左侧 6 个传感器 */
    else if (x1 == 0 && x2 == 0 && x3 == 0 && x4 == 0
          && x5 == 0 && x6 == 0 && x7 == 1 && x8 == 1)   /* 0000 0011 */
    {
        err = -12;
    }
    /*    线在右侧 5 个传感器 */
    else if (x1 == 1 && x2 == 1 && x3 == 0 && x4 == 0
          && x5 == 0 && x6 == 0 && x7 == 0 && x8 == 0)   /* 1100 0000 */
    {
        err = 12;
    }
    /* 3. 剩余情况: 质心法计算偏差 */
    else
    {
        int   count = 0;
        float sum   = 0;
        u8    sensors[8] = {x1, x2, x3, x4, x5, x6, x7, x8};

        for (int i = 0; i < 8; i++)
        {
            if (sensors[i] == 0)    /* 检测到黑线 */
            {
                count++;
                sum += (float)i;    /* 位置权重: 0 ~ 7 */
            }
        }

        if (count > 0 && count < 8)
        {
            /*
             * 传感器中心位置 = 3.5 (索引 3 和 4 之间)
             * 质心偏离中心 = (sum / count) - 3.5
             * 乘以 5 放大到合适的偏差范围
             */
            float centroid = sum / (float)count;
            err = (int8_t)((centroid - 3.5f) * 5.0f);
        }
        /* count == 0: 丢线 → 保持上一个 err */
        /* count == 8: 十字路口 → 保持上一个 err (直行穿过) */
    }

    /* 4. 偏差限幅 */
    if (err >  20)  err =  20;
    if (err < -20)  err = -20;

    /* 5. PID 计算 + 控制电机 */
    pid_output_IRR = (int)(PID_IR_Calc(err));

    Motion_Car_Control(IRR_SPEED, 0, pid_output_IRR);
}
