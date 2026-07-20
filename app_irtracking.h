/*
 * app_irtracking.h - 八路巡线模块接口
 * 8-channel IR tracking module interface
 */

#ifndef APP_IRTRACKING_H
#define APP_IRTRACKING_H

#include "app_common.h"

/* ========== 巡线 PID 参数 (可调) ========== */
/* Line tracking PID parameters (adjustable) */
#define IRTrack_Trun_KP   (500)   /* 比例系数 Proportional */
#define IRTrack_Trun_KI   (0)     /* 积分系数 Integral     */
#define IRTrack_Trun_KD   (0)     /* 微分系数 Derivative   */

/* 巡线速度 (0~1000) */
#define IRR_SPEED          300

/* ========== I2C 配置 ========== */
/* IR模块设备地址 0x12, 传感器数据在寄存器 0x30
   接线: SCL -> PA15, SDA -> PA16 */
#define IR_I2C_DEVICE_ADDR  0x12
#define IR_I2C_REG_DATA     0x30

/* ========== 函数声明 ========== */

/* 从 IR 模块读一字节 (8路传感器状态)
   bit7=X1(最左), bit0=X8(最右), 0=检测到黑线, 1=白底 */
uint8_t IRI2C_ReadByte(uint8_t addr);

/* 解析 8 路传感器数据 */
void deal_IRdata(u8 *x1, u8 *x2, u8 *x3, u8 *x4,
                 u8 *x5, u8 *x6, u8 *x7, u8 *x8);

/* 位置式 PID 计算 (转向控制) */
float PID_IR_Calc(int8_t actual_value);

/* 巡线主逻辑 */
void LineWalking(void);

#endif /* APP_IRTRACKING_H */