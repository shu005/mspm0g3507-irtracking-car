/*
 * app_irtracking.h - 八路巡线模块接口
 * 8-channel IR tracking module interface
 */

#ifndef APP_IRTRACKING_H
#define APP_IRTRACKING_H

#include "app_common.h"

/* ========== 巡线 PID 参数 (可调) ========== */
#define IRTrack_Trun_KP   (280)
#define IRTrack_Trun_KI   (0)
#define IRTrack_Trun_KD   (150)

/* 巡线速度 (0~1000) */
#define IRR_SPEED          300

/* ========== IMU 角速度阻尼参数 ========== */
/* 1 = 启用；0 = 保留纯红外循迹。 */
#define IMU_ASSIST_ENABLE             1

/* 当前 Motion_Car_Control 中正 Vz 表示右转。
 * 常见安装方式为 IMU 平放、Z 轴朝上，此时右转的原始 gyro Z 通常为负，
 * 因此默认乘 -1 将其转换为“右转为正”。
 * 若实车加 IMU 后摆动反而增大，把 -1.0f 改成 +1.0f。 */
#define IMU_GYRO_Z_SIGN              (-1.0f)

/* 小偏差/出弯阶段使用较强阻尼，大偏差入弯阶段使用较弱阻尼。 */
#define IMU_DAMP_GAIN_CENTER           5.0f
#define IMU_DAMP_GAIN_TURN             2.0f
#define IMU_DAMP_FULL_GAIN_ERROR       6
#define IMU_DAMP_OUTPUT_LIMIT          800

/* ========== I2C 配置 ========== */
#define IR_I2C_DEVICE_ADDR  0x12
#define IR_I2C_REG_DATA     0x30

uint8_t IRI2C_ReadByte(uint8_t addr);

void deal_IRdata(u8 *x1, u8 *x2, u8 *x3, u8 *x4,
                 u8 *x5, u8 *x6, u8 *x7, u8 *x8);

int32_t PID_IR_Calc(int8_t actual_value);

void LineWalking(void);

#endif /* APP_IRTRACKING_H */