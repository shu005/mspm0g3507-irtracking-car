/*
 * app_motor.h - 电机控制接口
 * Motor control interface
 */

#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include "app_common.h"

/* ========== 电机类型 ========== */
/* 对应 empty.c 开头的 MOTOR_TYPE 宏 */
#define MOTOR_TYPE_520          1   /* 520电机         */
#define MOTOR_TYPE_310          2   /* 310电机 (你的)   */
#define MOTOR_TYPE_TT           3   /* 测速码盘TT电机   */
#define MOTOR_TYPE_TT_DC        4   /* TT直流减速电机   */
#define MOTOR_TYPE_520_L        5   /* L型520电机       */

/* ========== 函数声明 ========== */

/* 设置电机类型 (一键配置电机参数) */
void Set_Motor(int MOTOR_TYPE);

/* 小车运动控制
   V_x: 前进速度 (0~1000)
   V_y: 横向速度 (预留, 填0)
   V_z: 旋转速度 (-1000~1000, 正=右转, 负=左转) */
void Motion_Car_Control(int16_t V_x, int16_t V_y, int16_t V_z);

/* 控制四个电机转速 (底层接口)
   L1,R1,R2,R2: 各电机速度 (-1000~1000)
   L1=左前, L2=左后, R1=右前, R2=右后 */
void Contrl_Speed(int16_t speed_L1, int16_t speed_L2,
                  int16_t speed_R1, int16_t speed_R2);

/* 获取轮距参数 (用于旋转速度换算) */
float Motion_Get_APB(void);

/* 设置电机 PID 参数 */
void send_motor_PID(float Kp, float Ki, float Kd);

#endif /* APP_MOTOR_H */
