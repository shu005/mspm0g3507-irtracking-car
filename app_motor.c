
/*
 * app_motor.c - 电机控制实现
 * Motor control implementation
 *
 * 电机接口对应关系:
 *   M1 -> 左上电机 (左前轮)
 *   M2 -> 左下电机 (左后轮)
 *   M3 -> 右上电机 (右前轮)
 *   M4 -> 右下电机 (右后轮)
 *
 * 运动学:
 *   差速转向: 左轮 = 前进 + 旋转, 右轮 = 前进 - 旋转
 *   speed_spin = (V_z / 1000) * APB
 *   APB (Axle Per Base): 半轮距参数, 需根据底盘实测调整
 */

#include "app_motor.h"
#include "app_motor_usart.h"

/* ========== 全局变量 ========== */
static int16_t speed_L1_setup = 0;   /* 左前轮目标速度 */
static int16_t speed_L2_setup = 0;   /* 左后轮目标速度 */
static int16_t speed_R1_setup = 0;   /* 右前轮目标速度 */
static int16_t speed_R2_setup = 0;   /* 右后轮目标速度 */

/* ========== 轮距参数 ========== */
/*
 * APB (Axle Per Base) 用于将旋转指令换算为左右轮差速
 * 默认值 150.0, 如果转弯过猛则减小, 转弯不够则增大
 * 四驱310底盘建议范围: 100 ~ 200
 */
#define ROBOT_APB_DEFAULT   150.0f

/* ========== 电机参数配置 ========== */
/*
 * 一键配置电机参数
 * 使用本店电机时不要修改此函数, 用空页的 MOTOR_TYPE 切换
 * 使用自己的电机时, 任选一个 MOTOR_TYPE 改参数
 */
void Set_Motor(int MOTOR_TYPE)
{
    if (MOTOR_TYPE == 1)         /* 520电机 */
    {
        send_motor_type(1);
        delay_ms(100);
        send_pulse_phase(30);     /* 减速比 */
        delay_ms(100);
        send_pulse_line(11);      /* 磁环线数 */
        delay_ms(100);
        send_wheel_diameter(67.00); /* 轮子直径 mm */
        delay_ms(100);
        send_motor_deadzone(1900);  /* 死区 */
        delay_ms(100);
    }
    else if (MOTOR_TYPE == 2)    /* 310电机 (你的) */
    {
        send_motor_type(2);
        delay_ms(100);
        send_pulse_phase(20);      /* 减速比 1:20 */
        delay_ms(100);
        send_pulse_line(13);       /* 磁环: 13 线 */
        delay_ms(100);
        send_wheel_diameter(48.00); /* 轮径: 48mm */
        delay_ms(100);
        send_motor_deadzone(1600);  /* 死区: 1600 */
        delay_ms(100);
    }
    else if (MOTOR_TYPE == 3)    /* 测速码盘TT电机 */
    {
        send_motor_type(3);
        delay_ms(100);
        send_pulse_phase(45);
        delay_ms(100);
        send_pulse_line(13);
        delay_ms(100);
        send_wheel_diameter(68.00);
        delay_ms(100);
        send_motor_deadzone(1600);
        delay_ms(100);
    }
    else if (MOTOR_TYPE == 4)    /* TT直流减速电机 */
    {
        send_motor_type(4);
        delay_ms(100);
        send_pulse_phase(48);
        delay_ms(100);
        send_motor_deadzone(1000);
        delay_ms(100);
    }
    else if (MOTOR_TYPE == 5)    /* L型520电机 */
    {
        send_motor_type(1);
        delay_ms(100);
        send_pulse_phase(40);
        delay_ms(100);
        send_pulse_line(11);
        delay_ms(100);
        send_wheel_diameter(67.00);
        delay_ms(100);
        send_motor_deadzone(1900);
        delay_ms(100);
    }
}

/* ========== 获取轮距参数 ========== */
float Motion_Get_APB(void)
{
    return ROBOT_APB_DEFAULT;
}

/* ========== 小车运动控制 ========== */
/*
 * V_x: 前进速度 (0 ~ 1000)
 * V_y: 横向速度 (预留, 填 0)
 * V_z: 旋转速度 (-1000 ~ 1000)
 *      正 = 右转 (左轮加速, 右轮减速)
 *      负 = 左转 (左轮减速, 右轮加速)
 *
 * 差速公式:
 *   左轮速度 = V_x + (V_z / 1000) * APB
 *   右轮速度 = V_x - (V_z / 1000) * APB
 */
void Motion_Car_Control(int16_t V_x, int16_t V_y, int16_t V_z)
{
    float robot_APB = Motion_Get_APB();
    float speed_fb;      /* 前进速度 */
    float speed_spin;    /* 旋转差速 */

    (void)V_y;           /* 暂不使用横向速度 */

    speed_fb   = (float)V_x;
    speed_spin = ((float)V_z / 1000.0f) * robot_APB;

    /* 全零指令 → 停车 */
    if (V_x == 0 && V_y == 0 && V_z == 0)
    {
        Contrl_Speed(0, 0, 0, 0);
        return;
    }

    /* 差速计算 */
    speed_L1_setup = (int16_t)(speed_fb + speed_spin);
    speed_L2_setup = (int16_t)(speed_fb + speed_spin);
    speed_R1_setup = (int16_t)(speed_fb - speed_spin);
    speed_R2_setup = (int16_t)(speed_fb - speed_spin);

    /* 速度限幅 */
    if (speed_L1_setup >  1000) speed_L1_setup =  1000;
    if (speed_L1_setup < -1000) speed_L1_setup = -1000;
    if (speed_L2_setup >  1000) speed_L2_setup =  1000;
    if (speed_L2_setup < -1000) speed_L2_setup = -1000;
    if (speed_R1_setup >  1000) speed_R1_setup =  1000;
    if (speed_R1_setup < -1000) speed_R1_setup = -1000;
    if (speed_R2_setup >  1000) speed_R2_setup =  1000;
    if (speed_R2_setup < -1000) speed_R2_setup = -1000;

    /* 发送速度指令到四路电机驱动板 */
    Contrl_Speed(speed_L1_setup, speed_L2_setup,
                 speed_R1_setup, speed_R2_setup);
}

/* ========== 控制四个电机转速 ========== */
/*
 * 向电机驱动板发送四路速度指令
 * 协议格式: $spd:M1,M2,M3,M4#
 * M1=左前, M2=左后, M3=右前, M4=右后
 */
void Contrl_Speed(int16_t speed_L1, int16_t speed_L2,
                  int16_t speed_R1, int16_t speed_R2)
{
    char cmd[64];
    int len;

    len = snprintf(cmd, sizeof(cmd),
                   "$spd:%d,%d,%d,%d#",
                   speed_L1, speed_L2, speed_R1, speed_R2);

    if (len > 0 && len < (int)sizeof(cmd))
    {
        Send_Motor_ArrayU8((const uint8_t *)cmd, (uint16_t)len);
    }
}

/* ========== 设置电机 PID ========== */
/*
 * 向电机驱动板发送 PID 参数
 * 这里参数是为四驱310底盘配置的, 其他底盘需自行测试修改
 */
void send_motor_PID(float Kp, float Ki, float Kd)
{
    char cmd[64];
    int len;

    len = snprintf(cmd, sizeof(cmd),
                   "$motorpid:%.1f,%.1f,%.1f#", Kp, Ki, Kd);

    if (len > 0 && len < (int)sizeof(cmd))
    {
        Send_Motor_ArrayU8((const uint8_t *)cmd, (uint16_t)len);
    }
}
