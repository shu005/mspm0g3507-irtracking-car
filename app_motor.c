/*
 * app_motor.c - 电机控制实现
 *
 * 电机对应关系：
 * M1 -> 左前轮
 * M2 -> 左后轮
 * M3 -> 右前轮
 * M4 -> 右后轮
 */

#include "app_motor.h"
#include "app_motor_usart.h"

/* ========== 四轮目标速度 ========== */

static int16_t speed_L1_setup = 0;
static int16_t speed_L2_setup = 0;
static int16_t speed_R1_setup = 0;
static int16_t speed_R2_setup = 0;

/* ========== 轮距参数 ========== */

#define ROBOT_APB_DEFAULT 150.0f

/*
 * 四轮物理速度校准：
 *
 * 实测前进1米向左偏约10cm，
 * 说明右侧实际速度略快。
 *
 * 左侧保持100%，右侧降低到97%。
 * 使用整数比例，避免增加浮点运算。
 */
#define MOTOR_LEFT_TRIM_NUM      100
#define MOTOR_RIGHT_TRIM_NUM      97
#define MOTOR_TRIM_DEN           100

/* ========== 电机参数配置 ========== */

void Set_Motor(int MOTOR_TYPE)
{
    if (MOTOR_TYPE == 1)
    {
        /* 520电机 */
        send_motor_type(1);
        delay_ms(100);

        send_pulse_phase(30);
        delay_ms(100);

        send_pulse_line(11);
        delay_ms(100);

        send_wheel_diameter(67.00);
        delay_ms(100);

        send_motor_deadzone(1900);
        delay_ms(100);
    }
    else if (MOTOR_TYPE == 2)
    {
        /* 310电机 */
        send_motor_type(2);
        delay_ms(100);

        send_pulse_phase(20);
        delay_ms(100);

        send_pulse_line(13);
        delay_ms(100);

        send_wheel_diameter(48.00);
        delay_ms(100);

        send_motor_deadzone(1600);
        delay_ms(100);
    }
    else if (MOTOR_TYPE == 3)
    {
        /* 测速码盘TT电机 */
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
    else if (MOTOR_TYPE == 4)
    {
        /* TT直流减速电机 */
        send_motor_type(4);
        delay_ms(100);

        send_pulse_phase(48);
        delay_ms(100);

        send_motor_deadzone(1000);
        delay_ms(100);
    }
    else if (MOTOR_TYPE == 5)
    {
        /* L型520电机 */
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
 * V_x：前进速度
 * V_y：横向速度，当前不用
 * V_z：旋转速度
 *
 * 正V_z：右转
 * 负V_z：左转
 */
void Motion_Car_Control(int16_t V_x,
                        int16_t V_y,
                        int16_t V_z)
{
    float robot_APB;
    float speed_fb;
    float speed_spin;

    robot_APB = Motion_Get_APB();

    (void)V_y;

    speed_fb = (float)V_x;

    speed_spin =
        ((float)V_z / 1000.0f) * robot_APB;

    /* 全零指令，停车 */
    if (V_x == 0 &&
        V_y == 0 &&
        V_z == 0)
    {
        Contrl_Speed(0, 0, 0, 0);
        return;
    }

    /* 左右差速计算 */
    speed_L1_setup =
        (int16_t)(speed_fb + speed_spin);

    speed_L2_setup =
        (int16_t)(speed_fb + speed_spin);

    speed_R1_setup =
        (int16_t)(speed_fb - speed_spin);

    speed_R2_setup =
        (int16_t)(speed_fb - speed_spin);

    /* 左前轮限幅 */
    if (speed_L1_setup > 1000)
    {
        speed_L1_setup = 1000;
    }

    if (speed_L1_setup < -1000)
    {
        speed_L1_setup = -1000;
    }

    /* 左后轮限幅 */
    if (speed_L2_setup > 1000)
    {
        speed_L2_setup = 1000;
    }

    if (speed_L2_setup < -1000)
    {
        speed_L2_setup = -1000;
    }

    /* 右前轮限幅 */
    if (speed_R1_setup > 1000)
    {
        speed_R1_setup = 1000;
    }

    if (speed_R1_setup < -1000)
    {
        speed_R1_setup = -1000;
    }

    /* 右后轮限幅 */
    if (speed_R2_setup > 1000)
    {
        speed_R2_setup = 1000;
    }

    if (speed_R2_setup < -1000)
    {
        speed_R2_setup = -1000;
    }

    Contrl_Speed(
        speed_L1_setup,
        speed_L2_setup,
        speed_R1_setup,
        speed_R2_setup
    );
}

/* ========== 四轮底层速度控制 ========== */

/*
 * 参数顺序：
 * 左前、左后、右前、右后。
 *
 * 校准放在这个最底层函数中，
 * 因此上电自检、普通巡线和直角弯都会生效。
 */
void Contrl_Speed(int16_t speed_L1,
                  int16_t speed_L2,
                  int16_t speed_R1,
                  int16_t speed_R2)
{
    char cmd[64];
    int len;

    /*
     * 使用int32_t作为乘法中间值，
     * 防止整数乘法发生溢出。
     */
    speed_L1 = (int16_t)(
        ((int32_t)speed_L1 *
         MOTOR_LEFT_TRIM_NUM) /
        MOTOR_TRIM_DEN
    );

    speed_L2 = (int16_t)(
        ((int32_t)speed_L2 *
         MOTOR_LEFT_TRIM_NUM) /
        MOTOR_TRIM_DEN
    );

    speed_R1 = (int16_t)(
        ((int32_t)speed_R1 *
         MOTOR_RIGHT_TRIM_NUM) /
        MOTOR_TRIM_DEN
    );

    speed_R2 = (int16_t)(
        ((int32_t)speed_R2 *
         MOTOR_RIGHT_TRIM_NUM) /
        MOTOR_TRIM_DEN
    );

    len = snprintf(
        cmd,
        sizeof(cmd),
        "$spd:%d,%d,%d,%d#",
        speed_L1,
        speed_L2,
        speed_R1,
        speed_R2
    );

    if (len > 0 &&
        len < (int)sizeof(cmd))
    {
        Send_Motor_ArrayU8(
            (const uint8_t *)cmd,
            (uint16_t)len
        );
    }
}

/* ========== 设置电机PID ========== */

void send_motor_PID(float Kp,
                    float Ki,
                    float Kd)
{
    char cmd[64];
    int len;

    len = snprintf(
        cmd,
        sizeof(cmd),
        "$MPID:%.2f,%.2f,%.2f#",
        Kp,
        Ki,
        Kd
    );

    if (len > 0 &&
        len < (int)sizeof(cmd))
    {
        Send_Motor_ArrayU8(
            (const uint8_t *)cmd,
            (uint16_t)len
        );
    }
}