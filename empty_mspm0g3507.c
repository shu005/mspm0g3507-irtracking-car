/*
 * empty_mspm0g3507.c
 *
 * 支持：
 *   1. K230数据监视
 *   2. 红外+IMU循迹
 *   3. K230钢球跟踪
 */

#include "app_common.h"
#include "app_motor.h"
#include "app_motor_usart.h"
#include "app_irtracking.h"
#include "app_imu.h"
#include "app_k230.h"
#include "app_ball_control.h"

#define CONTROL_LOOP_MS              IR_CONTROL_LOOP_MS
#define IMU_CALIBRATION_SAMPLES      40U
#define IMU_CALIBRATION_TIMEOUT_MS   9000U

/* 系统运行模式 */
#define APP_MODE_MONITOR             0U
#define APP_MODE_LINE_TRACKING       1U
#define APP_MODE_BALL_TRACKING       2U

/*
 * 初始使用监视模式：
 * 电机保持停止，只接收K230数据。
 *
 * 可以在CCS Expressions中直接修改g_app_mode：
 *   0：停车监视
 *   1：红外循迹
 *   2：钢球跟踪
 */
 volatile uint8_t g_app_mode = APP_MODE_LINE_TRACKING;

/*
 * 接入K230期间先关闭电机自检，避免上电突然运动。
 * 全部调通后可改为1。
 */
#define MOTOR_SELF_TEST_ENABLE       1U

static void App_DelayWithServices(uint32_t delay_time_ms)
{
    while (delay_time_ms >= 5U)
    {
        delay_ms(5U);

        IMU_Tick(5U);
        K230_Tick(5U);

        IMU_Process();
        K230_Process();

        delay_time_ms -= 5U;
    }

    if (delay_time_ms > 0U)
    {
        delay_ms(delay_time_ms);

        IMU_Tick((uint16_t)delay_time_ms);
        K230_Tick((uint16_t)delay_time_ms);

        IMU_Process();
        K230_Process();
    }
}

int main(void)
{
    uint32_t calibration_elapsed_ms = 0U;

    /* SysConfig初始化UART、I2C和引脚 */
    SYSCFG_DL_init();

    USART_Init();
    IMU_Init();
    K230_Init();
    BallControl_Init();

    /* IMU校准期间小车必须静止 */
    Contrl_Speed(0, 0, 0, 0);

    IMU_StartGyroCalibration(
        IMU_CALIBRATION_SAMPLES
    );

    while (!IMU_IsCalibrated() &&
           calibration_elapsed_ms <
           IMU_CALIBRATION_TIMEOUT_MS)
    {
        App_DelayWithServices(5U);
        calibration_elapsed_ms += 5U;
    }

    Contrl_Speed(0, 0, 0, 0);
    App_DelayWithServices(500U);

#if MOTOR_SELF_TEST_ENABLE

    Contrl_Speed(200, 200, 200, 200);
    App_DelayWithServices(2000U);

    Contrl_Speed(-200, -200, -200, -200);
    App_DelayWithServices(1000U);

    Contrl_Speed(0, 0, 0, 0);
    App_DelayWithServices(1000U);

#endif

    while (1)
    {
        /*
         * 先解析本周期收到的通信数据，
         * 再执行对应控制模块。
         */
        IMU_Process();
        K230_Process();

        switch (g_app_mode)
        {
            case APP_MODE_LINE_TRACKING:

                /*
                 * 只有循迹模块可以在该模式下发送电机指令。
                 */
                LineWalking();
                break;

            case APP_MODE_BALL_TRACKING:

                /*
                 * 只有钢球控制模块可以在该模式下发送电机指令。
                 */
                BallControl_Process();
                break;

            case APP_MODE_MONITOR:
            default:

                /*
                 * 通信测试模式：
                 * K230正常接收，但小车绝不运动。
                 */
                Contrl_Speed(0, 0, 0, 0);
                break;
        }

        delay_ms(CONTROL_LOOP_MS);

        IMU_Tick(CONTROL_LOOP_MS);
        K230_Tick(CONTROL_LOOP_MS);
    }
}