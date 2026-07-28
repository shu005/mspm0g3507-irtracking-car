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

/* 电机自检配置 */
#define MOTOR_SELF_TEST_ENABLE       1U
#define MOTOR_SELF_TEST_SPEED        350
#define MOTOR_DRIVER_BOOT_DELAY_MS   1500U

/*
 * CCS Expressions中可观察：
 * 0 = 尚未进入main
 * 1 = SysConfig初始化完成
 * 2 = 电机UART初始化完成
 * 3 = 正在执行电机自检
 * 4 = 电机自检完成
 * 5 = IMU/K230初始化完成
 * 6 = IMU校准阶段结束并进入主循环
 */
volatile uint8_t g_startup_stage = 0U;

/*
 * 可以在CCS Expressions中直接修改g_app_mode：
 *   0：停车监视
 *   1：红外循迹
 *   2：钢球跟踪
 */
volatile uint8_t g_app_mode = APP_MODE_LINE_TRACKING;

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

/*
 * 自检必须放在IMU/K230/循迹初始化之前。
 * 这样即使后续模块或传感器接线有问题，也不会影响电机链路验证。
 */
static void App_MotorSelfTest(void)
{
    delay_ms(MOTOR_DRIVER_BOOT_DELAY_MS);

    Contrl_Speed(0, 0, 0, 0);
    delay_ms(100U);

    Contrl_Speed(MOTOR_SELF_TEST_SPEED,
                 MOTOR_SELF_TEST_SPEED,
                 MOTOR_SELF_TEST_SPEED,
                 MOTOR_SELF_TEST_SPEED);
    delay_ms(1500U);

    Contrl_Speed(0, 0, 0, 0);
    delay_ms(500U);

    Contrl_Speed(-MOTOR_SELF_TEST_SPEED,
                 -MOTOR_SELF_TEST_SPEED,
                 -MOTOR_SELF_TEST_SPEED,
                 -MOTOR_SELF_TEST_SPEED);
    delay_ms(1000U);

    Contrl_Speed(0, 0, 0, 0);
    delay_ms(500U);
}

int main(void)
{
    uint32_t calibration_elapsed_ms = 0U;

    /* SysConfig初始化UART和GPIO引脚。 */
    SYSCFG_DL_init();
    g_startup_stage = 1U;

    /* 电机驱动板UART必须最先初始化。 */
    USART_Init();
    g_startup_stage = 2U;

#if MOTOR_SELF_TEST_ENABLE
    g_startup_stage = 3U;
    App_MotorSelfTest();
    g_startup_stage = 4U;
#endif

    /* 自检完成后再启动其他通信和控制模块。 */
    IMU_Init();
    K230_Init();
    BallControl_Init();
    g_startup_stage = 5U;

    /* IMU校准期间小车必须静止。 */
    Contrl_Speed(0, 0, 0, 0);

    IMU_StartGyroCalibration(IMU_CALIBRATION_SAMPLES);

    while (!IMU_IsCalibrated() &&
           calibration_elapsed_ms < IMU_CALIBRATION_TIMEOUT_MS)
    {
        App_DelayWithServices(5U);
        calibration_elapsed_ms += 5U;
    }

    Contrl_Speed(0, 0, 0, 0);
    App_DelayWithServices(500U);
    g_startup_stage = 6U;

    while (1)
    {
        IMU_Process();
        K230_Process();

        switch (g_app_mode)
        {
            case APP_MODE_LINE_TRACKING:
                LineWalking();
                break;

            case APP_MODE_BALL_TRACKING:
                BallControl_Process();
                break;

            case APP_MODE_MONITOR:
            default:
                Contrl_Speed(0, 0, 0, 0);
                break;
        }

        delay_ms(CONTROL_LOOP_MS);

        IMU_Tick(CONTROL_LOOP_MS);
        K230_Tick(CONTROL_LOOP_MS);
    }
}