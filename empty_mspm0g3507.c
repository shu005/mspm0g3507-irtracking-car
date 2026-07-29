/*
 * empty_mspm0g3507.c
 *
 * H题要求2专用主程序：
 *   上电静止 -> 按键启动 -> 顺时针一圈 -> 回到A点停车
 *
 * 当前版本暂不接OLED，也不运行K230和滚球控制状态机。
 */

#include "app_common.h"
#include "app_motor.h"
#include "app_motor_usart.h"
#include "app_imu.h"
#include "app_irtracking.h"
#include "app_task2.h"

#define CONTROL_LOOP_MS              IR_CONTROL_LOOP_MS
#define IMU_CALIBRATION_SAMPLES      40U
#define IMU_CALIBRATION_TIMEOUT_MS   3000U
#define MOTOR_DRIVER_BOOT_DELAY_MS   1500U

/*
 * CCS Expressions中可观察：
 * 0 = 尚未进入main
 * 1 = SysConfig初始化完成
 * 2 = 电机UART初始化完成
 * 3 = 电机驱动板启动等待完成，已发送停车指令
 * 4 = IMU初始化和校准中
 * 5 = 要求2状态机初始化完成
 * 6 = 已进入主循环，等待启动按键
 */
volatile uint8_t g_startup_stage = 0U;

static void App_DelayWithIMU(uint32_t delay_time_ms)
{
    while (delay_time_ms >= 5U)
    {
        delay_ms(5U);

        IMU_Tick(5U);
        IMU_Process();

        delay_time_ms -= 5U;
    }

    if (delay_time_ms > 0U)
    {
        delay_ms(delay_time_ms);

        IMU_Tick((uint16_t)delay_time_ms);
        IMU_Process();
    }
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

    /*
     * 不做电机前后自检，保证比赛上电后小车不会自行移动。
     * 等待驱动板启动后只发送一次全零速度。
     */
    delay_ms(MOTOR_DRIVER_BOOT_DELAY_MS);
    Contrl_Speed(0, 0, 0, 0);
    g_startup_stage = 3U;

    /* IMU掉线时校准会超时，但不阻塞后续纯红外循迹。 */
    IMU_Init();
    g_startup_stage = 4U;
    IMU_StartGyroCalibration(IMU_CALIBRATION_SAMPLES);

    while (!IMU_IsCalibrated() &&
           calibration_elapsed_ms < IMU_CALIBRATION_TIMEOUT_MS)
    {
        App_DelayWithIMU(5U);
        calibration_elapsed_ms += 5U;
    }

    Contrl_Speed(0, 0, 0, 0);
    Task2_Init();
    g_startup_stage = 5U;

    App_DelayWithIMU(100U);
    g_startup_stage = 6U;

    while (1)
    {
        IMU_Process();
        Task2_Process();

        delay_ms(CONTROL_LOOP_MS);

        IMU_Tick(CONTROL_LOOP_MS);
        Task2_Tick(CONTROL_LOOP_MS);
    }
}