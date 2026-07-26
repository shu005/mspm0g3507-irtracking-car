/*
 * empty_mspm0g3507.c - IR tracking with IMU yaw-rate assistance
 */

#include "app_common.h"
#include "app_motor.h"
#include "app_motor_usart.h"
#include "app_irtracking.h"
#include "app_imu.h"

/*
 * 与 app_irtracking.c 的非阻塞状态机共用同一个周期定义。
 * 不要再单独写 40U，否则状态机时间和真实调用周期会不一致。
 */
#define CONTROL_LOOP_MS              IR_CONTROL_LOOP_MS
#define IMU_CALIBRATION_SAMPLES      40U
#define IMU_CALIBRATION_TIMEOUT_MS   9000U

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

    SYSCFG_DL_init();
    USART_Init();
    IMU_Init();

    /* Keep the chassis completely stationary during this stage. */
    Contrl_Speed(0, 0, 0, 0);
    IMU_StartGyroCalibration(IMU_CALIBRATION_SAMPLES);

    /* The supplied module can require about 5 s to start. This loop keeps
     * parsing UART data and then collects stationary gyro-Z samples.
     * If the IMU is absent, the timeout expires and tracking falls back
     * automatically to the original pure-IR controller. */
    while (!IMU_IsCalibrated() &&
           calibration_elapsed_ms < IMU_CALIBRATION_TIMEOUT_MS)
    {
        App_DelayWithIMU(5U);
        calibration_elapsed_ms += 5U;
    }

    Contrl_Speed(0, 0, 0, 0);
    App_DelayWithIMU(500U);

    /* Preserve the existing motor self-test. */
    Contrl_Speed(200, 200, 200, 200);
    App_DelayWithIMU(2000U);

    Contrl_Speed(-200, -200, -200, -200);
    App_DelayWithIMU(1000U);

    Contrl_Speed(0, 0, 0, 0);
    App_DelayWithIMU(1000U);

    /*
     * IMU 默认 25 Hz 输出并不要求控制环也只能运行在 25 Hz。
     * UART 中断会缓存字节；10 ms 调用一次解析和红外状态机即可。
     */
    while (1)
    {
        LineWalking();
        delay_ms(CONTROL_LOOP_MS);
        IMU_Tick(CONTROL_LOOP_MS);
    }
}