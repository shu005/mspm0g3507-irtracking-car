/*
 * empty_mspm0g3507.c
 *
 * H task 2:
 *   Power on and stay still
 *   -> K1 starts the car and timer
 *   -> complete one clockwise lap
 *   -> detect A marker, stop car and freeze final time
 */

#include "app_common.h"
#include "app_motor.h"
#include "app_motor_usart.h"
#include "app_irtracking.h"
#include "app_imu.h"
#include "app_task2.h"
#include "app_oled.h"
#include "app_task2_display.h"
#include "app_timebase.h"

#define CONTROL_LOOP_MS              IR_CONTROL_LOOP_MS
#define MOTOR_DRIVER_BOOT_DELAY_MS   1500U

volatile uint8_t g_startup_stage = 0U;
volatile uint8_t g_display_online = 0U;

static void App_AdvanceTaskTime(uint32_t elapsed_ms)
{
    /*
     * Both existing Tick interfaces use uint16_t. Splitting also makes the
     * code safe if execution was paused in the debugger for over 65 seconds.
     */
    while (elapsed_ms > 0U)
    {
        uint16_t chunk = (elapsed_ms > 60000U) ?
                         60000U : (uint16_t)elapsed_ms;

        IMU_Tick(chunk);
        IR_Tick(chunk);
        Task2_Tick(chunk);
        elapsed_ms -= chunk;
    }
}

int main(void)
{
    uint32_t last_loop_ms;

    SYSCFG_DL_init();
    App_TimebaseInit();
    g_startup_stage = 1U;

    USART_Init();
    g_startup_stage = 2U;

    delay_ms(MOTOR_DRIVER_BOOT_DELAY_MS);
    Contrl_Speed(0, 0, 0, 0);
    g_startup_stage = 3U;

    IMU_Init();
    IMU_StartGyroCalibration(40U);

    IR_Init();
    Task2_Init();
    g_startup_stage = 4U;

    /*
     * OLED failure is non-fatal: the car remains testable and
     * g_display_online stays 0 for diagnosis.
     */
    g_display_online = Task2_DisplayInit() ? 1U : 0U;
    g_startup_stage = 5U;

    last_loop_ms = App_Millis();

    while (1)
    {
        uint32_t now_ms = App_Millis();
        uint32_t elapsed_ms = now_ms - last_loop_ms;

        last_loop_ms = now_ms;
        App_AdvanceTaskTime(elapsed_ms);

        IMU_Process();
        IR_Process();
        Task2_Process();
        Task2_DisplayProcess();

        /*
         * This is only the minimum control-loop spacing.
         * Actual lap time comes from SysTick, so OLED I2C transfer time is
         * included automatically and cannot make the displayed time slow.
         */
        delay_ms(CONTROL_LOOP_MS);
    }
}

