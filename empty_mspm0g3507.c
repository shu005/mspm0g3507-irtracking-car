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
#include "app_irtracking.h"
#include "app_task2.h"

#define CONTROL_LOOP_MS              IR_CONTROL_LOOP_MS
#define MOTOR_DRIVER_BOOT_DELAY_MS   1500U

/*
 * CCS Expressions中可观察：
 * 0 = 尚未进入main
 * 1 = SysConfig初始化完成
 * 2 = 电机UART初始化完成
 * 3 = 电机驱动板启动等待完成，已发送停车指令
 * 4 = 要求2状态机初始化完成
 * 5 = 已进入主循环，等待启动按键
 */
volatile uint8_t g_startup_stage = 0U;

int main(void)
{
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

    /*
     * 要求2基础验证版暂时完全不初始化IMU。
     * 已验证K1和电机链路正常后，先让纯灰度循迹状态机跑通。
     */
    Task2_Init();
    g_startup_stage = 4U;
    delay_ms(100U);
    g_startup_stage = 5U;

    while (1)
    {
        Task2_Process();

        delay_ms(CONTROL_LOOP_MS);

        Task2_Tick(CONTROL_LOOP_MS);
    }
}