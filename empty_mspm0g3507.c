/*
 * empty_mspm0g3507.c - 诊断第3步: LineWalking 循环调用 + 延时
 *
 * 测试: 在循环里调用 LineWalking, 每次间隔 50ms
 * 如果动了 → 之前是 UART 被指令淹没了
 * 如果不动 → 问题在 LineWalking 内部逻辑
 */

#include "app_common.h"
#include "app_motor.h"
#include "app_motor_usart.h"
#include "app_irtracking.h"

int main(void)
{
    SYSCFG_DL_init();
    USART_Init();

    delay_ms(3000);

    Contrl_Speed(0, 0, 0, 0);
    delay_ms(500);

    /* 电机自检 */
    Contrl_Speed(200, 200, 200, 200);
    delay_ms(2000);
    Contrl_Speed(300, 300, -200, -200);
    delay_ms(1000);
    Contrl_Speed(0, 0, 0, 0);
    delay_ms(1000);

    /* 循环巡线, 每次间隔 50ms */
    while (1)
    {
        LineWalking();
        delay_ms(50);
    }
}
