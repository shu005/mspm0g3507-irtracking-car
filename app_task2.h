/*
 * app_task2.h - H题要求2：按键启动，顺时针一圈，回到A点停车
 *
 * 启动按键：
 *   PA28 -> 按键 -> GND
 *   PA28由SysConfig配置为输入上拉，按下为低电平。
 */

#ifndef APP_TASK2_H
#define APP_TASK2_H

#include "app_common.h"
#include <stdbool.h>

typedef enum
{
    TASK2_STATE_WAIT_KEY = 0,
    TASK2_STATE_LEAVING_A,
    TASK2_STATE_RUNNING,
    TASK2_STATE_FINISHED
} Task2_State_t;

typedef enum
{
    TASK2_STOP_NONE = 0,
    TASK2_STOP_A_MARKER,
    TASK2_STOP_TIMEOUT
} Task2_StopReason_t;

/* 可在CCS Expressions/Watch中直接观察。 */
extern volatile Task2_State_t g_task2_state;
extern volatile Task2_StopReason_t g_task2_stop_reason;
extern volatile uint32_t g_task2_elapsed_ms;
extern volatile uint32_t g_task2_finish_ms;
extern volatile float g_task2_right_turn_deg;
extern volatile uint8_t g_task2_marker_now;
extern volatile uint8_t g_task2_marker_hits;
extern volatile uint8_t g_task2_approach_mode;

void Task2_Init(void);
void Task2_Process(void);
void Task2_Tick(uint16_t elapsed_ms);
bool Task2_IsFinished(void);

#endif /* APP_TASK2_H */