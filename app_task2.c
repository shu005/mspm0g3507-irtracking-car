/*
 * app_task2.c - H题要求2任务状态机
 *
 * 流程：
 *   等待按键 -> 驶离初始A线 -> 正常循迹 -> 接近A点减速
 *   -> 连续检测到A点启停线 -> 永久停车
 *
 * A点启停线判据：
 *   至少5路同时检测到黑色，并且传感器左右半区都检测到黑色。
 * 这样既能覆盖5cm启停线，也不会把普通1.8cm环线当作A点。
 */

#include "app_task2.h"
#include "app_irtracking.h"
#include "app_motor.h"
#include "app_imu.h"

#define TASK2_KEY_DEBOUNCE_FRAMES       3U
#define TASK2_LEAVE_CLEAR_FRAMES         6U
#define TASK2_MARKER_CONFIRM_FRAMES      2U
#define TASK2_MARKER_MIN_BLACK           5U

/* 小于该时间时，即使看到宽黑线也不能判为完成一圈。 */
#define TASK2_MIN_LAP_MS             10000U

/* IMU累计右转约300度，或运行13秒后，切换到终点接近速度。 */
#define TASK2_APPROACH_TURN_DEG       300.0f
#define TASK2_APPROACH_FALLBACK_MS  13000U

/* 丢失A点时的安全停车上限。 */
#define TASK2_MAX_RUN_MS            23000U

/* 与app_irtracking.h中的陀螺仪方向约定保持一致。 */
#define TASK2_GYRO_Z_TO_RIGHT_SIGN  IMU_GYRO_Z_SIGN
#define TASK2_GYRO_TURN_DEADBAND_DPS  5.0f

volatile Task2_State_t g_task2_state = TASK2_STATE_WAIT_KEY;
volatile Task2_StopReason_t g_task2_stop_reason = TASK2_STOP_NONE;
volatile uint32_t g_task2_elapsed_ms = 0U;
volatile uint32_t g_task2_finish_ms = 0U;
volatile float g_task2_right_turn_deg = 0.0f;
volatile uint8_t g_task2_marker_now = 0U;
volatile uint8_t g_task2_marker_hits = 0U;
volatile uint8_t g_task2_approach_mode = 0U;

static uint8_t s_leave_clear_frames = 0U;
static bool s_key_last_raw_pressed = false;
static bool s_key_stable_pressed = false;
static uint8_t s_key_same_frames = 0U;

static bool Task2_ReadKeyPressed(void)
{
    return (DL_GPIO_readPins(KEY_GPIO_PORT,
                             KEY_GPIO_START_KEY_PIN) == 0U);
}

static bool Task2_TakeKeyPressedEvent(void)
{
    bool raw_pressed = Task2_ReadKeyPressed();

    if (raw_pressed == s_key_last_raw_pressed)
    {
        if (s_key_same_frames < TASK2_KEY_DEBOUNCE_FRAMES)
        {
            s_key_same_frames++;
        }
    }
    else
    {
        s_key_last_raw_pressed = raw_pressed;
        s_key_same_frames = 1U;
    }

    if ((s_key_same_frames >= TASK2_KEY_DEBOUNCE_FRAMES) &&
        (raw_pressed != s_key_stable_pressed))
    {
        s_key_stable_pressed = raw_pressed;

        if (s_key_stable_pressed)
        {
            return true;
        }
    }

    return false;
}

static bool Task2_IsAMarker(void)
{
    uint8_t mask = g_ir_black_mask;
    bool left_half_has_black = ((mask & 0xF0U) != 0U);
    bool right_half_has_black = ((mask & 0x0FU) != 0U);

    return (g_ir_black_count >= TASK2_MARKER_MIN_BLACK) &&
           left_half_has_black &&
           right_half_has_black;
}

static void Task2_StartRun(void)
{
    g_task2_elapsed_ms = 0U;
    g_task2_finish_ms = 0U;
    g_task2_right_turn_deg = 0.0f;
    g_task2_marker_now = 0U;
    g_task2_marker_hits = 0U;
    g_task2_approach_mode = 0U;
    g_task2_stop_reason = TASK2_STOP_NONE;
    s_leave_clear_frames = 0U;

    IR_ResetController();
    IR_SetBaseSpeed(IR_SPEED_FAST);
    g_task2_state = TASK2_STATE_LEAVING_A;
}

static void Task2_Stop(Task2_StopReason_t reason)
{
    Contrl_Speed(0, 0, 0, 0);
    g_task2_finish_ms = g_task2_elapsed_ms;
    g_task2_stop_reason = reason;
    g_task2_state = TASK2_STATE_FINISHED;
}

static void Task2_UpdateApproachSpeed(void)
{
    if (g_task2_approach_mode != 0U)
    {
        return;
    }

    if ((g_task2_right_turn_deg >= TASK2_APPROACH_TURN_DEG) ||
        (g_task2_elapsed_ms >= TASK2_APPROACH_FALLBACK_MS))
    {
        g_task2_approach_mode = 1U;
        IR_SetBaseSpeed(IR_SPEED_APPROACH);
    }
}

void Task2_Init(void)
{
    g_task2_state = TASK2_STATE_WAIT_KEY;
    g_task2_stop_reason = TASK2_STOP_NONE;
    g_task2_elapsed_ms = 0U;
    g_task2_finish_ms = 0U;
    g_task2_right_turn_deg = 0.0f;
    g_task2_marker_now = 0U;
    g_task2_marker_hits = 0U;
    g_task2_approach_mode = 0U;

    s_leave_clear_frames = 0U;
    s_key_last_raw_pressed = Task2_ReadKeyPressed();
    s_key_stable_pressed = s_key_last_raw_pressed;
    s_key_same_frames = TASK2_KEY_DEBOUNCE_FRAMES;

    IR_ResetController();
    IR_SetBaseSpeed(IR_SPEED_FAST);
    Contrl_Speed(0, 0, 0, 0);
}

void Task2_Process(void)
{
    bool key_event = Task2_TakeKeyPressedEvent();
    bool marker;

    switch (g_task2_state)
    {
        case TASK2_STATE_WAIT_KEY:
            Contrl_Speed(0, 0, 0, 0);

            if (key_event)
            {
                Task2_StartRun();
            }
            break;

        case TASK2_STATE_LEAVING_A:
            Task2_UpdateApproachSpeed();
            LineWalking();

            marker = Task2_IsAMarker();
            g_task2_marker_now = marker ? 1U : 0U;

            /*
             * 只有看到普通环线（1~4路黑）并连续保持若干帧，
             * 才认为已经真正驶离初始A点。
             */
            if (!marker &&
                (g_ir_black_count > 0U) &&
                (g_ir_black_count < TASK2_MARKER_MIN_BLACK))
            {
                if (s_leave_clear_frames < TASK2_LEAVE_CLEAR_FRAMES)
                {
                    s_leave_clear_frames++;
                }

                if (s_leave_clear_frames >= TASK2_LEAVE_CLEAR_FRAMES)
                {
                    g_task2_state = TASK2_STATE_RUNNING;
                }
            }
            else
            {
                s_leave_clear_frames = 0U;
            }

            if (g_task2_elapsed_ms >= TASK2_MAX_RUN_MS)
            {
                Task2_Stop(TASK2_STOP_TIMEOUT);
            }
            break;

        case TASK2_STATE_RUNNING:
            Task2_UpdateApproachSpeed();
            LineWalking();

            marker = Task2_IsAMarker();
            g_task2_marker_now = marker ? 1U : 0U;

            if ((g_task2_elapsed_ms >= TASK2_MIN_LAP_MS) && marker)
            {
                if (g_task2_marker_hits <
                    TASK2_MARKER_CONFIRM_FRAMES)
                {
                    g_task2_marker_hits++;
                }

                if (g_task2_marker_hits >=
                    TASK2_MARKER_CONFIRM_FRAMES)
                {
                    Task2_Stop(TASK2_STOP_A_MARKER);
                }
            }
            else
            {
                g_task2_marker_hits = 0U;
            }

            if ((g_task2_state != TASK2_STATE_FINISHED) &&
                (g_task2_elapsed_ms >= TASK2_MAX_RUN_MS))
            {
                Task2_Stop(TASK2_STOP_TIMEOUT);
            }
            break;

        case TASK2_STATE_FINISHED:
        default:
            Contrl_Speed(0, 0, 0, 0);
            g_task2_marker_now = 0U;

            /* 松开后再次按键，可以直接开始下一次测试。 */
            if (key_event)
            {
                Task2_StartRun();
            }
            break;
    }
}

void Task2_Tick(uint16_t elapsed_ms)
{
    if ((g_task2_state == TASK2_STATE_LEAVING_A) ||
        (g_task2_state == TASK2_STATE_RUNNING))
    {
        float right_turn_rate;
        uint32_t updated_time =
            g_task2_elapsed_ms + (uint32_t)elapsed_ms;

        if (updated_time < g_task2_elapsed_ms)
        {
            updated_time = 0xFFFFFFFFU;
        }
        g_task2_elapsed_ms = updated_time;

        if (IMU_IsOnline() && IMU_IsCalibrated())
        {
            right_turn_rate =
                IMU_GetGyroZDps() * TASK2_GYRO_Z_TO_RIGHT_SIGN;

            if (right_turn_rate > TASK2_GYRO_TURN_DEADBAND_DPS)
            {
                g_task2_right_turn_deg +=
                    right_turn_rate * ((float)elapsed_ms / 1000.0f);
            }
        }
    }
}

bool Task2_IsFinished(void)
{
    return (g_task2_state == TASK2_STATE_FINISHED);
}