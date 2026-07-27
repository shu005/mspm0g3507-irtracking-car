/*
 * app_ball_control.c
 *
 * K230钢球低速移动跟踪控制
 */

#include "app_ball_control.h"
#include "app_k230.h"
#include "app_motor.h"
#include "app_imu.h"

/* ============================================================
 * CCS可观察变量
 * ============================================================ */

volatile BallControlState_t g_ball_control_state =
    BALL_CTRL_COMM_LOST;

volatile int16_t g_ball_raw_x = 0;
volatile int16_t g_ball_raw_y = 0;

volatile int16_t g_ball_filtered_x = 0;
volatile int16_t g_ball_filtered_y = 0;

volatile int16_t g_ball_error_x = 0;

volatile int16_t g_ball_forward_cmd = 0;
volatile int16_t g_ball_turn_cmd = 0;

/* ============================================================
 * 内部状态
 * ============================================================ */

static bool s_filter_initialized = false;
static bool s_current_frame_has_target = false;
static bool s_target_locked = false;

static uint8_t s_valid_frame_count = 0U;
static uint8_t s_lost_frame_count = 0U;

/*
 * 仅在收到新视觉帧时更新。
 * 用于X方向微分预测。
 */
static int16_t s_x_delta = 0;

/* 平滑后的实际输出指令 */
static int16_t s_forward_command = 0;
static int16_t s_turn_command = 0;

/* ============================================================
 * 内部函数声明
 * ============================================================ */

static int32_t Ball_Abs32(int32_t value);

static int32_t Ball_Clamp32(int32_t value,
                            int32_t minimum,
                            int32_t maximum);

static int16_t Ball_RampForward(int16_t current,
                                int16_t target);

static int16_t Ball_RampTurn(int16_t current,
                             int16_t target);

static void Ball_ResetTargetState(void);

static void Ball_HandleNewTargetFrame(void);

static void Ball_HandleNewNoTargetFrame(void);

static void Ball_ApplySmoothCommand(int16_t target_forward,
                                    int16_t target_turn);

/* ============================================================
 * 初始化
 * ============================================================ */

void BallControl_Init(void)
{
    g_ball_control_state = BALL_CTRL_COMM_LOST;

    g_ball_raw_x = 0;
    g_ball_raw_y = 0;

    g_ball_filtered_x = 0;
    g_ball_filtered_y = 0;

    g_ball_error_x = 0;

    g_ball_forward_cmd = 0;
    g_ball_turn_cmd = 0;

    s_filter_initialized = false;
    s_current_frame_has_target = false;
    s_target_locked = false;

    s_valid_frame_count = 0U;
    s_lost_frame_count = 0U;

    s_x_delta = 0;

    s_forward_command = 0;
    s_turn_command = 0;

    Motion_Car_Control(0, 0, 0);
}

/* ============================================================
 * 主控制函数
 * ============================================================ */

void BallControl_Process(void)
{
    bool new_frame;

    int32_t error_x;
    int32_t effective_error_x;
    int32_t abs_error_x;

    int32_t y_error;

    int32_t desired_forward;
    int32_t desired_turn;

    int32_t minimum_turn;

    /*
     * 1. 通信超时：
     *    立即停车，不允许缓慢减速。
     */
    if (!K230_IsOnline())
    {
        g_ball_control_state = BALL_CTRL_COMM_LOST;

        Ball_ResetTargetState();
        BallControl_Stop();
        return;
    }

    /*
     * 2. 只在真正收到K230新帧时更新坐标滤波和计数。
     *
     * 主循环10ms一次，而K230一般50~100ms一帧，
     * 不能在同一视觉帧上重复增加有效帧计数。
     */
    s_x_delta = 0;

    new_frame = K230_TakeNewFrame();

    if (new_frame)
    {
        if ((g_k230_ball.state == 1U) &&
            g_k230_ball.target_valid)
        {
            Ball_HandleNewTargetFrame();
        }
        else
        {
            Ball_HandleNewNoTargetFrame();
        }
    }

    /*
     * 3. K230明确发送无目标帧。
     *
     * 第1、2帧无目标时先平滑减速；
     * 连续达到阈值后立即停车。
     */
    if (!s_current_frame_has_target)
    {
        if (s_lost_frame_count >=
            BALL_TARGET_LOST_FRAMES)
        {
            g_ball_control_state =
                BALL_CTRL_NO_TARGET;

            BallControl_Stop();
        }
        else
        {
            g_ball_control_state =
                BALL_CTRL_SHORT_LOST;

            Ball_ApplySmoothCommand(0, 0);
        }

        return;
    }

    /*
     * 4. 刚识别到目标，但尚未连续确认。
     */
    if (!s_target_locked)
    {
        g_ball_control_state =
            BALL_CTRL_LOCKING;

        Ball_ApplySmoothCommand(0, 0);
        return;
    }

    /*
     * 5. 计算水平偏差。
     *
     * X大于320：钢球在右边，输出正Vz右转。
     * X小于320：钢球在左边，输出负Vz左转。
     */
    error_x =
        (int32_t)g_ball_filtered_x -
        (int32_t)BALL_IMAGE_CENTER_X;

    abs_error_x = Ball_Abs32(error_x);

    /*
     * 中心死区。
     */
    if (abs_error_x <= BALL_X_DEADBAND)
    {
        effective_error_x = 0;
        error_x = 0;
        abs_error_x = 0;
    }
    else if (error_x > 0)
    {
        effective_error_x =
            error_x - BALL_X_DEADBAND;
    }
    else
    {
        effective_error_x =
            error_x + BALL_X_DEADBAND;
    }

    g_ball_error_x = (int16_t)error_x;

    /*
     * 6. 根据Y坐标计算目标前进速度。
     *
     * 图像通常为：
     *   Y较小 → 钢球较远
     *   Y较大 → 钢球较近
     */
    y_error =
        (int32_t)BALL_HOLD_Y -
        (int32_t)g_ball_filtered_y;

    desired_forward = 0;

    /*
     * 钢球过近：停车。
     */
    if (g_ball_filtered_y >=
        BALL_TOO_CLOSE_Y)
    {
        desired_forward = 0;

        g_ball_control_state =
            BALL_CTRL_HOLDING;
    }
    /*
     * 已经进入目标距离附近：停车等待。
     */
    else if (y_error <= BALL_Y_DEADBAND)
    {
        desired_forward = 0;

        g_ball_control_state =
            BALL_CTRL_HOLDING;
    }
    /*
     * 横向偏差太大：
     * 先原地对准，不向前冲。
     */
    else if (abs_error_x >=
             BALL_ALIGN_STOP_ERROR)
    {
        desired_forward = 0;

        g_ball_control_state =
            BALL_CTRL_ALIGNING;
    }
    else
    {
        /*
         * 距离越远，前进速度越高。
         */
        desired_forward =
            BALL_FORWARD_MIN +
            (y_error - BALL_Y_DEADBAND) *
            BALL_FORWARD_KY;

        desired_forward =
            Ball_Clamp32(
                desired_forward,
                BALL_FORWARD_MIN,
                BALL_FORWARD_MAX
            );

        /*
         * 钢球不在画面中心时降低速度，
         * 防止边冲边转导致丢目标。
         */
        if (abs_error_x >
            BALL_ALIGN_SLOW_ERROR)
        {
            desired_forward =
                (desired_forward *
                 BALL_FORWARD_TURN_PERCENT) /
                100;
        }

        g_ball_control_state =
            BALL_CTRL_FOLLOWING;
    }

    /*
     * 7. 水平转向PD。
     *
     * P：根据当前水平误差转向；
     * D：根据钢球横向移动趋势提前补偿。
     */
    desired_turn =
        effective_error_x *
        BALL_TURN_KP;

    desired_turn +=
        (int32_t)s_x_delta *
        BALL_TURN_KD;

    /*
     * 8. 设置最低有效转向指令。
     *
     * 原地转向时需要更大指令；
     * 前进过程中只需较小差速。
     */
    if (desired_forward == 0)
    {
        minimum_turn =
            BALL_TURN_MIN_ALIGN;
    }
    else
    {
        minimum_turn =
            BALL_TURN_MIN_MOVE;
    }

    if (effective_error_x != 0)
    {
        int32_t abs_turn =
            Ball_Abs32(desired_turn);

        /*
         * 误差明显时才强制最低转向，
         * 避免刚出死区就突然大转。
         */
        if ((abs_error_x >=
             BALL_ALIGN_SLOW_ERROR) &&
            (abs_turn < minimum_turn))
        {
            desired_turn =
                (desired_turn >= 0) ?
                minimum_turn :
                -minimum_turn;
        }
    }
    else
    {
        desired_turn = 0;
    }

#if BALL_IMU_ASSIST_ENABLE

    /*
     * 9. IMU角速度阻尼。
     *
     * 当车体仍在快速旋转时，给出反向阻尼，
     * 减少转过头和左右摆动。
     */
    if (IMU_IsOnline() &&
        IMU_IsCalibrated())
    {
        float yaw_rate;

        yaw_rate =
            IMU_GetGyroZDps() *
            BALL_IMU_GYRO_Z_SIGN;

        if ((yaw_rate >
             BALL_IMU_MIN_DAMP_DPS) ||
            (yaw_rate <
             -BALL_IMU_MIN_DAMP_DPS))
        {
            desired_turn -=
                (int32_t)(
                    yaw_rate *
                    BALL_IMU_DAMP_GAIN
                );
        }
    }

#endif

    desired_turn =
        Ball_Clamp32(
            desired_turn,
            -BALL_TURN_LIMIT,
            BALL_TURN_LIMIT
        );

    /*
     * 10. 平滑输出给电机。
     */
    Ball_ApplySmoothCommand(
        (int16_t)desired_forward,
        (int16_t)desired_turn
    );
}

/* ============================================================
 * 新有效目标帧处理
 * ============================================================ */

static void Ball_HandleNewTargetFrame(void)
{
    int32_t old_filtered_x;
    int32_t difference_x;
    int32_t difference_y;

    g_ball_raw_x =
        (int16_t)g_k230_ball.x;

    g_ball_raw_y =
        (int16_t)g_k230_ball.y;

    /*
     * 首次识别或重新捕获目标：
     * 直接使用当前坐标，避免被旧坐标拖慢。
     */
    if ((!s_filter_initialized) ||
        (!s_current_frame_has_target))
    {
        g_ball_filtered_x =
            g_ball_raw_x;

        g_ball_filtered_y =
            g_ball_raw_y;

        s_x_delta = 0;
        s_filter_initialized = true;
    }
    else
    {
        old_filtered_x =
            g_ball_filtered_x;

        difference_x =
            (int32_t)g_ball_raw_x -
            (int32_t)g_ball_filtered_x;

        difference_y =
            (int32_t)g_ball_raw_y -
            (int32_t)g_ball_filtered_y;

        g_ball_filtered_x +=
            (int16_t)(
                difference_x *
                BALL_FILTER_NUM /
                BALL_FILTER_DEN
            );

        g_ball_filtered_y +=
            (int16_t)(
                difference_y *
                BALL_FILTER_NUM /
                BALL_FILTER_DEN
            );

        /*
         * 目标横向速度的近似量。
         */
        s_x_delta =
            (int16_t)(
                (int32_t)g_ball_filtered_x -
                old_filtered_x
            );
    }

    s_current_frame_has_target = true;
    s_lost_frame_count = 0U;

    if (s_valid_frame_count < 255U)
    {
        s_valid_frame_count++;
    }

    if (s_valid_frame_count >=
        BALL_TARGET_LOCK_FRAMES)
    {
        s_target_locked = true;
    }
}

/* ============================================================
 * 新无目标帧处理
 * ============================================================ */

static void Ball_HandleNewNoTargetFrame(void)
{
    s_current_frame_has_target = false;
    s_valid_frame_count = 0U;
    s_x_delta = 0;

    if (s_lost_frame_count < 255U)
    {
        s_lost_frame_count++;
    }

    if (s_lost_frame_count >=
        BALL_TARGET_LOST_FRAMES)
    {
        s_target_locked = false;
        s_filter_initialized = false;

        g_ball_raw_x = 0;
        g_ball_raw_y = 0;

        g_ball_filtered_x = 0;
        g_ball_filtered_y = 0;

        g_ball_error_x = 0;
    }
}

/* ============================================================
 * 平滑输出
 * ============================================================ */

static void Ball_ApplySmoothCommand(int16_t target_forward,
                                    int16_t target_turn)
{
    s_forward_command =
        Ball_RampForward(
            s_forward_command,
            target_forward
        );

    s_turn_command =
        Ball_RampTurn(
            s_turn_command,
            target_turn
        );

    g_ball_forward_cmd =
        s_forward_command;

    g_ball_turn_cmd =
        s_turn_command;

    Motion_Car_Control(
        s_forward_command,
        0,
        s_turn_command
    );
}

/* ============================================================
 * 停车
 * ============================================================ */

void BallControl_Stop(void)
{
    s_forward_command = 0;
    s_turn_command = 0;

    g_ball_forward_cmd = 0;
    g_ball_turn_cmd = 0;

    Motion_Car_Control(0, 0, 0);
}

/* ============================================================
 * 状态查询
 * ============================================================ */

bool BallControl_IsTracking(void)
{
    return
        (g_ball_control_state ==
         BALL_CTRL_ALIGNING) ||

        (g_ball_control_state ==
         BALL_CTRL_FOLLOWING) ||

        (g_ball_control_state ==
         BALL_CTRL_HOLDING);
}

/* ============================================================
 * 内部辅助函数
 * ============================================================ */

static void Ball_ResetTargetState(void)
{
    s_filter_initialized = false;
    s_current_frame_has_target = false;
    s_target_locked = false;

    s_valid_frame_count = 0U;
    s_lost_frame_count = 0U;

    s_x_delta = 0;

    g_ball_raw_x = 0;
    g_ball_raw_y = 0;

    g_ball_filtered_x = 0;
    g_ball_filtered_y = 0;

    g_ball_error_x = 0;
}

static int32_t Ball_Abs32(int32_t value)
{
    return (value >= 0) ?
           value : -value;
}

static int32_t Ball_Clamp32(int32_t value,
                            int32_t minimum,
                            int32_t maximum)
{
    if (value > maximum)
    {
        return maximum;
    }

    if (value < minimum)
    {
        return minimum;
    }

    return value;
}

/*
 * 前进速度：
 * 启动慢，停车快。
 */
static int16_t Ball_RampForward(int16_t current,
                                int16_t target)
{
    int32_t difference =
        (int32_t)target -
        (int32_t)current;

    if (difference > 0)
    {
        if (difference >
            BALL_FORWARD_ACCEL_STEP)
        {
            current +=
                BALL_FORWARD_ACCEL_STEP;
        }
        else
        {
            current = target;
        }
    }
    else if (difference < 0)
    {
        if (-difference >
            BALL_FORWARD_DECEL_STEP)
        {
            current -=
                BALL_FORWARD_DECEL_STEP;
        }
        else
        {
            current = target;
        }
    }

    return current;
}

/*
 * 转向速度双向平滑。
 */
static int16_t Ball_RampTurn(int16_t current,
                             int16_t target)
{
    int32_t difference =
        (int32_t)target -
        (int32_t)current;

    if (difference >
        BALL_TURN_RAMP_STEP)
    {
        current +=
            BALL_TURN_RAMP_STEP;
    }
    else if (difference <
             -BALL_TURN_RAMP_STEP)
    {
        current -=
            BALL_TURN_RAMP_STEP;
    }
    else
    {
        current = target;
    }

    return current;
}