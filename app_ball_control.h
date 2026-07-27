/*
 * app_ball_control.h
 *
 * K230钢球低速移动跟踪控制
 *
 * 通信协议：
 *   有目标：$1,X,Y#
 *   无目标：$0,0,0#
 *
 * 控制原则：
 *   X控制左右转向
 *   Y控制前进和停车
 */

#ifndef APP_BALL_CONTROL_H
#define APP_BALL_CONTROL_H

#include "app_common.h"
#include <stdbool.h>

/* ============================================================
 * 目标确认与丢失防抖
 * ============================================================ */

/* 连续收到2帧有效目标后才开始运动 */
#define BALL_TARGET_LOCK_FRAMES       2U

/* 连续3帧无目标后，彻底判定目标丢失 */
#define BALL_TARGET_LOST_FRAMES       3U

/* ============================================================
 * 坐标滤波
 *
 * filtered += (raw - filtered) * 3 / 8
 *
 * 数值越大响应越快，但坐标抖动也越明显。
 * ============================================================ */

#define BALL_FILTER_NUM               3
#define BALL_FILTER_DEN               8

/* ============================================================
 * 水平方向控制参数
 * ============================================================ */

/* 画面中心。640像素宽度时为320 */
#define BALL_IMAGE_CENTER_X           320

/* 中心死区：误差在±22像素内不转向 */
#define BALL_X_DEADBAND               22

/*
 * 横向误差超过110像素时：
 * 只转向对准，不向前移动。
 */
#define BALL_ALIGN_STOP_ERROR         110

/*
 * 横向误差超过60像素但未达到110像素时：
 * 允许前进，但降低前进速度。
 */
#define BALL_ALIGN_SLOW_ERROR         60

/* 转向PD参数 */
#define BALL_TURN_KP                  4
#define BALL_TURN_KD                  7

/*
 * Motion_Car_Control内部会把Vz换算为左右轮差速。
 * 原地对准时需要较大的最小Vz，避免轮速过低无法起转。
 */
#define BALL_TURN_MIN_ALIGN           700
#define BALL_TURN_MIN_MOVE            260
#define BALL_TURN_LIMIT               1000

/* ============================================================
 * 前后距离控制参数
 * ============================================================ */

/*
 * 希望钢球稳定停留在画面中的Y坐标。
 *
 * 必须根据实际摄像头安装角度标定。
 * 初始参考值：360。
 */
#define BALL_HOLD_Y                   360

/* 停车位置的Y方向死区 */
#define BALL_Y_DEADBAND               20

/*
 * Y超过该值说明钢球过近。
 * 当前策略只停车，不自动倒车。
 */
#define BALL_TOO_CLOSE_Y              410

/* 低速跟踪速度 */
#define BALL_FORWARD_MIN              180
#define BALL_FORWARD_MAX              260

/* Y方向比例：钢球越远，速度越高 */
#define BALL_FORWARD_KY               2

/*
 * 横向偏差较大时，前进速度降低到55%。
 */
#define BALL_FORWARD_TURN_PERCENT     55

/* ============================================================
 * 指令平滑参数
 *
 * 当前主循环约10ms调用一次。
 * ============================================================ */

/* 前进速度每周期最多增加4 */
#define BALL_FORWARD_ACCEL_STEP       4

/* 前进速度每周期最多减少18，停车快于启动 */
#define BALL_FORWARD_DECEL_STEP       18

/* 转向指令每周期最大变化量 */
#define BALL_TURN_RAMP_STEP           80

/* ============================================================
 * IMU角速度阻尼
 * ============================================================ */

#define BALL_IMU_ASSIST_ENABLE        1

/*
 * 与现有循迹模块保持相同方向：
 * 正Vz代表右转。
 *
 * 如果加入IMU后转向振荡变大，
 * 将-1.0f改成+1.0f。
 */
#define BALL_IMU_GYRO_Z_SIGN          (-1.0f)

#define BALL_IMU_DAMP_GAIN            3.0f
#define BALL_IMU_MIN_DAMP_DPS         1.5f

typedef enum
{
    BALL_CTRL_COMM_LOST = 0,
    BALL_CTRL_NO_TARGET,
    BALL_CTRL_LOCKING,
    BALL_CTRL_SHORT_LOST,
    BALL_CTRL_ALIGNING,
    BALL_CTRL_FOLLOWING,
    BALL_CTRL_HOLDING
} BallControlState_t;

/* CCS Expressions中可直接观察 */
extern volatile BallControlState_t g_ball_control_state;

extern volatile int16_t g_ball_raw_x;
extern volatile int16_t g_ball_raw_y;

extern volatile int16_t g_ball_filtered_x;
extern volatile int16_t g_ball_filtered_y;

extern volatile int16_t g_ball_error_x;

extern volatile int16_t g_ball_forward_cmd;
extern volatile int16_t g_ball_turn_cmd;

/* 初始化跟踪控制器 */
void BallControl_Init(void);

/* 每个主控制周期调用一次 */
void BallControl_Process(void);

/* 立即停车，并清除电机指令 */
void BallControl_Stop(void);

/* 当前是否处于有效跟踪状态 */
bool BallControl_IsTracking(void);

#endif /* APP_BALL_CONTROL_H */