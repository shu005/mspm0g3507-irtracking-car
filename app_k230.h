/*
 * app_k230.h - K230 UART通信与钢球坐标解析
 *
 * 协议：
 *   有目标：$1,X,Y#
 *   无目标：$0,0,0#
 *
 * UART：
 *   SysConfig实例名：UART_K230
 *   UART2_RX：PA22 <- K230 TX
 *   UART2_TX：PA21 -> K230 RX（目前可不接）
 *   115200，8N1
 */

#ifndef APP_K230_H
#define APP_K230_H

#include "app_common.h"
#include <stdbool.h>

/* 根据K230实际输出分辨率修改 */
#define K230_IMAGE_WIDTH          640U
#define K230_IMAGE_HEIGHT         480U

/* 超过300ms没有收到完整合法帧，认为通信离线 */
#define K230_ONLINE_TIMEOUT_MS    300U

typedef struct
{
    /* 最近一帧原始状态：0=无目标，1=有目标 */
    uint8_t state;

    /* 最近一次有效钢球坐标 */
    uint16_t x;
    uint16_t y;

    /* 当前目标是否有效 */
    bool target_valid;

    /* 是否收到尚未读取的新帧 */
    bool new_frame;

    /* 距离最后一帧合法数据的时间 */
    uint16_t packet_age_ms;
} K230_BallData_t;

typedef struct
{
    uint32_t rx_bytes;
    uint32_t valid_frames;
    uint32_t format_errors;
    uint32_t range_errors;
    uint32_t frame_overflows;
    uint32_t ring_overflows;
} K230_Diagnostics_t;

/* 方便在CCS Expressions中直接观察 */
extern K230_BallData_t g_k230_ball;
extern K230_Diagnostics_t g_k230_diag;

/* 初始化接收缓冲区和UART中断 */
void K230_Init(void);

/* 解析中断缓存的数据，主循环频繁调用 */
void K230_Process(void);

/* 更新时间，用实际经过的毫秒数调用 */
void K230_Tick(uint16_t elapsed_ms);

/* 通信是否在线 */
bool K230_IsOnline(void);

/* 当前是否有有效钢球 */
bool K230_HasTarget(void);

/* 获取钢球坐标，成功返回true */
bool K230_GetTarget(uint16_t *x, uint16_t *y);

/* 查询并清除新帧标志 */
bool K230_TakeNewFrame(void);

#endif /* APP_K230_H */