/*
 * app_k230.c - K230 UART接收、协议解析与超时检测
 */

#include "app_k230.h"

/* SysConfig中的实例名必须为UART_K230 */
#ifndef K230_UART_INST
#define K230_UART_INST       UART_K230_INST
#endif

#ifndef K230_UART_IRQN
#define K230_UART_IRQN       UART_K230_INST_INT_IRQN
#endif

/*
 * 256字节足以容纳直角弯期间积累的数据。
 * 必须是2的整数次幂。
 */
#define K230_RX_BUFFER_SIZE      256U
#define K230_RX_BUFFER_MASK      (K230_RX_BUFFER_SIZE - 1U)

/* 最大协议帧远小于32字节 */
#define K230_FRAME_BUFFER_SIZE   32U

#if ((K230_RX_BUFFER_SIZE & (K230_RX_BUFFER_SIZE - 1U)) != 0U)
#error "K230_RX_BUFFER_SIZE must be a power of two"
#endif

K230_BallData_t g_k230_ball;
K230_Diagnostics_t g_k230_diag;

/* UART中断环形缓冲区 */
static volatile uint8_t s_rx_buffer[K230_RX_BUFFER_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;

/* 协议帧缓存，不包含$和# */
static char s_frame_buffer[K230_FRAME_BUFFER_SIZE];
static uint8_t s_frame_index = 0U;
static bool s_receiving_frame = false;

static void K230_RxPushFromISR(uint8_t byte);
static bool K230_RxPop(uint8_t *byte);
static void K230_ParseByte(uint8_t byte);
static void K230_ParseFrame(const char *frame);

static bool K230_ParseNumber(const char **cursor,
                             uint16_t max_value,
                             uint16_t *result);

static uint16_t K230_SaturatingAdd(uint16_t value,
                                  uint16_t increment);

/* ============================================================
 * 初始化
 * ============================================================ */
void K230_Init(void)
{
    memset(&g_k230_ball, 0, sizeof(g_k230_ball));
    memset(&g_k230_diag, 0, sizeof(g_k230_diag));

    s_rx_head = 0U;
    s_rx_tail = 0U;

    s_frame_index = 0U;
    s_receiving_frame = false;

    /*
     * 0xFFFF表示上电后尚未收到合法数据。
     */
    g_k230_ball.packet_age_ms = 0xFFFFU;

    /* 清除上电时UART FIFO里可能残留的数据 */
    while (!DL_UART_Main_isRXFIFOEmpty(K230_UART_INST))
    {
        (void)DL_UART_Main_receiveData(K230_UART_INST);
    }

    /* SysConfig已经启用Receive中断，这里再次使能没有问题 */
    DL_UART_Main_enableInterrupt(
        K230_UART_INST,
        DL_UART_MAIN_INTERRUPT_RX
    );

    NVIC_ClearPendingIRQ(K230_UART_IRQN);
    NVIC_EnableIRQ(K230_UART_IRQN);

    __enable_irq();
}

/* ============================================================
 * 主循环解析入口
 * ============================================================ */
void K230_Process(void)
{
    uint8_t byte;

    while (K230_RxPop(&byte))
    {
        K230_ParseByte(byte);
    }
}

/* ============================================================
 * 时间更新与超时处理
 * ============================================================ */
void K230_Tick(uint16_t elapsed_ms)
{
    g_k230_ball.packet_age_ms =
        K230_SaturatingAdd(
            g_k230_ball.packet_age_ms,
            elapsed_ms
        );

    /*
     * 通信超时后立即作废目标，禁止继续使用旧坐标。
     */
    if (g_k230_ball.packet_age_ms >
        (uint16_t)K230_ONLINE_TIMEOUT_MS)
    {
        g_k230_ball.state = 0U;
        g_k230_ball.target_valid = false;
        g_k230_ball.x = 0U;
        g_k230_ball.y = 0U;
    }
}

bool K230_IsOnline(void)
{
    return (g_k230_diag.valid_frames > 0U) &&
           (g_k230_ball.packet_age_ms <=
            (uint16_t)K230_ONLINE_TIMEOUT_MS);
}

bool K230_HasTarget(void)
{
    return K230_IsOnline() &&
           g_k230_ball.target_valid &&
           (g_k230_ball.state == 1U);
}

bool K230_GetTarget(uint16_t *x, uint16_t *y)
{
    if ((x == 0) || (y == 0))
    {
        return false;
    }

    if (!K230_HasTarget())
    {
        return false;
    }

    *x = g_k230_ball.x;
    *y = g_k230_ball.y;

    return true;
}

bool K230_TakeNewFrame(void)
{
    bool result = g_k230_ball.new_frame;

    g_k230_ball.new_frame = false;

    return result;
}

/* ============================================================
 * 环形缓冲区
 * ============================================================ */
static void K230_RxPushFromISR(uint8_t byte)
{
    uint16_t next =
        (uint16_t)((s_rx_head + 1U) &
                   K230_RX_BUFFER_MASK);

    if (next == s_rx_tail)
    {
        g_k230_diag.ring_overflows++;
        return;
    }

    s_rx_buffer[s_rx_head] = byte;
    s_rx_head = next;

    g_k230_diag.rx_bytes++;
}

static bool K230_RxPop(uint8_t *byte)
{
    if (byte == 0)
    {
        return false;
    }

    if (s_rx_tail == s_rx_head)
    {
        return false;
    }

    *byte = s_rx_buffer[s_rx_tail];

    s_rx_tail =
        (uint16_t)((s_rx_tail + 1U) &
                   K230_RX_BUFFER_MASK);

    return true;
}

/* ============================================================
 * 字节状态机
 * ============================================================ */
static void K230_ParseByte(uint8_t byte)
{
    /*
     * 无论当前是否正在接收，只要收到$，
     * 都认为它是一帧新数据的开始。
     */
    if (byte == (uint8_t)'$')
    {
        s_receiving_frame = true;
        s_frame_index = 0U;
        return;
    }

    /* 没有收到帧头前，所有其他字符直接忽略 */
    if (!s_receiving_frame)
    {
        return;
    }

    /* 收到帧尾，开始解析 */
    if (byte == (uint8_t)'#')
    {
        if (s_frame_index == 0U)
        {
            g_k230_diag.format_errors++;
        }
        else
        {
            s_frame_buffer[s_frame_index] = '\0';
            K230_ParseFrame(s_frame_buffer);
        }

        s_receiving_frame = false;
        s_frame_index = 0U;
        return;
    }

    /*
     * 帧内只接受可见ASCII字符。
     * 遇到回车、换行或乱码时丢弃本帧，
     * 等待下一个$重新同步。
     */
    if ((byte < 0x20U) || (byte > 0x7EU))
    {
        g_k230_diag.format_errors++;

        s_receiving_frame = false;
        s_frame_index = 0U;
        return;
    }

    if (s_frame_index >=
        (K230_FRAME_BUFFER_SIZE - 1U))
    {
        g_k230_diag.frame_overflows++;

        s_receiving_frame = false;
        s_frame_index = 0U;
        return;
    }

    s_frame_buffer[s_frame_index++] = (char)byte;
}

/* ============================================================
 * 完整帧解析
 *
 * 输入示例：
 *   "1,392,226"
 *   "0,0,0"
 * ============================================================ */
static void K230_ParseFrame(const char *frame)
{
    const char *cursor = frame;

    uint16_t state;
    uint16_t x;
    uint16_t y;

    if (frame == 0)
    {
        return;
    }

    /* 状态字段 */
    if (!K230_ParseNumber(&cursor, 1U, &state))
    {
        g_k230_diag.format_errors++;
        return;
    }

    if (*cursor != ',')
    {
        g_k230_diag.format_errors++;
        return;
    }
    cursor++;

    /* X坐标字段 */
    if (!K230_ParseNumber(
            &cursor,
            (uint16_t)(K230_IMAGE_WIDTH - 1U),
            &x))
    {
        g_k230_diag.range_errors++;
        return;
    }

    if (*cursor != ',')
    {
        g_k230_diag.format_errors++;
        return;
    }
    cursor++;

    /* Y坐标字段 */
    if (!K230_ParseNumber(
            &cursor,
            (uint16_t)(K230_IMAGE_HEIGHT - 1U),
            &y))
    {
        g_k230_diag.range_errors++;
        return;
    }

    /* Y后面必须直接结束 */
    if (*cursor != '\0')
    {
        g_k230_diag.format_errors++;
        return;
    }

    /*
     * 无目标帧必须严格为$0,0,0#。
     */
    if (state == 0U)
    {
        if ((x != 0U) || (y != 0U))
        {
            g_k230_diag.format_errors++;
            return;
        }

        g_k230_ball.state = 0U;
        g_k230_ball.x = 0U;
        g_k230_ball.y = 0U;
        g_k230_ball.target_valid = false;
    }
    else
    {
        g_k230_ball.state = 1U;
        g_k230_ball.x = x;
        g_k230_ball.y = y;
        g_k230_ball.target_valid = true;
    }

    /*
     * 只有完整且合法的帧才刷新通信时间。
     * 乱码和半包不会被误判为通信正常。
     */
    g_k230_ball.packet_age_ms = 0U;
    g_k230_ball.new_frame = true;

    g_k230_diag.valid_frames++;
}

/* ============================================================
 * 解析一个无符号十进制整数
 * ============================================================ */
static bool K230_ParseNumber(const char **cursor,
                             uint16_t max_value,
                             uint16_t *result)
{
    const char *p;
    uint32_t value = 0U;
    bool has_digit = false;

    if ((cursor == 0) ||
        (*cursor == 0) ||
        (result == 0))
    {
        return false;
    }

    p = *cursor;

    while ((*p >= '0') && (*p <= '9'))
    {
        has_digit = true;

        value =
            value * 10U +
            (uint32_t)(*p - '0');

        if (value > (uint32_t)max_value)
        {
            return false;
        }

        p++;
    }

    if (!has_digit)
    {
        return false;
    }

    *cursor = p;
    *result = (uint16_t)value;

    return true;
}

static uint16_t K230_SaturatingAdd(uint16_t value,
                                  uint16_t increment)
{
    uint32_t sum =
        (uint32_t)value +
        (uint32_t)increment;

    if (sum > 0xFFFFU)
    {
        return 0xFFFFU;
    }

    return (uint16_t)sum;
}

/* ============================================================
 * UART2接收中断
 * ============================================================ */
void UART_K230_INST_IRQHandler(void)
{
    switch (DL_UART_Main_getPendingInterrupt(
                K230_UART_INST))
    {
        case DL_UART_MAIN_IIDX_RX:

            while (!DL_UART_Main_isRXFIFOEmpty(
                        K230_UART_INST))
            {
                K230_RxPushFromISR(
                    (uint8_t)
                    DL_UART_Main_receiveData(
                        K230_UART_INST)
                );
            }
            break;

        default:
            break;
    }
}