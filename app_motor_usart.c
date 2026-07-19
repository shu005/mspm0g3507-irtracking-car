/*
 * app_motor_usart.c - 电机驱动板串口通信实现
 * Motor driver board UART communication implementation
 *
 * 协议: ASCII 格式, $命令:参数#
 * 示例: $mtype:2#  $speed:300,300,300,300#
 *
 * 接线 (参考 MSPM0 机器人扩展板):
 *   MSPM0 UART TX -> 电机驱动板 RX
 *   MSPM0 UART RX -> 电机驱动板 TX (可选)
 *   具体引脚见扩展板丝印
 */

#include "app_motor_usart.h"

/* ========== 发送缓冲区 ========== */
uint8_t send_buff[128];

/* ========== 串口实例配置 ========== */
/*
 * SysConfig 中命名为 UART_2 → 生成 UART_2_INST
 * 四路电机驱动板: RX2->PB6, TX2->PB7
 * 如果 SysConfig 里改了命名, 修改下面的宏
 */
#ifndef MOTOR_UART_INST
#define MOTOR_UART_INST        UART_2_INST
#endif

/* ========== 发送原始数据 ========== */
/*
 * 通过串口向电机驱动板发送字节数组
 * 使用轮询方式 (阻塞发送), 每条指令几十字节, 不影响巡线
 */
void Send_Motor_ArrayU8(const uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++)
    {
        DL_UART_transmitDataBlocking(MOTOR_UART_INST, data[i]);
    }
}

/* ========== 初始化串口 ========== */
/*
 * 基础配置 (波特率/引脚) 由 SysConfig 生成的 SYSCFG_DL_init 完成
 * 这里可添加 DMA 中断等高级功能
 */
void USART_Init(void)
{
    /*
     * 如果 SysConfig 中配置了 DMA, 取消下面注释以启用 DMA 发送
     * (需要先在 SysConfig 里添加 DMA 并关联 UART1 TX)
     *
     * NVIC_ClearPendingIRQ(UART_1_INST_INT_IRQN);
     * DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
     * NVIC_EnableIRQ(UART_1_INST_INT_IRQN);
     */
}

/* ========== 配置命令发送函数 ========== */

/* 发送电机类型 */
void send_motor_type(uint16_t data)
{
    snprintf((char *)send_buff, sizeof(send_buff), "$mtype:%d#", data);
    Send_Motor_ArrayU8(send_buff, (uint16_t)strlen((char *)send_buff));
}

/* 发送电机死区 */
void send_motor_deadzone(uint16_t data)
{
    snprintf((char *)send_buff, sizeof(send_buff), "$deadzone:%d#", data);
    Send_Motor_ArrayU8(send_buff, (uint16_t)strlen((char *)send_buff));
}

/* 发送磁环脉冲数 */
void send_pulse_line(uint16_t data)
{
    snprintf((char *)send_buff, sizeof(send_buff), "$mline:%d#", data);
    Send_Motor_ArrayU8(send_buff, (uint16_t)strlen((char *)send_buff));
}

/* 发送减速比 */
void send_pulse_phase(uint16_t data)
{
    snprintf((char *)send_buff, sizeof(send_buff), "$mphase:%d#", data);
    Send_Motor_ArrayU8(send_buff, (uint16_t)strlen((char *)send_buff));
}

/* 发送轮子直径 */
void send_wheel_diameter(float data)
{
    snprintf((char *)send_buff, sizeof(send_buff),
             "$wdiameter:%.0f#", data);
    Send_Motor_ArrayU8(send_buff, (uint16_t)strlen((char *)send_buff));
}
