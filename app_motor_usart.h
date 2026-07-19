/*
 * app_motor_usart.h - 电机驱动板串口通信接口
 * Motor driver board UART communication interface
 *
 * 协议格式: $命令:参数#
 * 例如: $mtype:2#  (设置电机类型为310)
 */

#ifndef APP_MOTOR_USART_H
#define APP_MOTOR_USART_H

#include "app_common.h"

/* ========== UART 发送函数 ========== */

/* 发送原始字节数组到电机驱动板 */
void Send_Motor_ArrayU8(const uint8_t *data, uint16_t len);

/* 配置命令发送 */
void send_motor_type(uint16_t data);        /* $mtype:N#      电机类型 */
void send_motor_deadzone(uint16_t data);    /* $deadzone:N#   电机死区 */
void send_pulse_line(uint16_t data);        /* $mline:N#      磁环线数 */
void send_pulse_phase(uint16_t data);       /* $mphase:N#     减速比脉冲 */
void send_wheel_diameter(float data);       /* $wdiameter:N#  轮子直径 */

/* 初始化与电机驱动板的串口通信 */
void USART_Init(void);

/* 获取发送缓冲区 (调试用) */
extern uint8_t send_buff[128];

#endif /* APP_MOTOR_USART_H */
