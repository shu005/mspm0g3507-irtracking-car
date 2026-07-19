/*
 * app_common.h - 公共类型定义和配置
 * Common type definitions and configuration
 */

#ifndef APP_COMMON_H
#define APP_COMMON_H

#include "ti_msp_dl_config.h"
#include <ti/driverlib/m0p/dl_core.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/* 类型别名 Type aliases */
typedef uint8_t  u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef int8_t   s8;
typedef int16_t  s16;
typedef int32_t  s32;

/* 系统时钟 32MHz (SysConfig 默认配置) */
#define SYSTEM_CLOCK_HZ  32000000

/* 毫秒延时 (基于 CPU 周期) */
static inline void delay_ms(uint32_t ms)
{
    uint32_t cycles_per_ms = SYSTEM_CLOCK_HZ / 1000;
    for (uint32_t i = 0; i < ms; i++) {
        delay_cycles(cycles_per_ms);
    }
}

#endif /* APP_COMMON_H */
