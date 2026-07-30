/*
 * app_timebase.h - 1 ms system timebase based on Cortex-M0+ SysTick
 *
 * The independent clock keeps lap timing correct even when an OLED I2C
 * refresh temporarily makes one main-loop iteration longer than 10 ms.
 */

#ifndef APP_TIMEBASE_H
#define APP_TIMEBASE_H

#include <stdint.h>

void App_TimebaseInit(void);
uint32_t App_Millis(void);

#endif /* APP_TIMEBASE_H */

