/*
 * app_timebase.c - 1 ms Cortex-M0+ SysTick timebase
 */

#include "app_timebase.h"
#include "ti_msp_dl_config.h"

volatile uint32_t g_app_millis = 0U;

void SysTick_Handler(void)
{
    g_app_millis++;
}

void App_TimebaseInit(void)
{
    g_app_millis = 0U;

    /*
     * SysTick_Config loads CPUCLK_FREQ / 1000 - 1, selects the CPU clock,
     * enables the interrupt and starts the counter.
     */
    (void)SysTick_Config(CPUCLK_FREQ / 1000U);

    /* Keep UART receive interrupts above the timebase if priorities compete. */
    NVIC_SetPriority(SysTick_IRQn, 3U);
    __enable_irq();
}

uint32_t App_Millis(void)
{
    return g_app_millis;
}

