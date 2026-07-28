/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for UART_1 */
#define UART_1_INST                                                        UART1
#define UART_1_INST_FREQUENCY                                           32000000
#define UART_1_INST_IRQHandler                                  UART1_IRQHandler
#define UART_1_INST_INT_IRQN                                      UART1_INT_IRQn
#define GPIO_UART_1_RX_PORT                                                GPIOB
#define GPIO_UART_1_TX_PORT                                                GPIOB
#define GPIO_UART_1_RX_PIN                                         DL_GPIO_PIN_7
#define GPIO_UART_1_TX_PIN                                         DL_GPIO_PIN_6
#define GPIO_UART_1_IOMUX_RX                                     (IOMUX_PINCM24)
#define GPIO_UART_1_IOMUX_TX                                     (IOMUX_PINCM23)
#define GPIO_UART_1_IOMUX_RX_FUNC                      IOMUX_PINCM24_PF_UART1_RX
#define GPIO_UART_1_IOMUX_TX_FUNC                      IOMUX_PINCM23_PF_UART1_TX
#define UART_1_BAUD_RATE                                                (115200)
#define UART_1_IBRD_32_MHZ_115200_BAUD                                      (17)
#define UART_1_FBRD_32_MHZ_115200_BAUD                                      (23)
/* Defines for UART_IMU */
#define UART_IMU_INST                                                      UART3
#define UART_IMU_INST_FREQUENCY                                         32000000
#define UART_IMU_INST_IRQHandler                                UART3_IRQHandler
#define UART_IMU_INST_INT_IRQN                                    UART3_INT_IRQn
#define GPIO_UART_IMU_RX_PORT                                              GPIOA
#define GPIO_UART_IMU_TX_PORT                                              GPIOA
#define GPIO_UART_IMU_RX_PIN                                      DL_GPIO_PIN_25
#define GPIO_UART_IMU_TX_PIN                                      DL_GPIO_PIN_26
#define GPIO_UART_IMU_IOMUX_RX                                   (IOMUX_PINCM55)
#define GPIO_UART_IMU_IOMUX_TX                                   (IOMUX_PINCM59)
#define GPIO_UART_IMU_IOMUX_RX_FUNC                    IOMUX_PINCM55_PF_UART3_RX
#define GPIO_UART_IMU_IOMUX_TX_FUNC                    IOMUX_PINCM59_PF_UART3_TX
#define UART_IMU_BAUD_RATE                                              (115200)
#define UART_IMU_IBRD_32_MHZ_115200_BAUD                                    (17)
#define UART_IMU_FBRD_32_MHZ_115200_BAUD                                    (23)
/* Defines for UART_K230 */
#define UART_K230_INST                                                     UART2
#define UART_K230_INST_FREQUENCY                                        32000000
#define UART_K230_INST_IRQHandler                               UART2_IRQHandler
#define UART_K230_INST_INT_IRQN                                   UART2_INT_IRQn
#define GPIO_UART_K230_RX_PORT                                             GPIOA
#define GPIO_UART_K230_TX_PORT                                             GPIOB
#define GPIO_UART_K230_RX_PIN                                     DL_GPIO_PIN_22
#define GPIO_UART_K230_TX_PIN                                     DL_GPIO_PIN_15
#define GPIO_UART_K230_IOMUX_RX                                  (IOMUX_PINCM47)
#define GPIO_UART_K230_IOMUX_TX                                  (IOMUX_PINCM32)
#define GPIO_UART_K230_IOMUX_RX_FUNC                   IOMUX_PINCM47_PF_UART2_RX
#define GPIO_UART_K230_IOMUX_TX_FUNC                   IOMUX_PINCM32_PF_UART2_TX
#define UART_K230_BAUD_RATE                                             (115200)
#define UART_K230_IBRD_32_MHZ_115200_BAUD                                   (17)
#define UART_K230_FBRD_32_MHZ_115200_BAUD                                   (23)





/* Port definition for Pin Group IR_GPIO */
#define IR_GPIO_PORT                                                     (GPIOA)

/* Defines for IR_AD0: GPIOA.14 with pinCMx 36 on package pin 7 */
#define IR_GPIO_IR_AD0_PIN                                      (DL_GPIO_PIN_14)
#define IR_GPIO_IR_AD0_IOMUX                                     (IOMUX_PINCM36)
/* Defines for IR_AD1: GPIOA.15 with pinCMx 37 on package pin 8 */
#define IR_GPIO_IR_AD1_PIN                                      (DL_GPIO_PIN_15)
#define IR_GPIO_IR_AD1_IOMUX                                     (IOMUX_PINCM37)
/* Defines for IR_AD2: GPIOA.16 with pinCMx 38 on package pin 9 */
#define IR_GPIO_IR_AD2_PIN                                      (DL_GPIO_PIN_16)
#define IR_GPIO_IR_AD2_IOMUX                                     (IOMUX_PINCM38)
/* Defines for IR_OUT: GPIOA.17 with pinCMx 39 on package pin 10 */
#define IR_GPIO_IR_OUT_PIN                                      (DL_GPIO_PIN_17)
#define IR_GPIO_IR_OUT_IOMUX                                     (IOMUX_PINCM39)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_UART_1_init(void);
void SYSCFG_DL_UART_IMU_init(void);
void SYSCFG_DL_UART_K230_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
