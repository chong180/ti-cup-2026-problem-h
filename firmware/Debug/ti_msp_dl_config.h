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



/* Defines for PWM_MOTOR */
#define PWM_MOTOR_INST                                                     TIMG8
#define PWM_MOTOR_INST_IRQHandler                               TIMG8_IRQHandler
#define PWM_MOTOR_INST_INT_IRQN                                 (TIMG8_INT_IRQn)
#define PWM_MOTOR_INST_CLK_FREQ                                         32000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_C0_PORT                                             GPIOB
#define GPIO_PWM_MOTOR_C0_PIN                                     DL_GPIO_PIN_21
#define GPIO_PWM_MOTOR_C0_IOMUX                                  (IOMUX_PINCM49)
#define GPIO_PWM_MOTOR_C0_IOMUX_FUNC                 IOMUX_PINCM49_PF_TIMG8_CCP0
#define GPIO_PWM_MOTOR_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_C1_PORT                                             GPIOB
#define GPIO_PWM_MOTOR_C1_PIN                                     DL_GPIO_PIN_22
#define GPIO_PWM_MOTOR_C1_IOMUX                                  (IOMUX_PINCM50)
#define GPIO_PWM_MOTOR_C1_IOMUX_FUNC                 IOMUX_PINCM50_PF_TIMG8_CCP1
#define GPIO_PWM_MOTOR_C1_IDX                                DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMA1)
#define TIMER_0_INST_IRQHandler                                 TIMA1_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMA1_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                             (0U)
/* Defines for TIMER_1 */
#define TIMER_1_INST                                                     (TIMA0)
#define TIMER_1_INST_IRQHandler                                 TIMA0_IRQHandler
#define TIMER_1_INST_INT_IRQN                                   (TIMA0_INT_IRQn)
#define TIMER_1_INST_LOAD_VALUE                                             (0U)



/* Defines for UART_GYRO */
#define UART_GYRO_INST                                                     UART3
#define UART_GYRO_INST_FREQUENCY                                        32000000
#define UART_GYRO_INST_IRQHandler                               UART3_IRQHandler
#define UART_GYRO_INST_INT_IRQN                                   UART3_INT_IRQn
#define GPIO_UART_GYRO_RX_PORT                                             GPIOB
#define GPIO_UART_GYRO_TX_PORT                                             GPIOB
#define GPIO_UART_GYRO_RX_PIN                                      DL_GPIO_PIN_3
#define GPIO_UART_GYRO_TX_PIN                                      DL_GPIO_PIN_2
#define GPIO_UART_GYRO_IOMUX_RX                                  (IOMUX_PINCM16)
#define GPIO_UART_GYRO_IOMUX_TX                                  (IOMUX_PINCM15)
#define GPIO_UART_GYRO_IOMUX_RX_FUNC                   IOMUX_PINCM16_PF_UART3_RX
#define GPIO_UART_GYRO_IOMUX_TX_FUNC                   IOMUX_PINCM15_PF_UART3_TX
#define UART_GYRO_BAUD_RATE                                             (115200)
#define UART_GYRO_IBRD_32_MHZ_115200_BAUD                                   (17)
#define UART_GYRO_FBRD_32_MHZ_115200_BAUD                                   (23)
/* Defines for UART_dianji */
#define UART_dianji_INST                                                   UART2
#define UART_dianji_INST_FREQUENCY                                      32000000
#define UART_dianji_INST_IRQHandler                             UART2_IRQHandler
#define UART_dianji_INST_INT_IRQN                                 UART2_INT_IRQn
#define GPIO_UART_dianji_RX_PORT                                           GPIOB
#define GPIO_UART_dianji_TX_PORT                                           GPIOB
#define GPIO_UART_dianji_RX_PIN                                   DL_GPIO_PIN_16
#define GPIO_UART_dianji_TX_PIN                                   DL_GPIO_PIN_15
#define GPIO_UART_dianji_IOMUX_RX                                (IOMUX_PINCM33)
#define GPIO_UART_dianji_IOMUX_TX                                (IOMUX_PINCM32)
#define GPIO_UART_dianji_IOMUX_RX_FUNC                 IOMUX_PINCM33_PF_UART2_RX
#define GPIO_UART_dianji_IOMUX_TX_FUNC                 IOMUX_PINCM32_PF_UART2_TX
#define UART_dianji_BAUD_RATE                                           (115200)
#define UART_dianji_IBRD_32_MHZ_115200_BAUD                                 (17)
#define UART_dianji_FBRD_32_MHZ_115200_BAUD                                 (23)
/* Defines for UART_Camera */
#define UART_Camera_INST                                                   UART1
#define UART_Camera_INST_FREQUENCY                                      32000000
#define UART_Camera_INST_IRQHandler                             UART1_IRQHandler
#define UART_Camera_INST_INT_IRQN                                 UART1_INT_IRQn
#define GPIO_UART_Camera_RX_PORT                                           GPIOB
#define GPIO_UART_Camera_TX_PORT                                           GPIOB
#define GPIO_UART_Camera_RX_PIN                                    DL_GPIO_PIN_7
#define GPIO_UART_Camera_TX_PIN                                    DL_GPIO_PIN_6
#define GPIO_UART_Camera_IOMUX_RX                                (IOMUX_PINCM24)
#define GPIO_UART_Camera_IOMUX_TX                                (IOMUX_PINCM23)
#define GPIO_UART_Camera_IOMUX_RX_FUNC                 IOMUX_PINCM24_PF_UART1_RX
#define GPIO_UART_Camera_IOMUX_TX_FUNC                 IOMUX_PINCM23_PF_UART1_TX
#define UART_Camera_BAUD_RATE                                           (115200)
#define UART_Camera_IBRD_32_MHZ_115200_BAUD                                 (17)
#define UART_Camera_FBRD_32_MHZ_115200_BAUD                                 (23)





/* Defines for AIN1: GPIOB.0 with pinCMx 12 on package pin 47 */
#define GPIO_TB6612_AIN1_PORT                                            (GPIOB)
#define GPIO_TB6612_AIN1_PIN                                     (DL_GPIO_PIN_0)
#define GPIO_TB6612_AIN1_IOMUX                                   (IOMUX_PINCM12)
/* Defines for AIN2: GPIOB.1 with pinCMx 13 on package pin 48 */
#define GPIO_TB6612_AIN2_PORT                                            (GPIOB)
#define GPIO_TB6612_AIN2_PIN                                     (DL_GPIO_PIN_1)
#define GPIO_TB6612_AIN2_IOMUX                                   (IOMUX_PINCM13)
/* Defines for BIN1: GPIOA.13 with pinCMx 35 on package pin 6 */
#define GPIO_TB6612_BIN1_PORT                                            (GPIOA)
#define GPIO_TB6612_BIN1_PIN                                    (DL_GPIO_PIN_13)
#define GPIO_TB6612_BIN1_IOMUX                                   (IOMUX_PINCM35)
/* Defines for BIN2: GPIOA.12 with pinCMx 34 on package pin 5 */
#define GPIO_TB6612_BIN2_PORT                                            (GPIOA)
#define GPIO_TB6612_BIN2_PIN                                    (DL_GPIO_PIN_12)
#define GPIO_TB6612_BIN2_IOMUX                                   (IOMUX_PINCM34)
/* Defines for Gray_1: GPIOA.30 with pinCMx 5 on package pin 37 */
#define GPIO_Gray_Gray_1_PORT                                            (GPIOA)
#define GPIO_Gray_Gray_1_PIN                                    (DL_GPIO_PIN_30)
#define GPIO_Gray_Gray_1_IOMUX                                    (IOMUX_PINCM5)
/* Defines for Gray_2: GPIOB.19 with pinCMx 45 on package pin 16 */
#define GPIO_Gray_Gray_2_PORT                                            (GPIOB)
#define GPIO_Gray_Gray_2_PIN                                    (DL_GPIO_PIN_19)
#define GPIO_Gray_Gray_2_IOMUX                                   (IOMUX_PINCM45)
/* Defines for Gray_3: GPIOA.24 with pinCMx 54 on package pin 25 */
#define GPIO_Gray_Gray_3_PORT                                            (GPIOA)
#define GPIO_Gray_Gray_3_PIN                                    (DL_GPIO_PIN_24)
#define GPIO_Gray_Gray_3_IOMUX                                   (IOMUX_PINCM54)
/* Defines for Gray_4: GPIOB.25 with pinCMx 56 on package pin 27 */
#define GPIO_Gray_Gray_4_PORT                                            (GPIOB)
#define GPIO_Gray_Gray_4_PIN                                    (DL_GPIO_PIN_25)
#define GPIO_Gray_Gray_4_IOMUX                                   (IOMUX_PINCM56)
/* Defines for Gray_5: GPIOB.24 with pinCMx 52 on package pin 23 */
#define GPIO_Gray_Gray_5_PORT                                            (GPIOB)
#define GPIO_Gray_Gray_5_PIN                                    (DL_GPIO_PIN_24)
#define GPIO_Gray_Gray_5_IOMUX                                   (IOMUX_PINCM52)
/* Defines for Gray_6: GPIOB.20 with pinCMx 48 on package pin 19 */
#define GPIO_Gray_Gray_6_PORT                                            (GPIOB)
#define GPIO_Gray_Gray_6_PIN                                    (DL_GPIO_PIN_20)
#define GPIO_Gray_Gray_6_IOMUX                                   (IOMUX_PINCM48)
/* Defines for Gray_7: GPIOA.14 with pinCMx 36 on package pin 7 */
#define GPIO_Gray_Gray_7_PORT                                            (GPIOA)
#define GPIO_Gray_Gray_7_PIN                                    (DL_GPIO_PIN_14)
#define GPIO_Gray_Gray_7_IOMUX                                   (IOMUX_PINCM36)
/* Defines for Gray_8: GPIOB.18 with pinCMx 44 on package pin 15 */
#define GPIO_Gray_Gray_8_PORT                                            (GPIOB)
#define GPIO_Gray_Gray_8_PIN                                    (DL_GPIO_PIN_18)
#define GPIO_Gray_Gray_8_IOMUX                                   (IOMUX_PINCM44)
/* Port definition for Pin Group GPIO_OLED */
#define GPIO_OLED_PORT                                                   (GPIOA)

/* Defines for SCL: GPIOA.31 with pinCMx 6 on package pin 39 */
#define GPIO_OLED_SCL_PIN                                       (DL_GPIO_PIN_31)
#define GPIO_OLED_SCL_IOMUX                                       (IOMUX_PINCM6)
/* Defines for SDA: GPIOA.28 with pinCMx 3 on package pin 35 */
#define GPIO_OLED_SDA_PIN                                       (DL_GPIO_PIN_28)
#define GPIO_OLED_SDA_IOMUX                                       (IOMUX_PINCM3)
/* Defines for S1: GPIOA.25 with pinCMx 55 on package pin 26 */
#define GPIO_Key_S1_PORT                                                 (GPIOA)
#define GPIO_Key_S1_PIN                                         (DL_GPIO_PIN_25)
#define GPIO_Key_S1_IOMUX                                        (IOMUX_PINCM55)
/* Defines for S2: GPIOA.26 with pinCMx 59 on package pin 30 */
#define GPIO_Key_S2_PORT                                                 (GPIOA)
#define GPIO_Key_S2_PIN                                         (DL_GPIO_PIN_26)
#define GPIO_Key_S2_IOMUX                                        (IOMUX_PINCM59)
/* Defines for S3: GPIOB.23 with pinCMx 51 on package pin 22 */
#define GPIO_Key_S3_PORT                                                 (GPIOB)
#define GPIO_Key_S3_PIN                                         (DL_GPIO_PIN_23)
#define GPIO_Key_S3_IOMUX                                        (IOMUX_PINCM51)
/* Defines for S4: GPIOA.29 with pinCMx 4 on package pin 36 */
#define GPIO_Key_S4_PORT                                                 (GPIOA)
#define GPIO_Key_S4_PIN                                         (DL_GPIO_PIN_29)
#define GPIO_Key_S4_IOMUX                                         (IOMUX_PINCM4)
/* Port definition for Pin Group GPIO_ENCODEA */
#define GPIO_ENCODEA_PORT                                                (GPIOB)

/* Defines for PIN_0: GPIOB.8 with pinCMx 25 on package pin 60 */
// pins affected by this interrupt request:["PIN_0","PIN_3"]
#define GPIO_ENCODEA_INT_IRQN                                   (GPIOB_INT_IRQn)
#define GPIO_ENCODEA_INT_IIDX                   (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define GPIO_ENCODEA_PIN_0_IIDX                              (DL_GPIO_IIDX_DIO8)
#define GPIO_ENCODEA_PIN_0_PIN                                   (DL_GPIO_PIN_8)
#define GPIO_ENCODEA_PIN_0_IOMUX                                 (IOMUX_PINCM25)
/* Defines for PIN_3: GPIOB.9 with pinCMx 26 on package pin 61 */
#define GPIO_ENCODEA_PIN_3_IIDX                              (DL_GPIO_IIDX_DIO9)
#define GPIO_ENCODEA_PIN_3_PIN                                   (DL_GPIO_PIN_9)
#define GPIO_ENCODEA_PIN_3_IOMUX                                 (IOMUX_PINCM26)
/* Port definition for Pin Group GPIO_ENCODEB */
#define GPIO_ENCODEB_PORT                                                (GPIOA)

/* Defines for PIN_1: GPIOA.15 with pinCMx 37 on package pin 8 */
// pins affected by this interrupt request:["PIN_1","PIN_2"]
#define GPIO_ENCODEB_INT_IRQN                                   (GPIOA_INT_IRQn)
#define GPIO_ENCODEB_INT_IIDX                   (DL_INTERRUPT_GROUP1_IIDX_GPIOA)
#define GPIO_ENCODEB_PIN_1_IIDX                             (DL_GPIO_IIDX_DIO15)
#define GPIO_ENCODEB_PIN_1_PIN                                  (DL_GPIO_PIN_15)
#define GPIO_ENCODEB_PIN_1_IOMUX                                 (IOMUX_PINCM37)
/* Defines for PIN_2: GPIOA.16 with pinCMx 38 on package pin 9 */
#define GPIO_ENCODEB_PIN_2_IIDX                             (DL_GPIO_IIDX_DIO16)
#define GPIO_ENCODEB_PIN_2_PIN                                  (DL_GPIO_PIN_16)
#define GPIO_ENCODEB_PIN_2_IOMUX                                 (IOMUX_PINCM38)


/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_MOTOR_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_TIMER_1_init(void);
void SYSCFG_DL_UART_GYRO_init(void);
void SYSCFG_DL_UART_dianji_init(void);
void SYSCFG_DL_UART_Camera_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
