#pragma once

#include "user_io.h"
#include "fans.h"
#include "adc_interface.h"
#include "current_sense_interface.h"
#include "sto.h"
#include "phase_pwm.h"
#include "communication.h"


// void _estack(void);
// void Reset_Handler(void);
// void NMI_Handler(void);
// void HardFault_Handler(void);
// void MemManage_Handler(void);
// void BusFault_Handler(void);
// void UsageFault_Handler(void);
// void SVC_Handler(void);
// void DebugMon_Handler(void);
// void PendSV_Handler(void);

// void SysTick_Handler(void);
void WWDG_IRQHandler(void);                   /* Window WatchDog                             */
void PVD_IRQHandler(void);                    /* PVD through EXTI Line detection             */
void TAMP_STAMP_IRQHandler(void);             /* Tamper and TimeStamps through the EXTI line */
void RTC_WKUP_IRQHandler(void);               /* RTC Wakeup through the EXTI line            */
void FLASH_IRQHandler(void);                  /* FLASH                                       */
void RCC_IRQHandler(void);                    /* RCC                                         */
void EXTI0_IRQHandler(void);                  /* EXTI Line0                                  */
void EXTI1_IRQHandler(void);                  /* EXTI Line1                                  */
void EXTI2_IRQHandler(void);                  /* EXTI Line2                                  */
void EXTI3_IRQHandler(void);                  /* EXTI Line3                                  */
void EXTI4_IRQHandler(void);                  /* EXTI Line4                                  */
void DMA1_Stream0_IRQHandler(void);           /* DMA1 Stream 0                               */
void DMA1_Stream1_IRQHandler(void);           /* DMA1 Stream 1                               */
void DMA1_Stream2_IRQHandler(void);           /* DMA1 Stream 2                               */
void DMA1_Stream3_IRQHandler(void);           /* DMA1 Stream 3                               */
void DMA1_Stream4_IRQHandler(void);           /* DMA1 Stream 4                               */
void DMA1_Stream5_IRQHandler(void);           /* DMA1 Stream 5                               */
void DMA1_Stream6_IRQHandler(void);           /* DMA1 Stream 6                               */
void ADC_IRQHandler(void);                    /* ADC1, ADC2 and ADC3s                        */
void CAN1_TX_IRQHandler(void);                /* CAN1 TX                                     */
void CAN1_RX0_IRQHandler(void);               /* CAN1 RX0                                    */
void CAN1_RX1_IRQHandler(void);               /* CAN1 RX1                                    */
void CAN1_SCE_IRQHandler(void);               /* CAN1 SCE                                    */
void EXTI9_5_IRQHandler(void);                /* External Line[9:5]s                         */
void TIM1_BRK_TIM9_IRQHandler(void);          /* TIM1 Break and TIM9                         */
// void TIM1_UP_TIM10_IRQHandler(void);          /* TIM1 Update and TIM10                       */
void TIM1_TRG_COM_TIM11_IRQHandler(void);     /* TIM1 Trigger and Commutation and TIM11      */
void TIM1_CC_IRQHandler(void);                /* TIM1 Capture Compare                        */
void TIM2_IRQHandler(void);                   /* TIM2                                        */
void TIM3_IRQHandler(void);                   /* TIM3                                        */
void TIM4_IRQHandler(void);                   /* TIM4                                        */
// void I2C1_EV_IRQHandler(void);                /* I2C1 Event                                  */
void I2C1_ER_IRQHandler(void);                /* I2C1 Error                                  */
void I2C2_EV_IRQHandler(void);                /* I2C2 Event                                  */
void I2C2_ER_IRQHandler(void);                /* I2C2 Error                                  */
void SPI1_IRQHandler(void);                   /* SPI1                                        */
void SPI2_IRQHandler(void);                   /* SPI2                                        */
void USART1_IRQHandler(void);                 /* USART1                                      */
void USART2_IRQHandler(void);                 /* USART2                                      */
void USART3_IRQHandler(void);                 /* USART3                                      */
void EXTI15_10_IRQHandler(void);              /* External Line[15:10]s                       */
void RTC_Alarm_IRQHandler(void);              /* RTC Alarm (A and B) through EXTI Line       */
void OTG_FS_WKUP_IRQHandler(void);            /* USB OTG FS Wakeup through EXTI line         */
void TIM8_BRK_TIM12_IRQHandler(void);         /* TIM8 Break and TIM12                        */
void TIM8_UP_TIM13_IRQHandler(void);          /* TIM8 Update and TIM13                       */
void TIM8_TRG_COM_TIM14_IRQHandler(void);     /* TIM8 Trigger and Commutation and TIM14      */
void TIM8_CC_IRQHandler(void);                /* TIM8 Capture Compare                        */
void DMA1_Stream7_IRQHandler(void);           /* DMA1 Stream7                                */
void FSMC_IRQHandler(void);                   /* FSMC                                        */
void SDIO_IRQHandler(void);                   /* SDIO                                        */
void TIM5_IRQHandler(void);                   /* TIM5                                        */
void SPI3_IRQHandler(void);                   /* SPI3                                        */
void UART4_IRQHandler(void);                  /* UART4                                       */
void UART5_IRQHandler(void);                  /* UART5                                       */
void TIM6_DAC_IRQHandler(void);               /* TIM6, DAC1 and DAC2                         */
void TIM7_IRQHandler(void);                   /* TIM7                                        */
// void DMA2_Stream0_IRQHandler(void);           /* DMA2 Stream 0                               */
// void DMA2_Stream1_IRQHandler(void);           /* DMA2 Stream 1                               */
void DMA2_Stream2_IRQHandler(void);           /* DMA2 Stream 2                               */
void DMA2_Stream3_IRQHandler(void);           /* DMA2 Stream 3                               */
void DMA2_Stream4_IRQHandler(void);           /* DMA2 Stream 4                               */
void DFSDM1_FLT0_IRQHandler(void);            /* DFSDM1 Filter0                              */
void DFSDM1_FLT1_IRQHandler(void);            /* DFSDM1 Filter1                              */
void CAN2_TX_IRQHandler(void);                /* CAN2 TX                                     */
void CAN2_RX0_IRQHandler(void);               /* CAN2 RX0                                    */
void CAN2_RX1_IRQHandler(void);               /* CAN2 RX1                                    */
void CAN2_SCE_IRQHandler(void);               /* CAN2 SCE                                    */
void OTG_FS_IRQHandler(void);                 /* USB OTG FS                                  */
void DMA2_Stream5_IRQHandler(void);           /* DMA2 Stream 5                               */
void DMA2_Stream6_IRQHandler(void);           /* DMA2 Stream 6                               */
void DMA2_Stream7_IRQHandler(void);           /* DMA2 Stream 7                               */
// void USART6_IRQHandler(void);                 /* USART6                                      */
void I2C3_EV_IRQHandler(void);                /* I2C3 event                                  */
void I2C3_ER_IRQHandler(void);                /* I2C3 error                                  */
void CAN3_TX_IRQHandler(void);                /* CAN3 TX                                     */
void CAN3_RX0_IRQHandler(void);               /* CAN3 RX0                                    */
void CAN3_RX1_IRQHandler(void);               /* CAN3 RX1                                    */
void CAN3_SCE_IRQHandler(void);               /* CAN3 SCE                                    */
void RNG_IRQHandler(void);                    /* RNG                                         */
void FPU_IRQHandler(void);                    /* FPU                                         */
void UART7_IRQHandler(void);                  /* UART7                                       */
void UART8_IRQHandler(void);                  /* UART8                                       */
void SPI4_IRQHandler(void);                   /* SPI4                                        */
void SPI5_IRQHandler(void);                   /* SPI5                                        */
void SAI1_IRQHandler(void);                   /* SAI1                                        */
void UART9_IRQHandler(void);                  /* UART9                                       */
void UART10_IRQHandler(void);                 /* UART10                                      */
void QUADSPI_IRQHandler(void);                /* QuadSPI                                     */
void FMPI2C1_EV_IRQHandler(void);             /* FMPI2C1 Event                               */
void FMPI2C1_ER_IRQHandler(void);             /* FMPI2C1 Error                               */
void LPTIM1_IRQHandler(void);                 /* LPTIM1                                      */
void DFSDM2_FLT0_IRQHandler(void);            /* DFSDM2 Filter0                              */
void DFSDM2_FLT1_IRQHandler(void);            /* DFSDM2 Filter1                              */
void DFSDM2_FLT2_IRQHandler(void);            /* DFSDM2 Filter2                              */
void DFSDM2_FLT3_IRQHandler(void);            /* DFSDM2 Filter3                              */