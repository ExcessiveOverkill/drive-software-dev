#include "device.h"


__attribute__((weak)) void device::_estack(void){missed_irq_handler(IRQ::MISSED_IRQ__estack);}
__attribute__((weak)) void device::Reset_Handler(void){missed_irq_handler(IRQ::MISSED_IRQ_Reset_Handler);}
__attribute__((weak)) void device::NMI_Handler(void){missed_irq_handler(IRQ::MISSED_IRQ_NMI_Handler);}
__attribute__((weak)) void device::HardFault_Handler(void){missed_irq_handler(IRQ::MISSED_IRQ_HardFault_Handler);}
__attribute__((weak)) void device::MemManage_Handler(void){missed_irq_handler(IRQ::MISSED_IRQ_MemManage_Handler);}
__attribute__((weak)) void device::BusFault_Handler(void){missed_irq_handler(IRQ::MISSED_IRQ_BusFault_Handler);}
__attribute__((weak)) void device::UsageFault_Handler(void){missed_irq_handler(IRQ::MISSED_IRQ_UsageFault_Handler);}
__attribute__((weak)) void device::SVC_Handler(void){missed_irq_handler(IRQ::MISSED_IRQ_SVC_Handler);}
__attribute__((weak)) void device::DebugMon_Handler(void){missed_irq_handler(IRQ::MISSED_IRQ_DebugMon_Handler);}
__attribute__((weak)) void device::PendSV_Handler(void){missed_irq_handler(IRQ::MISSED_IRQ_PendSV_Handler);}
__attribute__((weak)) void device::SysTick_Handler(void){missed_irq_handler(IRQ::MISSED_IRQ_SysTick_Handler);}
__attribute__((weak)) void device::WWDG_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_WWDG_IRQHandler);}
__attribute__((weak)) void device::PVD_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_PVD_IRQHandler);}
__attribute__((weak)) void device::TAMP_STAMP_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_TAMP_STAMP_IRQHandler);}
__attribute__((weak)) void device::RTC_WKUP_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_RTC_WKUP_IRQHandler);}
__attribute__((weak)) void device::FLASH_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_FLASH_IRQHandler);}
__attribute__((weak)) void device::RCC_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_RCC_IRQHandler);}
__attribute__((weak)) void device::EXTI0_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_EXTI0_IRQHandler);}
__attribute__((weak)) void device::EXTI1_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_EXTI1_IRQHandler);}
__attribute__((weak)) void device::EXTI2_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_EXTI2_IRQHandler);}
__attribute__((weak)) void device::EXTI3_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_EXTI3_IRQHandler);}
__attribute__((weak)) void device::EXTI4_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_EXTI4_IRQHandler);}
__attribute__((weak)) void device::DMA1_Stream0_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DMA1_Stream0_IRQHandler);}
__attribute__((weak)) void device::DMA1_Stream1_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DMA1_Stream1_IRQHandler);}
__attribute__((weak)) void device::DMA1_Stream2_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DMA1_Stream2_IRQHandler);}
__attribute__((weak)) void device::DMA1_Stream3_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DMA1_Stream3_IRQHandler);}
__attribute__((weak)) void device::DMA1_Stream4_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DMA1_Stream4_IRQHandler);}
__attribute__((weak)) void device::DMA1_Stream5_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DMA1_Stream5_IRQHandler);}
__attribute__((weak)) void device::DMA1_Stream6_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DMA1_Stream6_IRQHandler);}
__attribute__((weak)) void device::ADC_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_ADC_IRQHandler);}
__attribute__((weak)) void device::CAN1_TX_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_CAN1_TX_IRQHandler);}
__attribute__((weak)) void device::CAN1_RX0_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_CAN1_RX0_IRQHandler);}
__attribute__((weak)) void device::CAN1_RX1_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_CAN1_RX1_IRQHandler);}
__attribute__((weak)) void device::CAN1_SCE_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_CAN1_SCE_IRQHandler);}
__attribute__((weak)) void device::EXTI9_5_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_EXTI9_5_IRQHandler);}
__attribute__((weak)) void device::TIM1_BRK_TIM9_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_TIM1_BRK_TIM9_IRQHandler);}
__attribute__((weak)) void device::TIM1_UP_TIM10_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_TIM1_UP_TIM10_IRQHandler);}
__attribute__((weak)) void device::TIM1_TRG_COM_TIM11_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_TIM1_TRG_COM_TIM11_IRQHandler);}
__attribute__((weak)) void device::TIM1_CC_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_TIM1_CC_IRQHandler);}
__attribute__((weak)) void device::TIM2_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_TIM2_IRQHandler);}
__attribute__((weak)) void device::TIM3_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_TIM3_IRQHandler);}
__attribute__((weak)) void device::TIM4_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_TIM4_IRQHandler);}
__attribute__((weak)) void device::I2C1_EV_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_I2C1_EV_IRQHandler);}
__attribute__((weak)) void device::I2C1_ER_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_I2C1_ER_IRQHandler);}
__attribute__((weak)) void device::I2C2_EV_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_I2C2_EV_IRQHandler);}
__attribute__((weak)) void device::I2C2_ER_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_I2C2_ER_IRQHandler);}
__attribute__((weak)) void device::SPI1_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_SPI1_IRQHandler);}
__attribute__((weak)) void device::SPI2_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_SPI2_IRQHandler);}
__attribute__((weak)) void device::USART1_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_USART1_IRQHandler);}
__attribute__((weak)) void device::USART2_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_USART2_IRQHandler);}
__attribute__((weak)) void device::USART3_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_USART3_IRQHandler);}
__attribute__((weak)) void device::EXTI15_10_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_EXTI15_10_IRQHandler);}
__attribute__((weak)) void device::RTC_Alarm_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_RTC_Alarm_IRQHandler);}
__attribute__((weak)) void device::OTG_FS_WKUP_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_OTG_FS_WKUP_IRQHandler);}
__attribute__((weak)) void device::TIM8_BRK_TIM12_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_TIM8_BRK_TIM12_IRQHandler);}
__attribute__((weak)) void device::TIM8_UP_TIM13_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_TIM8_UP_TIM13_IRQHandler);}
__attribute__((weak)) void device::TIM8_TRG_COM_TIM14_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_TIM8_TRG_COM_TIM14_IRQHandler);}
__attribute__((weak)) void device::TIM8_CC_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_TIM8_CC_IRQHandler);}
__attribute__((weak)) void device::DMA1_Stream7_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DMA1_Stream7_IRQHandler);}
__attribute__((weak)) void device::FSMC_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_FSMC_IRQHandler);}
__attribute__((weak)) void device::SDIO_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_SDIO_IRQHandler);}
__attribute__((weak)) void device::TIM5_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_TIM5_IRQHandler);}
__attribute__((weak)) void device::SPI3_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_SPI3_IRQHandler);}
__attribute__((weak)) void device::UART4_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_UART4_IRQHandler);}
__attribute__((weak)) void device::UART5_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_UART5_IRQHandler);}
__attribute__((weak)) void device::TIM6_DAC_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_TIM6_DAC_IRQHandler);}
__attribute__((weak)) void device::TIM7_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_TIM7_IRQHandler);}
__attribute__((weak)) void device::DMA2_Stream0_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DMA2_Stream0_IRQHandler);}
__attribute__((weak)) void device::DMA2_Stream1_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DMA2_Stream1_IRQHandler);}
__attribute__((weak)) void device::DMA2_Stream2_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DMA2_Stream2_IRQHandler);}
__attribute__((weak)) void device::DMA2_Stream3_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DMA2_Stream3_IRQHandler);}
__attribute__((weak)) void device::DMA2_Stream4_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DMA2_Stream4_IRQHandler);}
__attribute__((weak)) void device::DFSDM1_FLT0_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DFSDM1_FLT0_IRQHandler);}
__attribute__((weak)) void device::DFSDM1_FLT1_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DFSDM1_FLT1_IRQHandler);}
__attribute__((weak)) void device::CAN2_TX_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_CAN2_TX_IRQHandler);}
__attribute__((weak)) void device::CAN2_RX0_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_CAN2_RX0_IRQHandler);}
__attribute__((weak)) void device::CAN2_RX1_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_CAN2_RX1_IRQHandler);}
__attribute__((weak)) void device::CAN2_SCE_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_CAN2_SCE_IRQHandler);}
__attribute__((weak)) void device::OTG_FS_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_OTG_FS_IRQHandler);}
__attribute__((weak)) void device::DMA2_Stream5_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DMA2_Stream5_IRQHandler);}
__attribute__((weak)) void device::DMA2_Stream6_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DMA2_Stream6_IRQHandler);}
__attribute__((weak)) void device::DMA2_Stream7_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DMA2_Stream7_IRQHandler);}
__attribute__((weak)) void device::USART6_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_USART6_IRQHandler);}
__attribute__((weak)) void device::I2C3_EV_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_I2C3_EV_IRQHandler);}
__attribute__((weak)) void device::I2C3_ER_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_I2C3_ER_IRQHandler);}
__attribute__((weak)) void device::CAN3_TX_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_CAN3_TX_IRQHandler);}
__attribute__((weak)) void device::CAN3_RX0_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_CAN3_RX0_IRQHandler);}
__attribute__((weak)) void device::CAN3_RX1_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_CAN3_RX1_IRQHandler);}
__attribute__((weak)) void device::CAN3_SCE_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_CAN3_SCE_IRQHandler);}
__attribute__((weak)) void device::RNG_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_RNG_IRQHandler);}
__attribute__((weak)) void device::FPU_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_FPU_IRQHandler);}
__attribute__((weak)) void device::UART7_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_UART7_IRQHandler);}
__attribute__((weak)) void device::UART8_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_UART8_IRQHandler);}
__attribute__((weak)) void device::SPI4_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_SPI4_IRQHandler);}
__attribute__((weak)) void device::SPI5_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_SPI5_IRQHandler);}
__attribute__((weak)) void device::SAI1_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_SAI1_IRQHandler);}
__attribute__((weak)) void device::UART9_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_UART9_IRQHandler);}
__attribute__((weak)) void device::UART10_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_UART10_IRQHandler);}
__attribute__((weak)) void device::QUADSPI_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_QUADSPI_IRQHandler);}
__attribute__((weak)) void device::FMPI2C1_EV_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_FMPI2C1_EV_IRQHandler);}
__attribute__((weak)) void device::FMPI2C1_ER_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_FMPI2C1_ER_IRQHandler);}
__attribute__((weak)) void device::LPTIM1_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_LPTIM1_IRQHandler);}
__attribute__((weak)) void device::DFSDM2_FLT0_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DFSDM2_FLT0_IRQHandler);}
__attribute__((weak)) void device::DFSDM2_FLT1_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DFSDM2_FLT1_IRQHandler);}
__attribute__((weak)) void device::DFSDM2_FLT2_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DFSDM2_FLT2_IRQHandler);}
__attribute__((weak)) void device::DFSDM2_FLT3_IRQHandler(void){missed_irq_handler(IRQ::MISSED_IRQ_DFSDM2_FLT3_IRQHandler);}