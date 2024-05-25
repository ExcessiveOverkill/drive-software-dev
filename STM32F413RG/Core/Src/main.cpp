#include "main.h"

#define ADCCLK		20		// (MHz) ADC output clock
#define DEADTIME	100		// (ns)  PWM Deadtime (ns)
#define PWMCLK		25		// (KHz) PWM Output Frequency

#define PWM_ticks 1000*SYSCLK/PWMCLK/2

#define ADC_VOLTAGE 0.250			// (V) ADC optimal shunt input voltage
#define ADC_MIN_VALUE_PERC 0.1094
#define DFSDM_DIVISIONS 8388608
#define DFSDM_FOSR (1000*ADCCLK/PWMCLK/2 - 2)

#define VBUS 12
#define SHUNT_RESISTANCE 0.005	// (Ohms) Current shunt resistance


float Kp = .1;		// Proportional gain for current controller (V/A)
float Ki = .1;		// Integral gain for current controller (V/A/s)
float I_term_limit = 3;


float requested_Iq = 3.0;
float requested_Id = 0.0;
float electrical_rads_per_second = 10.0;

float theta = 0.0;

// DO NOT MODIFY THESE OUTSIDE OF ISR FUNCTION
uint32_t CAPTURE_COMP_U = PWM_ticks * 0.5;
uint32_t CAPTURE_COMP_V = PWM_ticks * 0.5;
uint32_t CAPTURE_COMP_W = PWM_ticks * 0.5;
float I_Term = 0;	// Integral term for current controller (V)

// TODO: Remove these...
uint8_t I2C_address = 0x20;
uint8_t I2C_Register_Address = 0b10100000;
uint8_t* I2C_data = &I2C_Register_Address;
uint8_t I2C_data_size = 1;

extern "C" {

void NMI_Handler(void){
	while(1);
}   

void HardFault_Handler(void){
	while(1);
}   

void MemManage_Handler(void){
	while(1);
}   

void BusFault_Handler(void){
	while(1);
}   

void UsageFault_Handler(void){
	while(1);
}   

void SVC_Handler(void){
	while(1);
}   

void DebugMon_Handler(void){
	while(1);
}   

void PendSV_Handler(void){
	while(1);
}   

// void SysTick_Handler(void){
// 	while(1);
// }   

void WWDG_IRQHandler(void){
	while(1);
}   

void PVD_IRQHandler(void){
	while(1);
}   

void TAMP_STAMP_IRQHandler(void){
	while(1);
}   

void RTC_WKUP_IRQHandler(void){
	while(1);
}   

void FLASH_IRQHandler(void){
	while(1);
}   

void RCC_IRQHandler(void){
	while(1);
}   

void EXTI0_IRQHandler(void){
	while(1);
}   

void EXTI1_IRQHandler(void){
	while(1);
}   

void EXTI2_IRQHandler(void){
	while(1);
}   

void EXTI3_IRQHandler(void){
	while(1);
}   

void EXTI4_IRQHandler(void){
	while(1);
}   

void DMA1_Stream0_IRQHandler(void){
	while(1);
}   

void DMA1_Stream1_IRQHandler(void){
	while(1);
}   

void DMA1_Stream2_IRQHandler(void){
	while(1);
}   

void DMA1_Stream3_IRQHandler(void){
	while(1);
}   

void DMA1_Stream4_IRQHandler(void){
	while(1);
}   

void DMA1_Stream5_IRQHandler(void){
	while(1);
}   

void DMA1_Stream6_IRQHandler(void){
	while(1);
}   

void ADC_IRQHandler(void){
	while(1);
}   

void CAN1_TX_IRQHandler(void){
	while(1);
}   

void CAN1_RX0_IRQHandler(void){
	while(1);
}   

void CAN1_RX1_IRQHandler(void){
	while(1);
}   

void CAN1_SCE_IRQHandler(void){
	while(1);
}   

void EXTI9_5_IRQHandler(void){
	while(1);
}   

void TIM1_BRK_TIM9_IRQHandler(void){
	while(1);
}   

// void TIM1_UP_TIM10_IRQHandler(void){
// 	while(1);
// }   

void TIM1_TRG_COM_TIM11_IRQHandler(void){
	while(1);
}   

void TIM1_CC_IRQHandler(void){
	while(1);
}   

void TIM2_IRQHandler(void){
	while(1);
}   

void TIM3_IRQHandler(void){
	while(1);
}   

void TIM4_IRQHandler(void){
	while(1);
}   

// void I2C1_EV_IRQHandler(void){
// 	while(1);
// }   

// void I2C1_ER_IRQHandler(void){
// 	while(1);
// }   

void I2C2_EV_IRQHandler(void){
	while(1);
}   

void I2C2_ER_IRQHandler(void){
	while(1);
}   

void SPI1_IRQHandler(void){
	while(1);
}   

void SPI2_IRQHandler(void){
	while(1);
}   

void USART1_IRQHandler(void){
	while(1);
}   

void USART2_IRQHandler(void){
	while(1);
}   

void USART3_IRQHandler(void){
	while(1);
}   

void EXTI15_10_IRQHandler(void){
	while(1);
}   

void RTC_Alarm_IRQHandler(void){
	while(1);
}   

void OTG_FS_WKUP_IRQHandler(void){
	while(1);
}   

void TIM8_BRK_TIM12_IRQHandler(void){
	while(1);
}   

void TIM8_UP_TIM13_IRQHandler(void){
	while(1);
}   

void TIM8_TRG_COM_TIM14_IRQHandler(void){
	while(1);
}   

void TIM8_CC_IRQHandler(void){
	while(1);
}   

void DMA1_Stream7_IRQHandler(void){
	while(1);
}   

void FSMC_IRQHandler(void){
	while(1);
}   

void SDIO_IRQHandler(void){
	while(1);
}   

void TIM5_IRQHandler(void){
	while(1);
}   

void SPI3_IRQHandler(void){
	while(1);
}   

void UART4_IRQHandler(void){
	while(1);
}   

void UART5_IRQHandler(void){
	while(1);
}   

void TIM6_DAC_IRQHandler(void){
	while(1);
}   

void TIM7_IRQHandler(void){
	while(1);
}   

void DMA2_Stream0_IRQHandler(void){
	while(1);
}   

void DMA2_Stream1_IRQHandler(void){
	while(1);
}   

void DMA2_Stream2_IRQHandler(void){
	while(1);
}   

void DMA2_Stream3_IRQHandler(void){
	while(1);
}   

void DMA2_Stream4_IRQHandler(void){
	while(1);
}   

void DFSDM1_FLT0_IRQHandler(void){
	while(1);
}   

void DFSDM1_FLT1_IRQHandler(void){
	while(1);
}   

void CAN2_TX_IRQHandler(void){
	while(1);
}   

void CAN2_RX0_IRQHandler(void){
	while(1);
}   

void CAN2_RX1_IRQHandler(void){
	while(1);
}   

void CAN2_SCE_IRQHandler(void){
	while(1);
}   

void OTG_FS_IRQHandler(void){
	while(1);
}   

void DMA2_Stream5_IRQHandler(void){
	while(1);
}   

void DMA2_Stream6_IRQHandler(void){
	while(1);
}   

void DMA2_Stream7_IRQHandler(void){
	while(1);
}   

void USART6_IRQHandler(void){
	while(1);
}   

void I2C3_EV_IRQHandler(void){
	while(1);
}   

void I2C3_ER_IRQHandler(void){
	while(1);
}   

void CAN3_TX_IRQHandler(void){
	while(1);
}   

void CAN3_RX0_IRQHandler(void){
	while(1);
}   

void CAN3_RX1_IRQHandler(void){
	while(1);
}   

void CAN3_SCE_IRQHandler(void){
	while(1);
}   

void RNG_IRQHandler(void){
	while(1);
}   

void FPU_IRQHandler(void){
	while(1);
}   

void UART7_IRQHandler(void){
	while(1);
}   

void UART8_IRQHandler(void){
	while(1);
}   

void SPI4_IRQHandler(void){
	while(1);
}   

void SPI5_IRQHandler(void){
	while(1);
}   

void SAI1_IRQHandler(void){
	while(1);
}   

void UART9_IRQHandler(void){
	while(1);
}   

void UART10_IRQHandler(void){
	while(1);
}   

void QUADSPI_IRQHandler(void){
	while(1);
}   

void FMPI2C1_EV_IRQHandler(void){
	while(1);
}   

void FMPI2C1_ER_IRQHandler(void){
	while(1);
}   

void LPTIM1_IRQHandler(void){
	while(1);
}   

void DFSDM2_FLT0_IRQHandler(void){
	while(1);
}   

void DFSDM2_FLT1_IRQHandler(void){
	while(1);
}   

void DFSDM2_FLT2_IRQHandler(void){
	while(1);
}   

void DFSDM2_FLT3_IRQHandler(void){
	while(1);
}   
}

void delay_us(uint32_t time_us);

void CPU_init(void);

void I2C1_init(void);
void I2C1_Start_Transmit(uint8_t address, uint8_t* data, uint8_t size) ;

void I2C1_Event_Interrupt_Handler(void);
void I2C1_Error_Interrupt_Handler(void);
void Start_Bit_Sent_Callback(void);
void Address_Sent_Callback(void);
void Ten_Bit_Header_Sent_Callback(void);
void Stop_Received_Callback(void);
void Data_Byte_Transfer_Finished_Callback(void);
void Receive_Buffer_Not_Empty_Callback(void);
void Transmit_Buffer_Empty_Callback(void);
void Bus_Error_Callback(void);
void Arbitration_Loss_Callback(void);
void Acknowledge_Failure_Callback(void);
void Overrun_Underrun_Callback(void);
void PEC_Error_Callback(void);
void Timeout_Tlow_Error_Callback(void);
void SMBus_Alert_Callback(void);

void USART6_init(void);
void USART6_Error_Interrupt_Handler(void);
void Parity_Error_Callback(void);
void Framing_Error_Callback(void);
void Noise_Detected_Error_Callback(void);
void Overrun_Error_Callback(void);

void TIM1_init(void);

void TIM2_init(void);
void PWM_enable(void);

void DFSDM2_init(void);

int Clarke_and_Park_Transform(float theta, float A, float B, float C, float *D, float *Q);
int Inverse_Carke_and_Park_Transform(float theta, float D, float Q, float *A, float *B, float *C);

user_io userIO(1000/SYSTICK_FREQUENCY);

int main(void){

	CPU_init();
	// GPIO_init();
	// TIM1_init();
	TIM2_init();
	// DFSDM2_init();
	// PWM_enable();
	//I2C1_init();

	SysTick->LOAD = (SYSCLK*1000000/8) / SYSTICK_FREQUENCY;
	
	NVIC_EnableIRQ(SysTick_IRQn);

	SysTick->CTRL = 0b11;	// enable counter and exception

	delay_us(1000);

	userIO.init();

	while(1){

		userIO.set_led_state(userIO.get_switch_states(), userIO.blink_fast);
		userIO.set_led_state(~userIO.get_switch_states(), userIO.off);

	}
}

void delay_us(uint32_t time_us) {
	// blocking delay up to 42949672 us
	 TIM2->CNT = 0;
	 while (TIM2->CNT < time_us*SYSCLK);
}

void CPU_init(void){


	FLASH->ACR |= FLASH_ACR_ICEN			// Enable intruction cache
			    | FLASH_ACR_DCEN 			// Enable Date Cache
			    | FLASH_ACR_PRFTEN 			// Enable prefetch
			    | FLASH_ACR_LATENCY_3WS;	// Set Flash latency to 3 wait states

	RCC->PLLCFGR = (8 << 0)    // Set PLL_M to 8. The input clock frequency is divided by this value.
	             | (SYSCLK << 6)  // Set PLL_N, the multiplication factor for the PLL. SYSCLK is presumably defined elsewhere, representing the desired system clock frequency.
	             | (0 << 16)   // Set PLL_P to 2 (0 in register corresponds to PLL_P = 2). The PLL output frequency is divided by this value to get the system clock.
	             | (4 << 24);  // Set PLL_Q to 4. This value is used for USB, SDIO, and random number generator clocks, but it's not critical for the main system clock (SYSCLK).

	// Turn on the PLL and wait for it to become stable
	RCC->CR |= RCC_CR_PLLON;  // Enable the PLL
	while (!(RCC->CR & RCC_CR_PLLRDY)); // Wait for PLL to be ready (PLL ready flag)

	// Switch the system clock source to the PLL
	RCC->CFGR &= ~RCC_CFGR_SW;  // Clear the clock switch bits
	RCC->CFGR |= RCC_CFGR_SW_PLL;  // Set the clock source to PLL
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait until PLL is used as the system clock source

	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;	// divide by 2 to get 50Mhz for APB1 peripherals

	// Update the SystemCoreClock variable to the new clock speed
	SystemCoreClockUpdate(); // Update the SystemCoreClock global variable with the new clock frequency
	
	SystemInit();	// Enable FPU
}

extern "C" {
void SysTick_Handler(void){		// called at SYSTICK_FREQUENCY
	userIO.update();
}   

void I2C1_EV_IRQHandler(void){
	userIO.I2C1_Event_Interrupt();
}

void I2C1_Error_Interrupt_Handler(void){

	if (I2C1->SR1 & I2C_SR1_BERR) {
        Bus_Error_Callback();
        I2C1->SR1 &= ~I2C_SR1_BERR; // Clear the interrupt flag
    }
    if (I2C1->SR1 & I2C_SR1_ARLO) {
        Arbitration_Loss_Callback();
        I2C1->SR1 &= ~I2C_SR1_ARLO; // Clear the interrupt flag
    }
    if (I2C1->SR1 & I2C_SR1_AF) {
        Acknowledge_Failure_Callback();
        I2C1->SR1 &= ~I2C_SR1_AF; // Clear the interrupt flag
    }
    if (I2C1->SR1 & I2C_SR1_OVR) {
        Overrun_Underrun_Callback();
        I2C1->SR1 &= ~I2C_SR1_OVR; // Clear the interrupt flag
    }
    if (I2C1->SR1 & I2C_SR1_PECERR) {
        PEC_Error_Callback();
        I2C1->SR1 &= ~I2C_SR1_PECERR; // Clear the interrupt flag
    }
    if (I2C1->SR1 & I2C_SR1_TIMEOUT) {
        Timeout_Tlow_Error_Callback();
        I2C1->SR1 &= ~I2C_SR1_TIMEOUT; // Clear the interrupt flag
    }
    if (I2C1->SR1 & I2C_SR1_SMBALERT) {
        SMBus_Alert_Callback();
        I2C1->SR1 &= ~I2C_SR1_SMBALERT; // Clear the interrupt flag
    }
}

void USART6_init(void){
	USART6->CR1 |= USART_CR1_UE;	// Enable USART1
	USART6->CR3 |= USART_CR3_DMAR;	// Enable DMA
	USART6->BRR |= USART_BRR_DIV_Mantissa;	// Program Baud rate Mantissa
	USART6->BRR |= USART_BRR_DIV_Fraction;	// Program Baud rate fraction
	USART6->CR1 |= USART_CR1_RE;	// Enable Receiver

}

void USART6_Error_Interrupt_Handler(void){

	if (USART6->SR & USART_SR_PE) {
        Parity_Error_Callback();
		// Cleared by read or write to USART_DR_DR
    }
	if (USART6->SR & USART_SR_FE) {
        Framing_Error_Callback();
		// Cleared by reading USART_DR_DR 
    }
	if (USART6->SR & USART_SR_NE) {
        Noise_Detected_Error_Callback();
		// Cleared by reading USART_DR_DR 
    }
	if (USART6->SR & USART_SR_ORE) {
        Overrun_Error_Callback();
		// Cleared by reading USART_DR_DR 
    }
}


void TIM1_init(void){

	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;		// Enable TIM1   Clock

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// Enable GPIOA  Clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    // Enable GPIOB  Clock

	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER8)  | GPIO_MODER_MODER8_1;	// Set UH  (PA8)  to AF mode
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER9)  | GPIO_MODER_MODER9_1;	// Set VH  (PA9)  to AF mode
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER10) | GPIO_MODER_MODER10_1;	// Set WH  (PA10) to AF mode
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER13) | GPIO_MODER_MODER13_1;	// Set UL  (PB13) to AF mode
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER14) | GPIO_MODER_MODER14_1;	// Set VL  (PB14) to AF mode
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER15) | GPIO_MODER_MODER15_1;	// Set WL  (PB15) to AF mode
	GPIOB->MODER = (GPIOB->MODER & ~GPIO_MODER_MODER12) | GPIO_MODER_MODER12_1;	// Set STO (PB12) to AF mode


	GPIOA->AFR[1] |= 1 << (4 * (8 - 8));	// Set UH  (PA8)  AF to TIM1_CH1
	GPIOA->AFR[1] |= 1 << (4 * (9 - 8));	// Set VH  (PA9)  AF to TIM1_CH2
	GPIOA->AFR[1] |= 1 << (4 * (10 - 8));	// Set WH  (PA10) AF to TIM1_CH3
	GPIOB->AFR[1] |= 1 << (4 * (13 - 8));	// Set UL  (PB13) AF to TIM1_CH1N
	GPIOB->AFR[1] |= 1 << (4 * (14 - 8));	// Set VL  (PB14) AF to TIM1_CH2N
	GPIOB->AFR[1] |= 1 << (4 * (15 - 8));	// Set WL  (PB15) AF to TIM1_CH3N
	GPIOB->AFR[1] |= 1 << (4 * (12 - 8));	// Set STO (PB12) AF to TIM1_BKIN


	uint32_t deadtime_ticks = (DEADTIME * SYSCLK) / 1000;
	TIM1->BDTR |= deadtime_ticks & TIM_BDTR_DTG;
	TIM1->BDTR |= TIM_BDTR_AOE;
	TIM1->BDTR |= TIM_BDTR_BKP;
	__NOP();__NOP();__NOP();		// Inserts a delay of 3 clock cycles
	TIM1->BDTR |= TIM_BDTR_BKE;		// Enable Brake (Safe Torque Off)
	__NOP();__NOP();__NOP();		// Inserts a delay of 3 clock cycles

	TIM1->CR1 |= TIM_CR1_CMS;     	// Set center-aligned mode
	TIM1->CR1 |= TIM_CR1_ARPE;		// Enable Auto-reload preload
	TIM1->ARR = 500*SYSCLK/PWMCLK;  // Auto-reload value for PWM frequency

	TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // PWM mode 1 on Channel 1
	TIM1->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // PWM mode 1 on Channel 2
	TIM1->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // PWM mode 1 on Channel 3

	TIM1->CCER |= TIM_CCER_CC1E | TIM_CCER_CC1NE; // Enable CH1 and CH1N
	TIM1->CCER |= TIM_CCER_CC2E | TIM_CCER_CC2NE; // Enable CH1 and CH1N
	TIM1->CCER |= TIM_CCER_CC3E | TIM_CCER_CC3NE; // Enable CH1 and CH1N

	TIM1->DIER |= TIM_DIER_UIE; // Enable Update Interrupt
	
	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn);
	// NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1); // Set priority as needed
	// __enable_irq();
}

void TIM2_init(){

	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;		// Enable TIM2   Clock

	TIM2->PSC = 0; // Prescaler
	TIM2->ARR = 0xFFFFFFFF;  // Max auto-reload value
	TIM2->CR1 |= TIM_CR1_CEN; // Enable TIM2
}


void PWM_enable(){

	TIM1->CCR1 = PWM_ticks * 0.5;
	TIM1->CCR2 = PWM_ticks * 0.5;
	TIM1->CCR3 = PWM_ticks * 0.5;

	TIM1->CNT = 0;					// Resest PWM counter
	TIM1->CR1 |= TIM_CR1_CEN;		// Enable PWM timer TIM1
	TIM1->BDTR |= TIM_BDTR_MOE;     // Main output enable (for advanced timers)

	// Start DFSDM ADC conversions
	DFSDM2_Filter1->FLTCR1 |= DFSDM_FLTCR1_RSWSTART;	// Start Filter 1 Conversion
	DFSDM2_Filter2->FLTCR1 |= DFSDM_FLTCR1_RSWSTART;	// Start Filter 2 Conversion
	DFSDM2_Filter3->FLTCR1 |= DFSDM_FLTCR1_RSWSTART;	// Start Filter 3 Conversion
}


void DFSDM2_init(){

	RCC->APB2ENR |= RCC_APB2ENR_DFSDM2EN;   // Enable DFSDM2 Clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    // Enable GPIOB  Clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;    // Enable GPIOC  Clock

	// GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED10_1;

	GPIOB->MODER |= (2 << (2 * 9));				// Set U_Data (PB9) to AF mode
	GPIOC->MODER |= (2 << (2 * 5));				// Set V_Data (PC5) to AF mode
	GPIOC->MODER |= (2 << (2 * 9));				// Set W_Data (PC9) to AF mode
	GPIOB->MODER |= (2 << (2 * 10));			// Set ADC_CLK (PB10) to AF mode
	GPIOB->AFR[1] |= (6  << (4 * (9  - 8)));	// Set PB9  AF to DFSDM2_ DATIN1
	GPIOC->AFR[0] |= (3  << (4 * (5  - 0)));	// Set PC5  AF to DFSDM2_ DATIN2
	GPIOC->AFR[1] |= (7  << (4 * (9  - 8)));	// Set PC9  AF to DFSDM2_ DATIN3
	GPIOB->AFR[1] |= (10 << (4 * (10 - 8)));	// Set PB10 AF to DFSDM2_CKOUT

	// GPIOC->MODER |= (1 << (2 * 10));	// Set STO_EN (PC10) to Output Mode 
	// GPIOC->MODER |= (0 << (2 * 11));	// Set STO_CH1_MCU_FBK (PC11) to Input Mode 
	// GPIOC->MODER |= (0 << (2 * 12));	// Set STO_CH2_MCU_FBK (PC12) to Input Mode 

	DFSDM2_Channel0->CHCFGR1 |= ((SYSCLK/ADCCLK - 1) << DFSDM_CHCFGR1_CKOUTDIV_Pos);

	DFSDM2_Channel1->CHCFGR1 |= (1 << DFSDM_CHCFGR1_SPICKSEL_Pos);	// Set channel 1 clock source CKOUT
	DFSDM2_Channel2->CHCFGR1 |= (1 << DFSDM_CHCFGR1_SPICKSEL_Pos);	// Set channel 2 clock source CKOUT
	DFSDM2_Channel3->CHCFGR1 |= (1 << DFSDM_CHCFGR1_SPICKSEL_Pos);	// Set channel 3 clock source CKOUT

	DFSDM2_Filter1->FLTCR1 |= (1 << DFSDM_FLTCR1_RCH_Pos);	// Set filter 1 to channel 1
	DFSDM2_Filter2->FLTCR1 |= (2 << DFSDM_FLTCR1_RCH_Pos);	// Set filter 2 to channel 2
	DFSDM2_Filter3->FLTCR1 |= (3 << DFSDM_FLTCR1_RCH_Pos);	// Set filter 3 to channel 3

//	DFSDM2_Filter1->FLTCR1 |= DFSDM_FLTCR1_RCONT;	// Filter 1 Continuous mode
//	DFSDM2_Filter2->FLTCR1 |= DFSDM_FLTCR1_RCONT;	// Filter 2 Continuous mode
//	DFSDM2_Filter3->FLTCR1 |= DFSDM_FLTCR1_RCONT;	// Filter 3 Continuous mode

	DFSDM2_Filter1->FLTFCR |= (3 << DFSDM_FLTFCR_FORD_Pos);	// Set Filter 1 to Sinc^3
	DFSDM2_Filter2->FLTFCR |= (3 << DFSDM_FLTFCR_FORD_Pos);	// Set Filter 2 to Sinc^3
	DFSDM2_Filter3->FLTFCR |= (3 << DFSDM_FLTFCR_FORD_Pos);	// Set Filter 3 to Sinc^3

	DFSDM2_Filter1->FLTFCR |= (DFSDM_FOSR << DFSDM_FLTFCR_FOSR_Pos);	// Set Filter 1 oversample
	DFSDM2_Filter2->FLTFCR |= (DFSDM_FOSR << DFSDM_FLTFCR_FOSR_Pos);	// Set Filter 2 oversample
	DFSDM2_Filter3->FLTFCR |= (DFSDM_FOSR << DFSDM_FLTFCR_FOSR_Pos);	// Set Filter 3 oversample

	DFSDM2_Channel1->CHCFGR2 |= 8 << DFSDM_CHCFGR2_DTRBS_Pos; 	// Right-shift data by 8 to fit 32bit into 24bit
	DFSDM2_Channel2->CHCFGR2 |= 8 << DFSDM_CHCFGR2_DTRBS_Pos; 	// Right-shift data by 8 to fit 32bit into 24bit
	DFSDM2_Channel3->CHCFGR2 |= 8 << DFSDM_CHCFGR2_DTRBS_Pos; 	// Right-shift data by 8 to fit 32bit into 24bit


	//TODO: Initialize short circuit detector
	//TODO: Initialize Analog Watchdog

	DFSDM2_Channel1->CHCFGR1 |= DFSDM_CHCFGR1_CHEN; 	// Enable Channel 1
	DFSDM2_Channel2->CHCFGR1 |= DFSDM_CHCFGR1_CHEN; 	// Enable Channel 2
	DFSDM2_Channel3->CHCFGR1 |= DFSDM_CHCFGR1_CHEN; 	// Enable Channel 3
	DFSDM2_Channel0->CHCFGR1 |= DFSDM_CHCFGR1_DFSDMEN; 	// Global enable DFSDM

	DFSDM2_Filter1->FLTCR1 |= DFSDM_FLTCR1_DFEN;	// Enable Filter 1
	DFSDM2_Filter2->FLTCR1 |= DFSDM_FLTCR1_DFEN;	// Enable Filter 2
	DFSDM2_Filter3->FLTCR1 |= DFSDM_FLTCR1_DFEN;	// Enable Filter 3
}

int Clarke_and_Park_Transform(float theta, float A, float B, float C, float *D, float *Q){
    
    // assert((0 <= theta) && (theta <= 2*pi));    // Assert theta
    // assert(fabs(A + B + C) < 0.1);    // Assert ABC currents agree
    
    float X = (2 * A - B - C) * (1 / sqrt(6));
    float Y = (B - C) * (1 / sqrt(2));
    
    float co = cos(theta);
    float si = sin(theta);
    
    *D = co*X + si*Y;
    *Q = co*Y - si*X;
    
    return 0;
}


int Inverse_Carke_and_Park_Transform(float theta, float D, float Q, float *A, float *B, float *C){
    
    // Inputs
    // theta = electrical angle [0, 2*pi] radians
    // D     = direct current [] amps
    // Q     = quadrature current [] amps
    
    // Outputs
    // A, B, C
    
    // assert((0 <= theta) && (theta <= 2*pi));    // Assert theta
    
    float co = cos(theta);
    float si = sin(theta);
    
    float X = co*D - si*Q;
    float Y = si*D + co*Q;
    
    *A = (sqrt(2.0 / 3.0)) * X;
    *B = -(1 / sqrt(6.0)) * X;
    *C = *B - (1.0 / sqrt(2.0)) * Y;
    *B += (1.0 / sqrt(2.0)) * Y;
    
    // assert(fabs(*A + *B + *C) < 0.001);    // Assert ABC currents agree
    
    return 0;
}


float Current_Controller(float EI){
	// Input:  EI = Error Current (A)
	// Output: RV = Requested Voltage (V)

	float P_Term = Kp * EI;
	I_Term += Ki *EI;

	if (I_Term > I_term_limit){
		I_Term = I_term_limit;
	}
	else if (I_Term < -I_term_limit){
		I_Term = -I_term_limit;
	}

	float RV = P_Term + I_Term;

	return RV;
}


extern "C" {
void TIM1_UP_TIM10_IRQHandler(void) {
    if (TIM1->SR & TIM_SR_UIF) { // Check if update interrupt flag is set
		TIM1->SR &= ~TIM_SR_UIF; // Clear update interrupt flag
		
		if(!(DFSDM2_Filter1->FLTISR & DFSDM_FLTISR_REOCF)){		// check if DFSDM conversion is complete
			//TODO: handle missing conversion, this should ony happen once at startup since there is not previous data
			return;
		}
		if(!(DFSDM2_Filter2->FLTISR & DFSDM_FLTISR_REOCF)){		// check if DFSDM conversion is complete
			//TODO: handle missing conversion, this should ony happen once at startup since there is not previous data
			return;
		}
		if(!(DFSDM2_Filter3->FLTISR & DFSDM_FLTISR_REOCF)){		// check if DFSDM conversion is complete
			//TODO: handle missing conversion, this should ony happen once at startup since there is not previous data
			return;
		}

		// set pwm exactly on edge based on data from last cycle
		TIM1->CCR1 = CAPTURE_COMP_U;
		TIM1->CCR2 = CAPTURE_COMP_V;
		TIM1->CCR3 = CAPTURE_COMP_W;

		theta += electrical_rads_per_second * (2*pi) * (.5/(PWMCLK*1000));

		if (theta >= (2*pi)){
			theta = 0.0;
		}


		// get conversion data from DFSDM
		// TODO: consider shifting a more optimal number of bit to get higher resolution
		int32_t MI_U_Raw = ((int32_t)(DFSDM2_Filter1->FLTRDATAR & 0xFFFFFF00));
		int32_t MI_V_Raw = ((int32_t)(DFSDM2_Filter2->FLTRDATAR & 0xFFFFFF00));
		int32_t MI_W_Raw = ((int32_t)(DFSDM2_Filter3->FLTRDATAR & 0xFFFFFF00));

		// start all conversions for next cycle
        // TODO: Ensure DFSDM conversions occur on PWM timer direction flips
		DFSDM2_Filter1->FLTCR1 |= DFSDM_FLTCR1_RSWSTART;	// Start Filter 1 Conversion
		DFSDM2_Filter2->FLTCR1 |= DFSDM_FLTCR1_RSWSTART;	// Start Filter 2 Conversion
		DFSDM2_Filter3->FLTCR1 |= DFSDM_FLTCR1_RSWSTART;	// Start Filter 3 Conversion

		float measured_current_U = -MI_U_Raw;
		measured_current_U *= ADC_VOLTAGE;
		measured_current_U /= (float)(pow(DFSDM_FOSR+1, 3) * (1.0 - ADC_MIN_VALUE_PERC*2.0) * SHUNT_RESISTANCE);

		float measured_current_V = -MI_V_Raw;
		measured_current_V *= ADC_VOLTAGE;
		measured_current_V /= (float)(pow(DFSDM_FOSR+1, 3) * (1.0 - ADC_MIN_VALUE_PERC*2.0) * SHUNT_RESISTANCE);

		float measured_current_W = -MI_W_Raw;
		measured_current_W *= ADC_VOLTAGE;
		measured_current_W /= (float)(pow(DFSDM_FOSR+1, 3) * (1.0 - ADC_MIN_VALUE_PERC*2.0) * SHUNT_RESISTANCE);

		float measured_Id;
		float measured_Iq;

		Clarke_and_Park_Transform(theta, measured_current_U, measured_current_V, measured_current_W, &measured_Id, &measured_Iq);

		float error_Iq = requested_Iq - measured_Iq;
		float error_Id = requested_Id - measured_Id;

		float requested_Vq = Current_Controller(error_Iq);
		float requested_Vd = Current_Controller(error_Id);

		float requested_duty_cycle_Q = requested_Vq / (0.5*VBUS);
		float requested_duty_cycle_D = requested_Vd / (0.5*VBUS);

		float requested_duty_cycle_U;
		float requested_duty_cycle_V;
		float requested_duty_cycle_W;

		Inverse_Carke_and_Park_Transform(theta, requested_duty_cycle_D, requested_duty_cycle_Q, &requested_duty_cycle_U, &requested_duty_cycle_V, &requested_duty_cycle_W);

		if (requested_duty_cycle_U > (float)0.95){
			requested_duty_cycle_U = 0.95;
		}
		if (requested_duty_cycle_U < (float)-0.95){
			requested_duty_cycle_U = -0.95;
		}

		if (requested_duty_cycle_V > (float)0.95){
			requested_duty_cycle_V = 0.95;
		}
		if (requested_duty_cycle_V < (float)-0.95){
			requested_duty_cycle_V = -0.95;
		}

		if (requested_duty_cycle_W > (float)0.95){
			requested_duty_cycle_W = 0.95;
		}
		if (requested_duty_cycle_W < (float)-0.95){
			requested_duty_cycle_W = -0.95;
		}

		requested_duty_cycle_U = (requested_duty_cycle_U + 1)/2;
		requested_duty_cycle_V = (requested_duty_cycle_V + 1)/2;
		requested_duty_cycle_W = (requested_duty_cycle_W + 1)/2;

		CAPTURE_COMP_U = requested_duty_cycle_U * 500*SYSCLK/PWMCLK;
		CAPTURE_COMP_V = requested_duty_cycle_V * 500*SYSCLK/PWMCLK;
		CAPTURE_COMP_W = requested_duty_cycle_W * 500*SYSCLK/PWMCLK;
    }
}


void FPU_IRQHandler(void){
	while(1);
}

void HardFault_Handler(void){
	while(1);
}

void MemManage_Handler(void){
	while(1);
}

void BusFault_Handler(void){
	while(1);
}

void UsageFault_Handler(void){
	while(1);
}




void Start_Bit_Sent_Callback(void){
	while(1);
}

void Address_Sent_Callback(void){
	while(1);
}

void Ten_Bit_Header_Sent_Callback(void){
	while(1);
}

void Stop_Received_Callback(void){
	while(1);
}

void Data_Byte_Transfer_Finished_Callback(void){
	while(1);
}

void Receive_Buffer_Not_Empty_Callback(void){
	while(1);
}

void Transmit_Buffer_Empty_Callback(void){
	



}




void Bus_Error_Callback(void){
	while(1);
}

void Arbitration_Loss_Callback(void){
	while(1);
}

void Acknowledge_Failure_Callback(void){
	while(1);
}

void Overrun_Underrun_Callback(void){
	while(1);
}

void PEC_Error_Callback(void){
	while(1);
}

void Timeout_Tlow_Error_Callback(void){
	while(1);
}

void SMBus_Alert_Callback(void){
	while(1);
}




void Parity_Error_Callback(void){
	while(1);
}

void Framing_Error_Callback(void){
	while(1);
}

void Noise_Detected_Error_Callback(void){
	while(1);
}

void Overrun_Error_Callback(void){
	while(1);
}
