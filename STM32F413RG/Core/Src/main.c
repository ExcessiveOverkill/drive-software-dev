#include "main.h"
#include "stm32f413xx.h"

#define SYSCLK		100		// (MHz) System Clock in MHz
#define ADCCLK		20		// (MHz) ADC output clock
#define DEADTIME	100		// (ns)  PWM Deadtime (ns)
#define PWMCLK		25		// (KHz) PWM Output Frequency

#define PWM_ticks 1000*SYSCLK/PWMCLK/2

#define ADC_VOLTAGE 0.25
#define ADC_MIN_VALUE_PERC 0.1094
#define DFSDM_DIVISIONS 8388608

#define VBUS 5
#define SHUNT_RESISTANCE 0.005


float Kp = 2;		// Proportional gain for current controller (V/A)
float Ki = 0;		// Integral gain for current controller (V/A/s)
float I_Term = 0;					// Integral term for current controller (V)

float RI_U = 1;

// DO NOT MODIFY THESE OUTSIDE OF ISR FUNCTION
uint32_t CAPTURE_COMP_U = PWM_ticks * 0.5;
uint32_t CAPTURE_COMP_V = PWM_ticks * 0.5;
uint32_t CAPTURE_COMP_W = PWM_ticks * 0.5;


void delay_us(uint32_t time_us) {
	 TIM2->CNT = 0;
	 while (TIM2->CNT < time_us*SYSCLK);
}

void RCC_init(){

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

	// Update the SystemCoreClock variable to the new clock speed
	SystemCoreClockUpdate(); // Update the SystemCoreClock global variable with the new clock frequency

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;	// Enable GPIOA  Clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;    // Enable GPIOB  Clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;    // Enable GPIOC  Clock
	RCC->APB2ENR |= RCC_APB2ENR_DFSDM2EN;   // Enable DFSDM2 Clock
	RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;		// Enable TIM1   Clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;		// Enable TIM2   Clock
}


void GPIO_init(){

	GPIOB->MODER |= (2 << (2 * 9));				// Set U_Data (PB9) to AF mode
	GPIOC->MODER |= (2 << (2 * 5));				// Set V_Data (PC5) to AF mode
	GPIOC->MODER |= (2 << (2 * 9));				// Set W_Data (PC9) to AF mode
	GPIOB->MODER |= (2 << (2 * 10));			// Set ADC_CLK (PB10) to AF mode
	GPIOB->AFR[1] |= (6  << (4 * (9  - 8)));	// Set PB9  AF to DFSDM2_ DATIN1
	GPIOC->AFR[0] |= (3  << (4 * (5  - 0)));	// Set PC5  AF to DFSDM2_ DATIN2
	GPIOC->AFR[1] |= (7  << (4 * (9  - 8)));	// Set PC9  AF to DFSDM2_ DATIN3
	GPIOB->AFR[1] |= (10 << (4 * (10 - 8)));	// Set PB10 AF to DFSDM2_CKOUT


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

	GPIOB->OSPEEDR |= GPIO_OSPEEDR_OSPEED10_1;

}

void TIM1_init(){

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
	NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 1); // Set priority as needed
	__enable_irq();
}

void TIM2_init(){
	TIM2->PSC = 0; // Prescaler
	TIM2->ARR = 0xFFFF;  // Max auto-reload value
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
	// others go here...
}


void DFSDM2_init(){

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

	DFSDM2_Filter1->FLTFCR |= ((1000*ADCCLK/PWMCLK/2 - 1) << DFSDM_FLTFCR_FOSR_Pos);	// Set Filter 1 oversample
	DFSDM2_Filter2->FLTFCR |= ((1000*ADCCLK/PWMCLK/2 - 1) << DFSDM_FLTFCR_FOSR_Pos);	// Set Filter 2 oversample
	DFSDM2_Filter3->FLTFCR |= ((1000*ADCCLK/PWMCLK/2 - 1) << DFSDM_FLTFCR_FOSR_Pos);	// Set Filter 3 oversample

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


int main(void){

	RCC_init();
	GPIO_init();
	TIM1_init();
	TIM2_init();
	DFSDM2_init();
	PWM_enable();

	while(1){

		//TODO: RS422 input Requested current

		delay_us(100);
	}
}



float Current_Controller(float EI){
	// Input:  EI = Error Current (A)
	// Output: RV = Requested Voltage (V)

	float P_Term = Kp * EI;
	I_Term += Ki *EI;

	float RV = P_Term + I_Term;

	return RV;
}



void TIM1_UP_TIM10_IRQHandler(void) {
    if (TIM1->SR & TIM_SR_UIF) { // Check if update interrupt flag is set
		TIM1->SR &= ~TIM_SR_UIF; // Clear update interrupt flag
		
		if(!(DFSDM2_Filter1->FLTISR & DFSDM_FLTISR_REOCF)){		// check if DFSDM conversion is complete
			//TODO: handle missing conversion, this should ony happen once at startup since there is not previous data
			return;
		}

		// set pwm exactly on edge based on data from last cycle
		TIM1->CCR1 = CAPTURE_COMP_U;


		// get conversion data from DFSDM
		int32_t MI_U_Raw = ((int32_t)(DFSDM2_Filter1->FLTRDATAR & 0xFFFFFF00)) / 0x100;
		// other channels...

		// start all conversions for next cycle
        // TODO: Ensure DFSDM conversions occur on PWM timer direction flips
		DFSDM2_Filter1->FLTCR1 |= DFSDM_FLTCR1_RSWSTART;	// Start Filter 1 Conversion
//		DFSDM2_Filter2->FLTCR1 |= DFSDM_FLTCR1_RSWSTART;	// Start Filter 2 Conversion
//		DFSDM2_Filter3->FLTCR1 |= DFSDM_FLTCR1_RSWSTART;	// Start Filter 3 Conversion


		float scale = 0x7FFFFF*0.89/50;

		float MI_U = -1 * MI_U_Raw / scale;

		float EI_U = RI_U - MI_U;

		float RV_U = Current_Controller(EI_U);

		float RD_U = RV_U / (0.5*VBUS);

		if (RD_U > 0.95) RD_U = 0.95;
		else if (RD_U < -0.95) RD_U = -0.95;

		RD_U = (RD_U + 1)/2;

		CAPTURE_COMP_U = RD_U * 500*SYSCLK/PWMCLK;
    }
}



