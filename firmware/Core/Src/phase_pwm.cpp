#include "phase_pwm.h"


/*!
    \brief Create object for TIM1 peripheral for outputting main PWM
*/
phase_pwm::phase_pwm(uint32_t i){

}

/*!
    \brief Initialize hardware
    
    \note Run this after clocks are configured but before the main loop is started
*/
void phase_pwm::init(){
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
	
    // TIM1->BDTR |= TIM_BDTR_AOE; // TODO: remove automatic restart support by adding a separate call to re-enable main outputs
	
    TIM1->BDTR |= TIM_BDTR_BKP;
	__NOP();__NOP();__NOP();		// Inserts a delay of 3 clock cycles	//TODO: investigate NOPs
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

    TIM1->CR1 |= TIM_CR1_CEN;		// Enable counting (this does not enable main outputs yet)

}


/*!
    \brief Attempt to turn on main PWM outputs
*/
void phase_pwm::enable(){
    // Set all channels to 50% by default
	TIM1->CCR1 = PWM_ticks * 0.5;
	TIM1->CCR2 = PWM_ticks * 0.5;
	TIM1->CCR3 = PWM_ticks * 0.5;

	TIM1->CNT = 0;					// Reset PWM counter
    TIM1->SR &= ~TIM_SR_BIF;     // Clear break input flag
	TIM1->BDTR |= TIM_BDTR_MOE;     // Main output enable
}


/*!
    \brief Turn off main PWM outputs
*/
void phase_pwm::disable(){
	TIM1->BDTR &= ~TIM_BDTR_MOE;     // Main output disable
}


/*!
    \brief Check if the break input flag has been set
*/
bool phase_pwm::break_flag_triggered(){
	return TIM1->SR & TIM_SR_BIF != 0;     // get break input flag
}


/*!
    \brief Attempt to clear break input flag (will not be cleared if the break input is still active)
*/
void phase_pwm::clear_break_flag(){
	TIM1->SR &= ~TIM_SR_BIF;     // Clear break input flag
}


/*!
    \brief Sets the compare values that control the PWM dutycycles

    \note these are not verified in any way, make sure they are in range!
*/
void phase_pwm::set(uint32_t U, uint32_t V, uint32_t W){
    TIM1->CCR1 = U;
    TIM1->CCR2 = V;
    TIM1->CCR3 = W;
}
