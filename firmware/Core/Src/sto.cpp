#include "sto.h"

void sto::log_message(uint32_t message) {
    log->log_message(message);
}


/*!
    \brief Create object for managing STO IO
*/
sto::sto(uint32_t i){

}

/*!
    \brief Initialize hardware
    
    \note Run this after clocks are configured but before the main loop is started
*/
// void sto::init(){
    
// 	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;    // Enable GPIOC  Clock

// 	GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER10) | GPIO_MODER_MODER10_0;		// set STO_EN (PC10) to output
// 	GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER11);		// set STO_CH1_FBK (PC11) to input
// 	GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER12);		// set STO_CH2_FBK (PC12) to input

// }


// /*!
//     \brief Enable drive
// */
// void sto::enable(){
// 	GPIOC->BSRR |= GPIO_BSRR_BS10;	// turn on STO_EN output (note this also needs both external STO channels to be on to allow the drive to output)
// }


// /*!
//     \brief Disable drive
// */
// void sto::disable(){
// 	GPIOC->BSRR |= GPIO_BSRR_BR10;	// turn off STO_EN output (note this also needs both external STO channels to be on to allow the drive to output)
// }


/*!
    \brief Check if the STO feedback makes sense (this NOT checking if the drive is enabled, just that the hardware seems good)

	\returns 0 when OK
*/
uint32_t sto::check_fault(){

	// check both feedback signals are the same
	if((GPIOC->IDR & GPIO_IDR_ID11)>>GPIO_IDR_ID11_Pos != (GPIOC->IDR & GPIO_IDR_ID12)>>GPIO_IDR_ID12_Pos){
		return 0b1;		// return signifying a fault
	}

	if(!((GPIOC->ODR & GPIO_ODR_OD10)>0)){		// MCU STO output enable is off
		if(!((GPIOC->IDR & GPIO_IDR_ID11) > 0)){
			return 0b10;		// return signifying a fault, feedback is not allowed to be on(inverted) if the enable is off
		}
	}

	return 0;
}


/*!
    \brief Check if the drive is allowed to enable PWM output
*/
bool sto::output_allowed(){

	// check both feedback signals are the same
	if((GPIOC->IDR & GPIO_IDR_ID11)>>GPIO_IDR_ID11_Pos != (GPIOC->IDR & GPIO_IDR_ID12)>>GPIO_IDR_ID12_Pos){
		return false;		// inputs do not match
	}

	if(!(GPIOC->ODR & GPIO_ODR_OD10)){		// MCU STO output enable is off
		return false;		// enable is off
	}

	if(GPIOC->IDR & GPIO_IDR_ID11 || GPIOC->IDR & GPIO_IDR_ID12){
		return false;		// one or both feedback signals are not active (inverted)
	}

	return true;
}