#include "adc.h"


/*!
    \brief Create object for managing ADC (analog to digital converter)
*/
adc::adc(){
}

/*!
    \brief Initialize hardware and start continuous conversions
    
    \note Run this after clocks are configured but before the main loop is started
*/
void adc::init(){

    // ADC is driven by the APB2 clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;     // enable ADC clock

    // ADC sample clock is derived from the APB2 clock and a configurable divider. 36MHz MAX
    // APB2 clock is 100MHz so we must set the divider to /4 which gives a 25Mhz ADC sample clock
    ADC->CCR = ADC_CCR_TSVREFE | (0b1 << ADC_CCR_ADCPRE_Pos);   // enable internal temp sensor and set clock prescaler

    ADC1->CR1 |= ADC_CR1_SCAN;  // enable scan mode

    ADC1->CR1 |= ADC_CR1_EOCIE; // enable end of conversion interrupt (after all channels have been converted)

    // TODO: setup trigger to adc triggers in sync with TIM1 (PWM out)
    // ADC1->CR2 |= (0b1 << ADC_CR2_EXTEN_Pos);    // rising edge trigger
    // ADC1->CR2 |= (0b0 << ADC_CR2_EXTSEL_Pos);   // timer 1 CC1 event

    // TODO: setup with DMA
    // ADC1->CR2 |= ADC_CR2_DMA;

    //ADC1->SMPR1 and ADC1->SMPR2 may be used to configure the number of samples taken on each ADC channel, default is the lowest which is 3

    // setup sample sequence
    ADC1->SQR3 |= (0 << ADC_SQR3_SQ1_Pos);  // Phase U voltage
    ADC1->SQR3 |= (1 << ADC_SQR3_SQ2_Pos);  // Phase V voltage
    ADC1->SQR3 |= (2 << ADC_SQR3_SQ3_Pos);  // Phase W voltage
    ADC1->SQR3 |= (3 << ADC_SQR3_SQ4_Pos);  // Phase U PFC sense voltage
    ADC1->SQR3 |= (4 << ADC_SQR3_SQ5_Pos);  // Phase V PFC sense voltage
    ADC1->SQR3 |= (5 << ADC_SQR3_SQ6_Pos);  // Phase W PFC sense voltage

    ADC1->SQR2 |= (6 << ADC_SQR2_SQ7_Pos);  // DC bus voltage
    ADC1->SQR2 |= (7 << ADC_SQR2_SQ8_Pos);  // Gate drive voltage
    ADC1->SQR2 |= (9 << ADC_SQR2_SQ7_Pos);  // Heatsink temp 1
    ADC1->SQR2 |= (10 << ADC_SQR2_SQ7_Pos);  // Heatsink temp 2
    ADC1->SQR2 |= (11 << ADC_SQR2_SQ7_Pos);  // Board temp 1
    ADC1->SQR2 |= (18 << ADC_SQR2_SQ7_Pos);  // MCU internal temp

    ADC1->SQR1 |= ((12-1) << ADC_SQR1_L_Pos);  // Set to do 12 total conversions (as many as setup above)

    ADC1->CR2 |= ADC_CR2_ADON;  // turn on ADC

}