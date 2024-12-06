#include "fans.h"

#define APB1_Clock_Frequency 50000000 

fans::fans()
{
    // Enable clock for GPIOB
RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

// Configure PB6 as alternate function (AF2 for TIM4_CH1)
GPIOB->MODER &= ~(GPIO_MODER_MODER6);
GPIOB->MODER |= (GPIO_MODER_MODER6_1);
GPIOB->AFR[0] |= (2 << GPIO_AFRL_AFSEL6_Pos);

// Enable clock for TIM4
RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

// Set prescaler to get 25kHz PWM frequency
TIM4->PSC = (APB1_Clock_Frequency / 2 / 25000) - 1;

// Set auto-reload register for 1000 ticks (for 25kHz PWM)
TIM4->ARR = 1000 - 1;

// Set compare value to 500 (50% duty cycle initially)
TIM4->CCR1 = 500;

// Configure TIM4 channel 1 in PWM mode 1
TIM4->CCMR1 &= ~(TIM_CCMR1_OC1M);
TIM4->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);
TIM4->CCMR1 |= TIM_CCMR1_OC1PE;

// Enable capture/compare channel 1
TIM4->CCER |= TIM_CCER_CC1E;

// Enable counter
TIM4->CR1 |= TIM_CR1_CEN;
}

uint32_t fans::set_speed(uint32_t speed_rpm)
{
    // Limit speed to 14500 RPM
    if (speed_rpm > 14500) speed_rpm = 14500;

    // Calculate duty cycle
    uint32_t duty_cycle = (speed_rpm * 1000) / 14500;
    TIM4->CCR1 = duty_cycle;

    // Return Highest Error
    return 0;
}