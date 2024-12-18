#include "device.h"

device::device(){
}

void device::init(){
    CPU_init();
    timer_us_init();
    sysTick_init();

    logs.init();
    // Initialize all the low level classes
    Comm.init();
    Fans.init();
    UserIO.init();
    CurrentSense.init();
    PhasePWM.init();
    Adc.init();
    Sto.init();

    // short startup test
    startup_demo();
}

void device::CPU_init(){

    // 100MHz SYSCLK
    assert(SYSCLK == 100);

	FLASH->ACR |= FLASH_ACR_ICEN			// Enable intruction cache
			    | FLASH_ACR_DCEN 			// Enable Date Cache
			    | FLASH_ACR_PRFTEN 			// Enable prefetch
			    | FLASH_ACR_LATENCY_3WS;	// Set Flash latency to 3 wait states

    // HSI clock is used as the PLL input clock (16MHz)
    // set VCO to 2Mhz
    // set PLL_N to get SYSCLK*2
    // set PLL_P to 2 to get SYSCLK for they system
    // set PLL_Q to 5 (SYSCLK*2 / 5) for USB, SDIO, RNG, must be 48Mhz or lower
    // set PLL_R to 2 (SYSCLK*2 / 2) for I2S, DFSDM, must be 96Mhz or lower

	RCC->PLLCFGR = (8 << 0)    // Set PLL_M to 8. The input clock frequency is divided by this value.
	             | (SYSCLK << 6)  // Set PLL_N, the multiplication factor for the PLL. SYSCLK is presumably defined elsewhere, representing the desired system clock frequency.
	             | (0 << 16)   // Set PLL_P to 2 (0 in register corresponds to PLL_P = 2). The PLL output frequency is divided by this value to get the system clock.
	             | (5 << 24)  // Set PLL_Q to 5. This value is used for USB, SDIO, and random number generator clocks
                 | (2 << 28); // Set PLL_R to 2. This value is used for I2S and DFSDM clocks
	
    // Turn on the PLL and wait for it to become stable
	RCC->CR |= RCC_CR_PLLON;  // Enable the PLL
	while (!(RCC->CR & RCC_CR_PLLRDY)); // Wait for PLL to be ready (PLL ready flag)

	// Switch the system clock source to the PLL
	RCC->CFGR &= ~RCC_CFGR_SW;  // Clear the clock switch bits
	RCC->CFGR |= RCC_CFGR_SW_PLL;  // Set the clock source to PLL
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait until PLL is used as the system clock source

	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;	// divide by 2 to get 50Mhz for APB1 peripherals (max allowed)

	// Update the SystemCoreClock variable to the new clock speed
	SystemCoreClockUpdate(); // Update the SystemCoreClock global variable with the new clock frequency
	
	SystemInit();	// Initialize system
}

void device::timer_us_init(){
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;		// Enable TIM2 Clock
    __DSB();
	TIM2->PSC = (SYSCLK*1e6 / 1e6) - 1; // Prescaler to count in microseconds
    TIM2->EGR |= TIM_EGR_UG; // Generate an update event to update the prescaler
	TIM2->ARR = 0xFFFFFFFF;  // Max auto-reload value (2^32 - 1)
    TIM2->DIER |= TIM_DIER_UIE; // Enable update interrupt
	TIM2->CR1 |= TIM_CR1_CEN; // Enable TIM2
    NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 interrupt
    microseconds = 0; // Initialize the microseconds counter
}

uint64_t device::get_microseconds(){
    microseconds &= 0xFFFFFFFF00000000; // Clear the lower 32 bits of the microseconds counter
    microseconds |= (TIM2->CNT & 0xFFFFFFFF); // Add the current timer value to the microseconds counter
    return microseconds;
}

void device::delay_us(uint32_t time_us){
    volatile uint64_t time = TIM2->CNT;
    uint64_t end_time = time + time_us;

    while (time < end_time){    // Wait until the desired time has passed
        time = TIM2->CNT; // Update the time
    }
}

void device::delay_ms(uint32_t time_ms){
    for (uint32_t i = 0; i < time_ms; i++){
        delay_us(1e3); // delay 1ms
    }
}

void device::sysTick_init(){
    SysTick->LOAD = ((SYSCLK*1e6) / SYSTICK_FREQUENCY) - 1; // Set the SysTick timer to count at the desired frequency
    NVIC_EnableIRQ(SysTick_IRQn); // Enable the SysTick interrupt
    SysTick->CTRL = 0b111;	// Enable counter, interrupt, and set clock source to system clock
}

void device::startup_demo(){
    // Test LEDs and fans
    delay_ms(1000);    // Wait 1 second

    UserIO.set_led_state(0b1111, UserIO.on);
    Fans.set_speed(Fans.max_rpm);

    delay_ms(2000);    // Wait 2 seconds
    UserIO.set_led_state(0b1111, UserIO.blink_slow);
    Fans.set_speed(0);

    // TODO: Add more startup tests
}

void device::run(){
    while(1){
        // Run the main loop
    }
}

void device::update(){
    // TODO: Add low frequency update code
}

void device::critical_shutdown(){
    // Disable all hardware and enter a safe state
    UserIO.set_led_state(0b1111, UserIO.blink_fast);
    Fans.set_speed(Fans.max_rpm);

    // TODO: Add more shutdown procedures
}

void device::SysTick_Handler(void){
    get_microseconds(); // Update the microseconds counter (must be called periodically to prevent rollover errors)
    
    Fans.SysTick_Handler();
    UserIO.SysTick_Handler();

    update();
}

void device::TIM2_IRQHandler(void){
    TIM2->SR &= ~TIM_SR_UIF; // Clear the update interrupt flag
    microseconds += 0x100000000; // Add 2^32 to the microseconds counter
}

void device::I2C1_EV_IRQHandler(void){
    UserIO.I2C1_EV_IRQHandler();
}

void device::I2C1_ER_IRQHandler(void){
    UserIO.I2C1_ER_IRQHandler();
}

void device::DMA2_Stream0_IRQHandler(void){
    Adc.dma_interrupt_handler();
}

void device::DMA2_Stream1_IRQHandler(void){
    Comm.dma_stream1_interrupt_handler();
}

void device::TIM1_UP_TIM10_IRQHandler(void){
    // TODO: Add TIM1_UP_TIM10_IRQHandler code
    if (TIM1->SR & TIM_SR_UIF) { // Check if update interrupt flag is set
        TIM1->SR &= ~TIM_SR_UIF; // Clear update interrupt flag
    }
}

void device::USART6_IRQHandler(void){
    Comm.usart6_interrupt_handler();
}

device::IRQ device::missed_irq = IRQ::NONE;
void device::missed_irq_handler(IRQ irq){
    missed_irq = irq;
}