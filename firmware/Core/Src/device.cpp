#include "device.h"

// extern device_struct vars;
// extern void* var_pointers[86];

device::device(){
    comm_vars = &vars;
    comm_var_pointers = var_pointers;
}

void device::init(){
    CPU_init();
    sysTick_init();
 
    #ifdef RELEASE_MODE
        watchdog_init();
    #endif

    logs.init();
    // Initialize all the low level classes
    Comm.init();
    Fans.init();
    UserIO.init();
    CurrentSense.init();
    PhasePWM.init();
    Adc.init();
    Sto.init();


    micros = Comm.micros;
    logs.microseconds = Comm.micros;
    Comm.comm_vars = comm_vars;
    Comm.comm_var_pointers = comm_var_pointers;

    // TODO: add watchdog timer to disable PWM if system is unresponsive

    // short startup test
    //startup_demo();

    // todo: get this from the controller
    Comm.set_sync_frequency(1000); // 1kHz sync frequency
    Comm.set_pwm_timer_sync_offset_us(0); // no offset

    UserIO.set_led_state(0b1000, UserIO.blink_slow);    // signify device is on and running

    delay_ms(500); // wait for userIO to update

    // wait until we have a valid communication address before initializing communication
    Comm.set_device_address(UserIO.get_switch_states());
    
    delay_ms(500); // wait for communication to update
    Comm.enable_resync = true; // enable resync

    Sto.enable();
    PhasePWM.enable();
    
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

void device::delay_us(uint32_t time_us){
    uint64_t end_time = Comm.get_microseconds() + time_us;
    while (Comm.get_microseconds() < end_time){    // Wait until the desired time has passed
        watchdog_reload();
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
    NVIC_SetPriority(USART6_IRQn, 6);
    SysTick->CTRL = 0b111;	// Enable counter, interrupt, and set clock source to system clock
}

void device::watchdog_init(){
    IWDG->KR = 0x5555; // Enable write access to the IWDG_PR and IWDG_RLR registers
    IWDG->PR = 0b000; // Set the prescaler to 4 (LSI at 32khz / 4)
    IWDG->RLR = 8000 / 1000; // Set the reload value to 1ms
    IWDG->KR = 0xAAAA; // Reload the watchdog timer
    IWDG->KR = 0xCCCC; // Start the watchdog timer
}

void device::watchdog_reload(){
    IWDG->KR = 0xAAAA; // Reload the watchdog timer
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

    volatile uint32_t speed = comm_vars->fan_speed_cmd;

    Fans.set_speed(Fans.max_rpm*vars.fan_speed_cmd/0xffff);

    update_leds();
    watchdog_reload();
}

void device::update_leds(){
    // LED 1 shown connection status
    if(Comm.is_ok()){
        UserIO.set_led_state(0b0001, UserIO.blink_fast);
    }
    else{
        UserIO.set_led_state(0b0001, UserIO.blink_slow);
    }
}

void device::critical_shutdown(){
    // Disable all hardware and enter a safe state
    UserIO.set_led_state(0b1111, UserIO.blink_fast);
    Fans.set_speed(Fans.max_rpm);

    // TODO: Add more shutdown procedures
}

void device::SysTick_Handler(void){
    Fans.SysTick_Handler();
    UserIO.SysTick_Handler();

    update();
    
}

void device::TIM2_IRQHandler(void){
    Comm.TIM2_IRQHandler();
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

    Comm.update_timeout();
    watchdog_reload();
}

void device::USART6_IRQHandler(void){
    Comm.usart6_interrupt_handler();
}

device::IRQ device::missed_irq = IRQ::NONE;
void device::missed_irq_handler(IRQ irq){
    missed_irq = irq;
}