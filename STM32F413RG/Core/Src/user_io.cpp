#include "user_IO.h"


/*!
    \brief Create object for managing user IO (LEDs and DIP switches)
    
    \param update_period_ Period in milliseconds at which the update() function will be called.
*/
user_io::user_io(uint32_t update_period_){
    update_period_ms = update_period_;
}


/*!
    \brief Initialize hardware and configure IO expander
    
    \note Run this after clocks are configured but before the main loop is started
*/
void user_io::init(void){

    //PB7 = SDA
	//PB8 = SCL

    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;		// Enable I2C1 Peripheral Clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;	// Enable GPIOB Peripheral Clock

	GPIOB->MODER &= ~(GPIO_MODER_MODER7 | GPIO_MODER_MODER8); 				// Clear mode bits
    GPIOB->MODER |= (GPIO_MODER_MODER7_1 | GPIO_MODER_MODER8_1); 			// Alternate function mode
    GPIOB->OTYPER |= (GPIO_OTYPER_OT_7 | GPIO_OTYPER_OT_8); 				// Open-drain
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD7 | GPIO_PUPDR_PUPD8); 				// Clear pull-up/pull-down bits
    GPIOB->PUPDR |= (GPIO_PUPDR_PUPD7_0 | GPIO_PUPDR_PUPD8_0); 				// Pull-up
    GPIOB->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR7 | GPIO_OSPEEDER_OSPEEDR8);	// Very high speed

    GPIOB->AFR[0] |= (4 << (7 * 4)); 			// Set AFR bits to AF4 (I2C1) for PB7
    GPIOB->AFR[1] |= (4 << ((8 - 8) * 4)); 		// Set AFR bits to AF4 (I2C1) for PB8

	RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;	// Reset I2C peripheral to clear any settings
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;

	I2C1->CR1 |= (1<<15);  // reset I2C
	I2C1->CR1 &= ~(1<<15);

	I2C1->CR2 |= (50 << I2C_CR2_FREQ_Pos) & I2C_CR2_FREQ_Msk;	// Set I2C1 Peripheral Clock Frequency to 50MHz
	
	// I2C1->CCR = (1 << I2C_CCR_FS_Pos) & I2C_CCR_FS_Msk;	// Set I2C1 Fast/Standard mode to Fast Mode
	I2C1->CCR |= (250 << I2C_CCR_CCR_Pos) & I2C_CCR_CCR_Msk;	// Set I2C1 Clock control register to 100kHz
	// CCR = Peripheral_Clock_1 / (3 * SCL_Output_Frequency)

	I2C1->TRISE |= (51 << I2C_TRISE_TRISE_Pos) & I2C_TRISE_TRISE_Msk;	//Set Maximum Rise Time to 1000ns
	//TRISE = Maximum_Rise_Time / Peripheral_Clock_1_Period + 1
	// 51   =       1000ns      /            20ns           + 1

	I2C1->CR1 |= (1 << I2C_CR1_PE_Pos) & I2C_CR1_PE_Msk;	// Enable I2C1 Peripheral

	I2C1->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN;	// Enable I2C1 Interrupts

	NVIC_EnableIRQ(I2C1_EV_IRQn);	// Configure NVIC for I2C1 Event Inturrpt
    NVIC_EnableIRQ(I2C1_ER_IRQn);	// Configure NVIC for I2C1 Error Inturrpt

    update_step = done;    // enter idle state
}


/*!
    \brief Check for errors related to reading/writing to the IO expander
    
    \return Error value, zero means no error
*/
uint32_t user_io::get_errors(void){
    return error;
}


/*!
    \brief Get states of the DIP switches on the front of the drive
    
    \return Binary representation of the switch states
*/
uint32_t user_io::get_switch_states(void){
    return switch_state;
}


/*!
    \brief Set states of LEDs on the front of the drive
    
    \param led_select_ Bit mask for which leds you wish to modify. 0b1111 will modify all, 0b0001 will only modify LED_0

    \param led_mode_ Mode to set the selected LEDs to given by led_mode_enum

    \note
    Led mode options: {off, on, blink_slow, blink_medium, blink_fast}

    \note
    Example:
    `userIO.set_led_state(0b1111, userIO.blink_medium)  // set all LEDs to blink at medium speed`
*/
void user_io::set_led_state(uint32_t led_select_, uint32_t led_mode_){
    assert(led_mode_ <= on);   // user tried using a mode not supported

    if(led_select_ & 0b0001) led_modes[0] = led_mode_;
    if(led_select_ & 0b0010) led_modes[1] = led_mode_;
    if(led_select_ & 0b0100) led_modes[2] = led_mode_;
    if(led_select_ & 0b1000) led_modes[3] = led_mode_;
}


/*!
    \brief Update IO states
    
    \note Run this as whatever period was set when user_IO was created
    
    \note a call to this function STARTS the update cycle, it is non-blocking so the actual states will be done updating at some later time
*/
void user_io::update(void){

    if(error){  // reset I2C if it gets stuck in an error state
        I2C1->CR1 |= I2C_CR1_SWRST;	// Reset I2C
        I2C1->CR1 &= ~I2C_CR1_SWRST;
        
        I2C1->CR2 |= (50 << I2C_CR2_FREQ_Pos) & I2C_CR2_FREQ_Msk;	// Set I2C1 Peripheral Clock Frequency to 50MHz
	
        // I2C1->CCR = (1 << I2C_CCR_FS_Pos) & I2C_CCR_FS_Msk;	// Set I2C1 Fast/Standard mode to Fast Mode
        I2C1->CCR |= (250 << I2C_CCR_CCR_Pos) & I2C_CCR_CCR_Msk;	// Set I2C1 Clock control register to 100kHz
        // CCR = Peripheral_Clock_1 / (3 * SCL_Output_Frequency)

        I2C1->TRISE |= (51 << I2C_TRISE_TRISE_Pos) & I2C_TRISE_TRISE_Msk;	//Set Maximum Rise Time to 1000ns
        //TRISE = Maximum_Rise_Time / Peripheral_Clock_1_Period + 1
        // 51   =       1000ns      /            20ns           + 1

        I2C1->CR1 |= (1 << I2C_CR1_PE_Pos) & I2C_CR1_PE_Msk;	// Enable I2C1 Peripheral

        I2C1->CR2 |= I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN;	// Enable I2C1 Interrupts
    }

    slow_blink_cycle_counter += update_period_ms;
    medium_blink_cycle_counter += update_period_ms;
    fast_blink_cycle_counter += update_period_ms;

    // toggle blink state bits if their period has passed
    if(slow_blink_cycle_counter >= slow_blink_time_ms){
        blink_state ^= (1 << 1);
        slow_blink_cycle_counter = 0;
    }
    if(medium_blink_cycle_counter >= medium_blink_time_ms){
        blink_state ^= (1 << 2);
        medium_blink_cycle_counter = 0;
    }
    if(fast_blink_cycle_counter >= fast_blink_time_ms){
        blink_state ^= (1 << 3);
        fast_blink_cycle_counter = 0;
    }

    led_state = (blink_state & led_modes[0]) ? 1 : 0;
    led_state |= (blink_state & led_modes[1]) ? 1<<1 : 0;
    led_state |= (blink_state & led_modes[2]) ? 1<<2 : 0;
    led_state |= (blink_state & led_modes[3]) ? 1<<3 : 0;

    if(update_step == done){
        update_step = idle;

        //I2C1->CR1 |= (1 << I2C_CR1_ACK_Pos);	// Enable I2C1 Acknowledge for reading data
        increment_update_step();
    }

}


/*!
    \brief Write to an i2c register

    \param device_address_ Device address (not left shifted)

    \param device_register_ Data register address in the target device

    \param register_data_ Data to write into the register

    \note Non-blocking, relies on interrupt calls
*/
void user_io::i2c_write(uint8_t device_address_, uint8_t device_register_, uint8_t register_data_){
    device_address = device_address_;
    device_register = device_register_;
    register_data = register_data_;

    I2C1->CR1 |= I2C_CR1_START;	// Send START condition
    i2c_state = write_send_start_bit_wait;
}

/*!
    \brief Read from an i2c register

    \param device_address_ Device address (not left shifted)

    \param device_register_ Data register address in the target device

    \note Non-blocking, relies on interrupt calls
*/
void user_io::i2c_read(uint8_t device_address_, uint8_t device_register_){
    device_address = device_address_;
    device_register = device_register_;

    I2C1->CR1 |= I2C_CR1_START;	// Send START condition
    i2c_state = read_send_start_bit_wait;
}


/*!
    \brief Increment to the next update step (i2c transaction)
*/
void user_io::increment_update_step(void){
    if(update_step < done) update_step++;
    if(update_step == done) error = 0;

    switch(update_step){
        case set_config0:
            i2c_write(0x20, 0x06, 0b11110000);  // configure certain pins to be outputs (LEDs)
            break;

        case set_inversion0:
            i2c_write(0x20, 0x04, 0b11111111);  // set invert pins
            break;

        case set_output0:
            i2c_write(0x20, 0x02, (uint8_t)~(led_state & 0b1111));  // set outputs
            break;

        case get_input0:
            i2c_read(0x20, 0x00);
            break;
    };
}


/*!
    \brief handler for i2c event interrupts
    \note this needs to be placed in `I2C1_EV_IRQHandler(void)`
*/
void user_io::I2C1_Event_Interrupt(void){

    volatile bool start_bit_sent = 0;
    volatile bool address_sent = 0;
    volatile bool stop_bit_received = 0;
    volatile bool data_byte_transfer_complete = 0;
    volatile bool receive_buffer_not_empty = 0;
    volatile bool transmit_buffer_empty = 0;
    volatile bool addr_match = 0;

	if (I2C1->SR1 & I2C_SR1_SB) {
        start_bit_sent = 1;
        I2C1->SR1 &= ~I2C_SR1_SB; // Clear the interrupt flag
    }
    if (I2C1->SR1 & I2C_SR1_ADDR) {
        address_sent = 1;
        (void) I2C1->SR2; // Clear the interrupt flag by reading SR2
    }
    if (I2C1->SR1 & I2C_SR1_ADDR) {
        addr_match = 1;
        I2C1->SR1 &= ~I2C_SR1_ADDR; // Clear the interrupt flag
    }
    if (I2C1->SR1 & I2C_SR1_STOPF) {
        stop_bit_received = 1;
        I2C1->CR1 |= I2C_CR1_PE; // Clear the STOPF flag by writing to CR1
    }
    if (I2C1->SR1 & I2C_SR1_BTF) {
        data_byte_transfer_complete = 1;
        I2C1->SR1 &= ~I2C_SR1_BTF; // Clear the interrupt flag
    }
    if (I2C1->SR1 & I2C_SR1_RXNE) {
        receive_buffer_not_empty = 1;
        I2C1->SR1 &= ~I2C_SR1_RXNE; // Clear the interrupt flag
    }
    if (I2C1->SR1 & I2C_SR1_TXE) {
        transmit_buffer_empty = 1;
        
    }
    

    switch (i2c_state){

        // handle WRITE
        case write_send_start_bit_wait:
            if(start_bit_sent){
                i2c_state++;    // move to next i2c state
                I2C1->DR = (device_address << 1);	// Set I2C data to slave device address (write mode)
            }
            break;

        case write_send_device_address_wait:
            if(transmit_buffer_empty){
                i2c_state++;    // move to next i2c state
                I2C1->DR = (device_register);	// Set I2C data to desired device register
            }
            break;

        case write_send_register_address_wait:
            if(transmit_buffer_empty){
                i2c_state++;    // move to next i2c state
                I2C1->DR = (register_data);	// Set I2C data to desired register data
            }
            break;

        case write_send_register_data_wait:
            if(transmit_buffer_empty){
                i2c_state++;    // move to next i2c state
                I2C1->CR1 |= I2C_CR1_STOP;	// Send STOP condition
                increment_update_step();
            }
            break;


        // handle READ
        case read_send_start_bit_wait:
            if(start_bit_sent){
                i2c_state++;    // move to next i2c state
                I2C1->DR = (device_address << 1);	// Set I2C data to slave device address (write mode)
            }
            break;

        case read_send_device_address_wait:
            if(transmit_buffer_empty){
                i2c_state++;    // move to next i2c state
                I2C1->DR = (device_register);	// Set I2C data to desired device register
            }
            break;
        
        case read_send_register_address_wait:
            if(transmit_buffer_empty){
                i2c_state++;    // move to next i2c state
                I2C1->CR1 |= I2C_CR1_START;     // Set repeat START state
            }
            break;

        case read_send_repeat_start_bit_wait:
            if(start_bit_sent){
                i2c_state++;    // move to next i2c state
                I2C1->DR = (device_address << 1) | 0b1;	// Set I2C data to slave device address (read mode)
            }
            break;

        case read_send_repeat_device_address_wait:
            if(receive_buffer_not_empty){   // TODO: figure out why this is not being set automatically
                i2c_state++;    // move to next i2c state
                I2C1->CR1 |= I2C_CR1_STOP;	// Send STOP condition
                switch_state = I2C1->DR >> 4;
                increment_update_step();
            }
            break;

    }

    
}


/*!
    \brief handler for i2c error interrupts
    \note this needs to be placed in `I2C1_ER_IRQHandler(void)`
*/
void user_io::I2C1_Error_Interrupt(void){
    error = 1;  //TODO: add support for more error states
}