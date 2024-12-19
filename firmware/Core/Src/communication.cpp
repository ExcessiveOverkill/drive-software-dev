#include "communication.h"


communication::communication(logging* logs, const uint64_t* microseconds){
	this->microseconds = microseconds;
	this->logs = logs;
}

/*!
    \brief Initialize hardware
    
    \note Run this after clocks are configured but before the main loop is started
*/
void communication::init(){
    
	RCC->APB2ENR |= RCC_APB2ENR_USART6EN;     // enable clock (100 MHz)
	RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;	// enable CRC clock

	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;    // Enable GPIOA  Clock
	GPIOA->MODER |= GPIO_MODER_MODER11_1;	// set PA11 (TX) as alternate function
	GPIOA->MODER |= GPIO_MODER_MODER12_1;	// set PA12 (RX) as alternate function
	GPIOA->OSPEEDR |= 0b10 << GPIO_OSPEEDR_OSPEED11_Pos;	// set TX as fast speed output
	GPIOA->AFR[1] |= 8 << GPIO_AFRH_AFSEL11_Pos;	// set PA11 alternate function to 8 (USART6)
	GPIOA->AFR[1] |= 8 << GPIO_AFRH_AFSEL12_Pos;	// set PA12 alternate function to 8 (USART6)


	USART6->CR1 |= USART_CR1_OVER8;	// set 8x oversampling

	USART6->CR1 |= USART_CR1_UE;	// Enable USART6


	// set USARTDIV to 1 (maximum baudrate of 12.5 Mbit/s)
	USART6->BRR |= 1<<USART_BRR_DIV_Mantissa_Pos;	// Program Baud rate Mantissa
	USART6->BRR |= 0<<USART_BRR_DIV_Fraction;	// Program Baud rate fraction

	USART6->CR1 |= USART_CR1_IDLEIE;	// enable idle detect interrupt

	USART6->CR1 |= USART_CR1_RE;	// Enable Receiver

	// setup DMA2 stream1 for USART6 RX (stream 2 is also available)
	// setup DMA2 stream6 for USART6 TX	(stream 7 is also available)

    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN; // Enable DMA clock

    DMA2_Stream1->CR &= ~(DMA_SxCR_EN); // Disable DMA stream
	DMA2_Stream6->CR &= ~(DMA_SxCR_EN); // Disable DMA stream

    while(DMA2_Stream1->CR & (DMA_SxCR_EN_Msk));    // Wait for stream to disable
	while(DMA2_Stream6->CR & (DMA_SxCR_EN_Msk));    // Wait for stream to disable

	DMA2_Stream1->CR |= 5 << DMA_SxCR_CHSEL_Pos;	// select channel 5 for rx stream
	DMA2_Stream6->CR |= 5 << DMA_SxCR_CHSEL_Pos;	// select channel 5 for tx stream

    DMA2_Stream1->CR |= 0b00 << DMA_SxCR_PSIZE_Pos;     // set peripheral data size to 8bit (default)
	DMA2_Stream6->CR |= 0b00 << DMA_SxCR_PSIZE_Pos;     // set peripheral data size to 8bit (default)
    DMA2_Stream1->CR |= 0b10 << DMA_SxCR_MSIZE_Pos;     // set memory data size to 32bit
	DMA2_Stream6->CR |= 0b10 << DMA_SxCR_MSIZE_Pos;     // set memory data size to 32bit

    DMA2_Stream1->CR |= DMA_SxCR_MINC;      // auto-increment memory address (by set memory size)
	DMA2_Stream6->CR |= DMA_SxCR_MINC;      // auto-increment memory address (by set memory size)

    DMA2_Stream1->PAR = (uint32_t)&USART6->DR;        // use USART6 data register (rx)
	DMA2_Stream6->PAR = (uint32_t)&USART6->DR;        // use USART6 data register (tx)

    DMA2_Stream1->M0AR = (uint32_t)&rx.data_words;   // use rx_data as the memory for RX data
	DMA2_Stream6->M0AR = (uint32_t)&tx.data_words;   // use tx_data as the memory for TX data

	DMA2_Stream6->CR |= DMA_SxCR_DIR_0;		// configure TX stream for memory-to-peripheral

    set_rx_packet_length(4);	// default packet size
	set_tx_packet_length(4);

    //DMA2_Stream1->CR |= 0b01 << DMA_SxCR_MBURST_Pos;    // use 4 beat bursts
	//DMA2_Stream6->CR |= 0b01 << DMA_SxCR_MBURST_Pos;    // use 4 beat bursts

    DMA2_Stream1->CR |= 0b11 << DMA_SxCR_PL_Pos;    // Very high priority
	DMA2_Stream6->CR |= 0b11 << DMA_SxCR_PL_Pos;    // Very high priority

    DMA2_Stream1->FCR |= DMA_SxFCR_DMDIS;   // Disable direct transfer mode
	DMA2_Stream6->FCR |= DMA_SxFCR_DMDIS;   // Disable direct transfer mode

    //DMA2_Stream1->FCR |= 0b10 << DMA_SxFCR_FTH_Pos;     // Set FIFO threshold to full
	//DMA2_Stream6->FCR |= 0b10 << DMA_SxFCR_FTH_Pos;     // Set FIFO threshold to full

    DMA2_Stream1->CR |= DMA_SxCR_TCIE;  // Trigger interrupt when RX transfer to memory is complete
	//DMA2_Stream6->CR |= DMA_SxCR_TCIE;  // Trigger interrupt when TX transfer to peripheral is complete

	USART6->CR3 |= USART_CR3_DMAT;	// Enable DMA for tx

    NVIC_EnableIRQ(DMA2_Stream1_IRQn);  // Configure NVIC for DMA RX Interrupt	TODO: make sure this priority is lower than the PWM update IRQ
	//NVIC_EnableIRQ(DMA2_Stream6_IRQn);  // Configure NVIC for DMA TX Interrupt

	NVIC_EnableIRQ(USART6_IRQn);	// Enable USART6 interrupts

    DMA2_Stream1->CR |= DMA_SxCR_EN; // Enable DMA RX stream
	//DMA2_Stream6->CR |= DMA_SxCR_EN; // Enable DMA TX stream

	//TODO: verify no AHB error will be generated due to crossing a 1Kbyte boundary
}

bool communication::is_ok(){
	return !timed_out;
}

void communication::update_time_us(){
	if(*microseconds - last_valid_packet_time_us > timeout_limit_us){
		timed_out = true;
	}
}

void communication::reset_timeout(){
	timed_out = false;
	last_valid_packet_time_us = *microseconds;
}

/*!
    \brief Configure DMA for a new TX packet size in 32 bit words, the maximum size is defined by MAX_PACKET_SIZE
*/
void communication::set_tx_packet_length(uint32_t length){

	bool stream_enabled = DMA2_Stream6->CR & (DMA_SxCR_EN_Msk);

	if(stream_enabled){
		DMA2_Stream6->CR &= ~(DMA_SxCR_EN); // Disable DMA stream
		while(DMA2_Stream6->CR & (DMA_SxCR_EN_Msk));    // Wait for stream to disable
	}

	DMA2_Stream6->NDTR = length * 4;	// set number of 8 bit transfer cycles

	// if(stream_enabled){
	// 	DMA2_Stream6->CR |= DMA_SxCR_EN; // Enable DMA stream
	// }
	expected_tx_length = length;
}


/*!
    \brief Configure DMA for a new RX packet size in 32 bit words, the maximum size is defined by MAX_PACKET_SIZE
*/
void communication::set_rx_packet_length(uint32_t length){

	bool stream_enabled = DMA2_Stream1->CR & (DMA_SxCR_EN_Msk);

	if(stream_enabled){
		DMA2_Stream1->CR &= ~(DMA_SxCR_EN); // Disable DMA stream
		while(DMA2_Stream1->CR & (DMA_SxCR_EN_Msk));    // Wait for stream to disable
	}

	DMA2_Stream1->NDTR = length * 4;	// set number of 8 bit transfer cycles

	if(stream_enabled){
		DMA2_Stream1->CR |= DMA_SxCR_EN; // Enable DMA stream
	}
	expected_rx_length = length;
}


/*!
	\brief Set device address offset from DEVICE_STARTING_ADDRESS
*/
void communication::set_device_address(uint8_t address){
	device_address = address + DEVICE_STARTING_ADDRESS;
}


/*!
    \brief Receive data packet
*/
void communication::start_receive(){
	receive_complete = false;
	receive_started = true;
	USART6->CR3 |= USART_CR3_DMAR;	// Enable DMA for rx
	//USART6->CR3 |= USART_CR3_DMAT;	// Enable DMA for tx
}


/*!
    \brief Send data packet
*/
void communication::start_transmit(){
	
	set_tx_packet_length(expected_tx_length);	// set the number of 32 bit words to transfer
	
	// clear DMA flags
	DMA2->HIFCR |= DMA_HIFCR_CDMEIF6 | DMA_HIFCR_CTEIF6 | DMA_HIFCR_CFEIF6 | DMA_HIFCR_CHTIF6 | DMA_HIFCR_CTCIF6;
	USART6->CR1 &= ~USART_CR1_TE;	// Disable Transmitter
	USART6->SR &= ~USART_SR_TC_Msk;	// clear transmission complete flag

	DMA2_Stream6->CR |= DMA_SxCR_EN; // Enable DMA TX stream

	//USART6->DR = 0b10101010;	// send dummy byte
	USART6->CR1 |= USART_CR1_TE;	// Enable Transmitter
	
}


/*!
    \brief RX line is in idle state
*/
bool communication::rx_idle_detected(){
	return (USART6->SR & USART_SR_IDLE_Msk) >> USART_SR_IDLE_Pos;	// return true if IDLE flag is set
}


/*!
    \brief Handle DMA interrupt
*/
void communication::dma_stream1_interrupt_handler(){

	if(DMA2->LISR & DMA_LISR_TCIF1){
        USART6->CR3 &= ~USART_CR3_DMAR;	// Disable DMA for rx
		receive_complete = true;
		receive_started = false;
        DMA2->LIFCR |= DMA_LIFCR_CTCIF1;	// clear trasfer finished flag
    }
}


/*!
    \brief Handle USART6 interrupt
*/
void communication::usart6_interrupt_handler(){

	if(USART6->SR & USART_SR_IDLE_Msk){		// RX IDLE state detected (incomming transmission over)
		clear_rx_idle_flag();
        restart_rx_dma();	// TODO: only do this on invalid packet receive? (maybe)
		start_receive();

		int8_t result = interpret_rx_packet();
		if(result == 0){	// packet addressed to this device
			// start TX transmission upon valid packet receive
			generate_tx_packet();
			start_transmit();
			reset_timeout();
		}
		else if(result == 1){	// broadcast packet
			reset_timeout();
		}
		else{	// invalid packet
			// do nothing
		}
		
    }
	
}

uint32_t communication::calculate_crc(uint32_t *data, uint8_t data_length){
	CRC->CR = 0b1;	// clear and reset previous calculation
	for(uint8_t i=0; i<data_length; i++){
		CRC->DR = data[i];
	}
	return CRC->DR;
}

int8_t communication::interpret_rx_packet(){
	if(calculate_crc(rx.data_words, expected_rx_length-1) == rx.data_words[expected_rx_length-1]){	// verify CRC is correct
		if(rx.data_bytes[0] == device_address){	// verify device address matches
			//TODO: update registers
			return 0;	// address match
		}
		if(rx.data_bytes[0] == 0xFF){	// broadcast address
			return 1;	// broadcast mode
		}
	}
	return -1;	// invalid packet
}

void communication::pack_8_to_8_array(uint8_t data_in, uint16_t &offset, uint8_t *array){
	array[offset] = data_in;
	offset++;
}
void communication::pack_8_to_8_array(int8_t data_in, uint16_t &offset, uint8_t *array){
	array[offset] = reinterpret_cast<uint8_t&>(data_in);
	offset++;
}
void communication::pack_16_to_8_array(uint16_t data_in, uint16_t &offset, uint8_t *array){
	array[offset] = data_in & 0xFF;
	array[offset+1] = (data_in >> 8) & 0xFF;
	offset += 2;
}
void communication::pack_16_to_8_array(int16_t data_in, uint16_t &offset, uint8_t *array){
	uint16_t u_data_in = reinterpret_cast<uint16_t&>(data_in);
	array[offset] = u_data_in & 0xFF;
	array[offset+1] = (u_data_in >> 8) & 0xFF;
	offset += 2;
}
void communication::pack_32_to_8_array(uint32_t data_in, uint16_t &offset, uint8_t *array){
	array[offset] = data_in & 0xFF;
	array[offset+1] = (data_in >> 8) & 0xFF;
	array[offset+2] = (data_in >> 16) & 0xFF;
	array[offset+3] = (data_in >> 24) & 0xFF;
	offset += 4;
}
void communication::pack_32_to_8_array(int32_t data_in, uint16_t &offset, uint8_t *array){
	uint32_t u_data_in = reinterpret_cast<uint32_t&>(data_in);
	array[offset] = u_data_in & 0xFF;
	array[offset+1] = (u_data_in >> 8) & 0xFF;
	array[offset+2] = (u_data_in >> 16) & 0xFF;
	array[offset+3] = (u_data_in >> 24) & 0xFF;
	offset += 4;
}
void communication::pack_float_to_8_array(float data_in, uint16_t &offset, uint8_t *array){
	uint32_t u_data_in = reinterpret_cast<uint32_t&>(data_in);
	array[offset] = u_data_in & 0xFF;
	array[offset+1] = (u_data_in >> 8) & 0xFF;
	array[offset+2] = (u_data_in >> 16) & 0xFF;
	array[offset+3] = (u_data_in >> 24) & 0xFF;
	offset += 4;
}

uint8_t communication::get_register_size(uint16_t raw_address){
	if(raw_address >= int8_t_register_rw_start && raw_address <= int8_t_register_w_end){
		return 1;
	}

	else if(raw_address >= int16_t_register_rw_start && raw_address <= int16_t_register_w_end){
		return 2;
	}

	else if(raw_address >= int32_t_register_rw_start && raw_address <= int32_t_register_w_end){
		return 4;
	}
	
	else if(raw_address >= uint8_t_register_rw_start && raw_address <= uint8_t_register_w_end){
		return 1;
	}

	else if(raw_address >= uint16_t_register_rw_start && raw_address <= uint16_t_register_w_end){
		return 2;
	}
	
	else if(raw_address >= uint32_t_register_rw_start && raw_address <= uint32_t_register_w_end){
		return 4;
	}

	else if(raw_address >= float_register_rw_start && raw_address <= float_register_w_end){
		return 4;
	}
	
	else{
		return 0;
	}
}

void communication::debug(){
	//device_set_register(cyclic_read_address_0_register, static_cast<uint16_t>(current_measured_d_register));
	//device_set_register(enable_cyclic_data_register, 1);
	//device_set_register(current_measured_d_register, 1234);
	generate_tx_packet();
}

void communication::generate_tx_packet(){
	
	if(device_get_register(enable_cyclic_data_register) != 0 && !cyclic_mode_enabled){
		
		// setup cyclic mode
		cyclic_read_count = 0;
		cyclic_write_count = 0;

		for(uint8_t i=0; i<CYCLIC_ADDRESS_COUNT; i++){
			uint16_t read_address = device_get_register(static_cast<uint16_t_register_rw>(cyclic_read_address_0_register+i));
			uint16_t write_address = device_get_register(static_cast<uint16_t_register_rw>(cyclic_write_address_0_register+i));
			if(read_address != 0xFFFF){
				cyclic_read_addresses[cyclic_read_count] = read_address;
				cyclic_read_sizes[cyclic_read_count] = get_register_size(read_address);
				cyclic_read_count++;
			}
			if(write_address != 0xFFFF){
				cyclic_write_addresses[i] = write_address;
				cyclic_write_sizes[cyclic_write_count] = get_register_size(write_address);
				cyclic_write_count++;
			}
		}
		cyclic_mode_enabled = true;
	}

	packing_offset = 0;
	// device address
	pack_8_to_8_array(device_address, packing_offset, tx.data_bytes);

	// sequential data control/address and response
	pack_32_to_8_array(sequential_register_address, packing_offset, tx.data_bytes);
	pack_32_to_8_array(sequential_register_data_response, packing_offset, tx.data_bytes);

	// add cyclic data read packets
	if(cyclic_mode_enabled){
		for(int i=0; i<cyclic_read_count; i++){
			uint32_t raw_value;
			if(controller_get_register(cyclic_read_addresses[i], raw_value) == SUCCESS){
				switch (cyclic_read_sizes[i]){
					case 1:
						pack_8_to_8_array(reinterpret_cast<uint8_t&>(raw_value), packing_offset, tx.data_bytes);
						break;

					case 2:
						pack_16_to_8_array(reinterpret_cast<uint16_t&>(raw_value), packing_offset, tx.data_bytes);
						break;

					case 4:
						pack_32_to_8_array(raw_value, packing_offset, tx.data_bytes);
						break;
				
					default:
						break;
				}
			}
			else{
				// we must still fill packet space even though the register address was invalid
				error_reading_cyclic_address = true;
				switch (cyclic_read_sizes[i]){
					case 1:
						pack_8_to_8_array(static_cast<uint8_t>(0), packing_offset, tx.data_bytes);
						break;

					case 2:
						pack_16_to_8_array(static_cast<uint16_t>(0), packing_offset, tx.data_bytes);
						break;

					case 4:
						pack_32_to_8_array(static_cast<uint32_t>(0), packing_offset, tx.data_bytes);
						break;
				
					default:
						break;
				}
			}
		}
	}

	while(packing_offset % 4 != 0){	// pack additional blank bytes to ensure packet is a multiple of 32bits
		pack_8_to_8_array(static_cast<uint8_t>(0), packing_offset, tx.data_bytes);
	}

	pack_32_to_8_array(calculate_crc(tx.data_words, packing_offset/4), packing_offset, tx.data_bytes);

	if(expected_tx_length != packing_offset/4){
		set_tx_packet_length(packing_offset/4);
	}
}

/*!
    \brief Restart RX dma to re-sync with next data packet
*/
void communication::restart_rx_dma(){
	receive_started = false;
	receive_complete = false;
	DMA2_Stream1->CR &= ~(DMA_SxCR_EN); // Disable DMA stream
	while(DMA2_Stream1->CR & (DMA_SxCR_EN_Msk));    // Wait for stream to disable
	DMA2->LIFCR |= DMA_LIFCR_CTCIF1 | DMA_LIFCR_CHTIF1 | DMA_LIFCR_CTEIF1 | DMA_LIFCR_CDMEIF1 | DMA_LIFCR_CFEIF1;	// clear any interrupt flags
	DMA2_Stream1->CR |= DMA_SxCR_EN; // Enable DMA stream
}


/*!
    \brief Clear RX IDLE detection flag
*/
void communication::clear_rx_idle_flag(){
	volatile uint32_t temp = USART6->SR;	// read SR register
	temp = USART6->DR;	// read DR register
}


communication::controller_register_access_result communication::controller_set_register(uint16_t raw_address, uint32_t &raw_value){
	if(raw_address >= int8_t_register_rw_start && raw_address <= int8_t_register_rw_end){
		device_set_register(static_cast<int8_t_register_rw>(raw_address), raw_value);
	}
	else if(raw_address >= int8_t_register_w_start && raw_address <= int8_t_register_w_end){
		device_set_register(static_cast<int8_t_register_w>(raw_address), raw_value);
	}

	else if(raw_address >= int16_t_register_rw_start && raw_address <= int16_t_register_rw_end){
		device_set_register(static_cast<int16_t_register_rw>(raw_address), raw_value);
	}
	else if(raw_address >= int16_t_register_w_start && raw_address <= int16_t_register_w_end){
		device_set_register(static_cast<int16_t_register_w>(raw_address), raw_value);
	}

	else if(raw_address >= int32_t_register_rw_start && raw_address <= int32_t_register_rw_end){
		device_set_register(static_cast<int32_t_register_rw>(raw_address), raw_value);
	}
	else if(raw_address >= int32_t_register_w_start && raw_address <= int32_t_register_w_end){
		device_set_register(static_cast<int32_t_register_w>(raw_address), raw_value);
	}

	else if(raw_address >= uint8_t_register_rw_start && raw_address <= uint8_t_register_rw_end){
		device_set_register(static_cast<uint8_t_register_rw>(raw_address), raw_value);
	}
	else if(raw_address >= uint8_t_register_w_start && raw_address <= uint8_t_register_w_end){
		device_set_register(static_cast<uint8_t_register_w>(raw_address), raw_value);
	}

	else if(raw_address >= uint16_t_register_rw_start && raw_address <= uint16_t_register_rw_end){
		device_set_register(static_cast<uint16_t_register_rw>(raw_address), raw_value);
	}
	else if(raw_address >= uint16_t_register_w_start && raw_address <= uint16_t_register_w_end){
		device_set_register(static_cast<uint16_t_register_w>(raw_address), raw_value);
	}

	else if(raw_address >= uint32_t_register_rw_start && raw_address <= uint32_t_register_rw_end){
		device_set_register(static_cast<uint32_t_register_rw>(raw_address), raw_value);
	}
	else if(raw_address >= uint32_t_register_w_start && raw_address <= uint32_t_register_w_end){
		device_set_register(static_cast<uint32_t_register_w>(raw_address), raw_value);
	}

	else if(raw_address >= float_register_rw_start && raw_address <= float_register_rw_end){
		device_set_register(static_cast<float_register_rw>(raw_address), raw_value);
	}
	else if(raw_address >= float_register_w_start && raw_address <= float_register_w_end){
		device_set_register(static_cast<float_register_w>(raw_address), raw_value);
	}

	else{
		return controller_register_access_result::FAIL;
	}

	return controller_register_access_result::SUCCESS;
}

communication::controller_register_access_result communication::controller_get_register(uint16_t raw_address, uint32_t &raw_value){
	if(raw_address >= int8_t_register_rw_start && raw_address <= int8_t_register_rw_end){
		raw_value = device_get_register(static_cast<int8_t_register_rw>(raw_address));
	}
	else if(raw_address >= int8_t_register_r_start && raw_address <= int8_t_register_r_end){
		raw_value = device_get_register(static_cast<int8_t_register_r>(raw_address));
	}

	else if(raw_address >= int16_t_register_rw_start && raw_address <= int16_t_register_rw_end){
		raw_value = device_get_register(static_cast<int16_t_register_rw>(raw_address));
	}
	else if(raw_address >= int16_t_register_r_start && raw_address <= int16_t_register_r_end){
		raw_value = device_get_register(static_cast<int16_t_register_r>(raw_address));
	}

	else if(raw_address >= int32_t_register_rw_start && raw_address <= int32_t_register_rw_end){
		raw_value = device_get_register(static_cast<int32_t_register_rw>(raw_address));
	}
	else if(raw_address >= int32_t_register_r_start && raw_address <= int32_t_register_r_end){
		raw_value = device_get_register(static_cast<int32_t_register_r>(raw_address));
	}

	else if(raw_address >= uint8_t_register_rw_start && raw_address <= uint8_t_register_rw_end){
		raw_value = device_get_register(static_cast<uint8_t_register_rw>(raw_address));
	}
	else if(raw_address >= uint8_t_register_r_start && raw_address <= uint8_t_register_r_end){
		raw_value = device_get_register(static_cast<uint8_t_register_r>(raw_address));
	}

	else if(raw_address >= uint16_t_register_rw_start && raw_address <= uint16_t_register_rw_end){
		raw_value = device_get_register(static_cast<uint16_t_register_rw>(raw_address));
	}
	else if(raw_address >= uint16_t_register_r_start && raw_address <= uint16_t_register_r_end){
		raw_value = device_get_register(static_cast<uint16_t_register_r>(raw_address));
	}

	else if(raw_address >= uint32_t_register_rw_start && raw_address <= uint32_t_register_rw_end){
		raw_value = device_get_register(static_cast<uint32_t_register_rw>(raw_address));
	}
	else if(raw_address >= uint32_t_register_r_start && raw_address <= uint32_t_register_r_end){
		raw_value = device_get_register(static_cast<uint32_t_register_r>(raw_address));
	}

	else if(raw_address >= float_register_rw_start && raw_address <= float_register_rw_end){
		raw_value = device_get_register(static_cast<float_register_rw>(raw_address));
	}
	else if(raw_address >= float_register_r_start && raw_address <= float_register_r_end){
		raw_value = device_get_register(static_cast<float_register_r>(raw_address));
	}

	else{
		return controller_register_access_result::FAIL;
	}

	return controller_register_access_result::SUCCESS;
}


// int8 register get/sets
int8_t communication::device_get_register(int8_t_register_rw address){
	return int8_t_data_array[address - INT8_T_REGISTER_OFFSET];
}
int8_t communication::device_get_register(int8_t_register_r address){
	return int8_t_data_array[address - INT8_T_REGISTER_OFFSET];
}
int8_t communication::device_get_register(int8_t_register_w address){
	return int8_t_data_array[address - INT8_T_REGISTER_OFFSET];
}

void communication::device_set_register(int8_t_register_rw address, int8_t value){
	int8_t_data_array[address - INT8_T_REGISTER_OFFSET] = value;
}
void communication::device_set_register(int8_t_register_r address, int8_t value){
	int8_t_data_array[address - INT8_T_REGISTER_OFFSET] = value;
}
void communication::device_set_register(int8_t_register_w address, int8_t value){
	int8_t_data_array[address - INT8_T_REGISTER_OFFSET] = value;
}

// int16 register get/sets
int16_t communication::device_get_register(int16_t_register_rw address){
	return int16_t_data_array[address - INT16_T_REGISTER_OFFSET];
}
int16_t communication::device_get_register(int16_t_register_r address){
	return int16_t_data_array[address - INT16_T_REGISTER_OFFSET];
}
int16_t communication::device_get_register(int16_t_register_w address){
	return int16_t_data_array[address - INT16_T_REGISTER_OFFSET];
}

void communication::device_set_register(int16_t_register_rw address, int16_t value){
	int16_t_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}
void communication::device_set_register(int16_t_register_r address, int16_t value){
	int16_t_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}
void communication::device_set_register(int16_t_register_w address, int16_t value){
	int16_t_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}

// int32 register get/sets
int32_t communication::device_get_register(int32_t_register_rw address){
	return int32_t_data_array[address - INT16_T_REGISTER_OFFSET];
}
int32_t communication::device_get_register(int32_t_register_r address){
	return int32_t_data_array[address - INT16_T_REGISTER_OFFSET];
}
int32_t communication::device_get_register(int32_t_register_w address){
	return int32_t_data_array[address - INT16_T_REGISTER_OFFSET];
}

void communication::device_set_register(int32_t_register_rw address, int32_t value){
	int32_t_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}
void communication::device_set_register(int32_t_register_r address, int32_t value){
	int32_t_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}
void communication::device_set_register(int32_t_register_w address, int32_t value){
	int32_t_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}

// uint8 register get/sets
uint8_t communication::device_get_register(uint8_t_register_rw address){
	return uint8_t_data_array[address - INT16_T_REGISTER_OFFSET];
}
uint8_t communication::device_get_register(uint8_t_register_r address){
	return uint8_t_data_array[address - INT16_T_REGISTER_OFFSET];
}
uint8_t communication::device_get_register(uint8_t_register_w address){
	return uint8_t_data_array[address - INT16_T_REGISTER_OFFSET];
}

void communication::device_set_register(uint8_t_register_rw address, uint8_t value){
	uint8_t_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}
void communication::device_set_register(uint8_t_register_r address, uint8_t value){
	uint8_t_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}
void communication::device_set_register(uint8_t_register_w address, uint8_t value){
	uint8_t_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}

// uint16 register get/sets
uint16_t communication::device_get_register(uint16_t_register_rw address){
	return uint16_t_data_array[address - INT16_T_REGISTER_OFFSET];
}
uint16_t communication::device_get_register(uint16_t_register_r address){
	return uint16_t_data_array[address - INT16_T_REGISTER_OFFSET];
}
uint16_t communication::device_get_register(uint16_t_register_w address){
	return uint16_t_data_array[address - INT16_T_REGISTER_OFFSET];
}

void communication::device_set_register(uint16_t_register_rw address, uint16_t value){
	uint16_t_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}
void communication::device_set_register(uint16_t_register_r address, uint16_t value){
	uint16_t_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}
void communication::device_set_register(uint16_t_register_w address, uint16_t value){
	uint16_t_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}


// uint32 register get/sets
uint32_t communication::device_get_register(uint32_t_register_rw address){
	return uint32_t_data_array[address - INT16_T_REGISTER_OFFSET];
}
uint32_t communication::device_get_register(uint32_t_register_r address){
	return uint32_t_data_array[address - INT16_T_REGISTER_OFFSET];
}
uint32_t communication::device_get_register(uint32_t_register_w address){
	return uint32_t_data_array[address - INT16_T_REGISTER_OFFSET];
}

void communication::device_set_register(uint32_t_register_rw address, uint32_t value){
	uint32_t_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}
void communication::device_set_register(uint32_t_register_r address, uint32_t value){
	uint32_t_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}
void communication::device_set_register(uint32_t_register_w address, uint32_t value){
	uint32_t_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}

// int32 register get/sets
float communication::device_get_register(float_register_rw address){
	return float_data_array[address - INT16_T_REGISTER_OFFSET];
}
float communication::device_get_register(float_register_r address){
	return float_data_array[address - INT16_T_REGISTER_OFFSET];
}
float communication::device_get_register(float_register_w address){
	return float_data_array[address - INT16_T_REGISTER_OFFSET];
}

void communication::device_set_register(float_register_rw address, float value){
	float_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}
void communication::device_set_register(float_register_r address, float value){
	float_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}
void communication::device_set_register(float_register_w address, float value){
	float_data_array[address - INT16_T_REGISTER_OFFSET] = value;
}
