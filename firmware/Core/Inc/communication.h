/**
  ******************************************************************************
  * @file           : communication.h
  * @brief          : Header for communication.cpp file.
  *                   communication files contain the requred functions to setup and use UART for communicating with the controller
  ******************************************************************************
**/

#pragma once

#include "stm32f413xx.h"
#include <assert.h>
#include <memory.h>
#include "device_descriptor.h"

#define MAX_PACKET_SIZE 4+CYCLIC_ADDRESS_COUNT
// Class for managing uart hardware
class communication{
    private:

        //TODO: find out if the 32bit array is even needed by the DMA
        union rx_data{
          uint32_t data_words[MAX_PACKET_SIZE];   // rx bytes are packed into this array by the DMA
          uint8_t data_bytes[MAX_PACKET_SIZE*4];  // same data as rx_data, but packed as bytes
        };
        
        union tx_data{
          uint32_t data_words[MAX_PACKET_SIZE];   // tx bytes are unpacked from this array by the DMA
          uint8_t data_bytes[MAX_PACKET_SIZE*4];  // same data as tx_data, but packed as bytes
        };

        rx_data rx;
        tx_data tx;

        uint8_t expected_rx_length = 4;   // 4 x 32bit words is the smallest possible packet
        uint8_t expected_tx_length = 4;   // 4 x 32bit words is the smallest possible packet
        uint8_t device_address = DEVICE_STARTING_ADDRESS;

        // sequential data transfer
        uint32_t sequential_register_address = 0;
        uint32_t sequential_register_data_input = 0;
        uint32_t sequential_register_data_response = 0;

        // cyclic data transfer
        // it is critical cyclic config registers are only modified while cyclic mode is disabled
        bool cyclic_mode_enabled = false;
        uint16_t cyclic_read_count = 0;
        uint16_t cyclic_write_count = 0;
        uint16_t cyclic_read_addresses[CYCLIC_ADDRESS_COUNT];
        uint16_t cyclic_read_sizes[CYCLIC_ADDRESS_COUNT];
        uint16_t cyclic_write_addresses[CYCLIC_ADDRESS_COUNT];
        uint16_t cyclic_write_sizes[CYCLIC_ADDRESS_COUNT];

        void set_tx_packet_length(uint32_t length);
        void set_rx_packet_length(uint32_t length);

        uint16_t packing_offset = 0;
        void pack_8_to_8_array(uint8_t data_in, uint16_t &offset, uint8_t *array);
        void pack_8_to_8_array(int8_t data_in, uint16_t &offset, uint8_t *array);
        void pack_16_to_8_array(uint16_t data_in, uint16_t &offset, uint8_t *array);
        void pack_16_to_8_array(int16_t data_in, uint16_t &offset, uint8_t *array);
        void pack_32_to_8_array(uint32_t data_in, uint16_t &offset, uint8_t *array);
        void pack_32_to_8_array(int32_t data_in, uint16_t &offset, uint8_t *array);
        void pack_float_to_8_array(float data_in, uint16_t &offset, uint8_t *array);

        uint8_t get_register_size(uint16_t raw_address);
        uint32_t calculate_crc(uint32_t *data, uint8_t data_length);
        bool interpret_rx_packet();
        void generate_tx_packet();

        enum controller_register_access_result: uint8_t{
          SUCCESS,
          FAIL  // invalid address or permissions
        };

        //errors
        bool error_reading_cyclic_address = false;

        controller_register_access_result controller_get_register(uint16_t raw_address, uint32_t &raw_value);
        controller_register_access_result controller_set_register(uint16_t raw_address, uint32_t &raw_value);

    public:

        bool receive_complete = false;
        bool receive_started = false;

        communication(uint32_t i);
        
        void init(void);

        void start_receive(void);
        bool rx_idle_detected(void);
        void clear_rx_idle_flag(void);
        void start_transmit(void);

        void restart_rx_dma(void);

        void dma_stream1_interrupt_handler(void);
        void usart6_interrupt_handler(void);

        // setup access to registers from device side
        // the DEVICE may read/write to ALL registers, regardless of their read/write setting in device_descriptor.h
        // the CONTROLLER however can only read/write to/from the register if the permission is set
        int8_t device_get_register(int8_t_register_rw address);
        int8_t device_get_register(int8_t_register_r address);
        int8_t device_get_register(int8_t_register_w address);
        void device_set_register(int8_t_register_rw address, int8_t value);
        void device_set_register(int8_t_register_r address, int8_t value);
        void device_set_register(int8_t_register_w address, int8_t value);

        int16_t device_get_register(int16_t_register_rw address);
        int16_t device_get_register(int16_t_register_r address);
        int16_t device_get_register(int16_t_register_w address);
        void device_set_register(int16_t_register_rw address, int16_t value);
        void device_set_register(int16_t_register_r address, int16_t value);
        void device_set_register(int16_t_register_w address, int16_t value);

        int32_t device_get_register(int32_t_register_rw address);
        int32_t device_get_register(int32_t_register_r address);
        int32_t device_get_register(int32_t_register_w address);
        void device_set_register(int32_t_register_rw address, int32_t value);
        void device_set_register(int32_t_register_r address, int32_t value);
        void device_set_register(int32_t_register_w address, int32_t value);

        uint8_t device_get_register(uint8_t_register_rw address);
        uint8_t device_get_register(uint8_t_register_r address);
        uint8_t device_get_register(uint8_t_register_w address);
        void device_set_register(uint8_t_register_rw address, uint8_t value);
        void device_set_register(uint8_t_register_r address, uint8_t value);
        void device_set_register(uint8_t_register_w address, uint8_t value);

        uint16_t device_get_register(uint16_t_register_rw address);
        uint16_t device_get_register(uint16_t_register_r address);
        uint16_t device_get_register(uint16_t_register_w address);
        void device_set_register(uint16_t_register_rw address, uint16_t value);
        void device_set_register(uint16_t_register_r address, uint16_t value);
        void device_set_register(uint16_t_register_w address, uint16_t value);

        uint32_t device_get_register(uint32_t_register_rw address);
        uint32_t device_get_register(uint32_t_register_r address);
        uint32_t device_get_register(uint32_t_register_w address);
        void device_set_register(uint32_t_register_rw address, uint32_t value);
        void device_set_register(uint32_t_register_r address, uint32_t value);
        void device_set_register(uint32_t_register_w address, uint32_t value);

        float device_get_register(float_register_rw address);
        float device_get_register(float_register_r address);
        float device_get_register(float_register_w address);
        void device_set_register(float_register_rw address, float value);
        void device_set_register(float_register_r address, float value);
        void device_set_register(float_register_w address, float value);

        void debug();

        //uint32_t get_errors(void);

};