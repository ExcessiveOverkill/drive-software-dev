/**
  ******************************************************************************
  * @file           : communication.h
  * @brief          : Header for communication.cpp file.
  *                   communication files contain the requred functions to setup and use UART for communicating with the controller
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#include "stm32f413xx.h"
#include <assert.h>
#include <memory.h>
#include "device_descriptor.h"

// Class for managing uart hardware
class communication{
    private:
        uint32_t rx_data[MAX_PACKET_SIZE];   // rx bytes are packed into this array by the DMA
        uint32_t tx_data[MAX_PACKET_SIZE];   // tx bytes are unpacked from this array by the DMA

        uint8_t expected_rx_length = 4;   // 4 x 32bit words is the smallest possible packet
        uint8_t expected_tx_length = 4;   // 4 x 32bit words is the smallest possible packet
        uint8_t device_address = DEVICE_STARTING_ADDRESS;

        void set_tx_packet_length(uint32_t length);
        void set_rx_packet_length(uint32_t length);

        uint32_t calculate_crc(uint32_t *data, uint8_t data_length);
        bool interpret_rx_packet();

        enum controller_register_access_result: uint8_t{
          SUCCESS,
          FAIL  // invalid address or permissions
        };

        controller_register_access_result controller_get_register(uint16_t raw_address, uint32_t &raw_value);
        controller_register_access_result controller_set_register(uint16_t raw_address, uint32_t &raw_value);

        uint16_t commutation_angle = 0;

    public:

        bool receive_complete = false;
        bool receive_started = false;

        communication(uint32_t i);
        
        void init(void);

        uint32_t get_commutaion_angle();
        int16_t get_current_command_milliamps();

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

        //uint32_t get_errors(void);

};

#endif