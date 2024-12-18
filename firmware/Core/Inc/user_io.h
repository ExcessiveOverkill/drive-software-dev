/**
  ******************************************************************************
  * @file           : user_io.h
  * @brief          : Header for user_io.cpp file.
  *                   user_io files contain the requred functions to setup and use the indicator LEDs and DIP switches on the drive.
  ******************************************************************************
**/

#pragma once

#include "stm32f413xx.h"
#include <assert.h>
#include "logging.h"


// Class for managing user IO hardware (LEDs and DIP switches)
class user_io{
    private:
        logging* logs;

        const uint32_t update_period_ms = 1000 / SYSTICK_FREQUENCY;  // Period the update() function will be called at

        uint32_t slow_blink_time_ms = 2000;   // half of the total period
        uint32_t medium_blink_time_ms = 500;  // half of the total period
        uint32_t fast_blink_time_ms = 100;    // half of the total period

        uint32_t slow_blink_cycle_counter = 0;
        uint32_t medium_blink_cycle_counter = 0;
        uint32_t fast_blink_cycle_counter = 0;

        uint8_t blink_state = 0b10000;  //states of each led mode  (bit 0:off 1:slow, 2:medium, 3:fast 4:on)

        uint32_t switch_state = 0;
        uint32_t led_state;
        uint8_t led_modes[4] = {0, 0, 0, 0};

        int32_t update_step = no_init;

        uint32_t error = 0;

        enum i2c_states{
          write_send_start_bit_wait,
          write_send_device_address_wait,
          write_send_register_address_wait,
          write_send_register_data_wait,
          write_done,

          read_send_start_bit_wait,
          read_send_device_address_wait,
          read_send_register_address_wait,
          read_send_repeat_start_bit_wait,
          read_send_repeat_device_address_wait,
          read_send_nak,
          read_done
        };

        enum update_states{
          no_init,
          idle,
          set_config0,
          set_inversion0,
          set_output0,
          get_input0,
          done
        };

        uint32_t i2c_state = write_done;

        uint8_t device_address = 0x20;
        uint8_t device_register = 0;
        uint8_t register_data = 0;

        void increment_update_step(void);

        void i2c_write(uint8_t device_address_, uint8_t device_register_, uint8_t register_data_);
        void i2c_read(uint8_t device_address, uint8_t device_register);

    public:

        // led mode enums
        enum led_mode_enum{
          off = 1,
          blink_slow = 2,
          blink_medium = 4,
          blink_fast = 8,
          on = 16
        };
        
        user_io(logging* logs);

        void init(void);
        
        uint32_t get_errors(void);

        uint32_t get_switch_states(void);

        void set_led_state(uint32_t led_select_, uint32_t led_mode_);

        void SysTick_Handler(void);

        void I2C1_EV_IRQHandler(void);

        void I2C1_ER_IRQHandler(void);
};
