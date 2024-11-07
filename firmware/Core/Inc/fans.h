/**
  ******************************************************************************
  * @file           : fans.h
  * @brief          : Header for fans.cpp file.
  *                   contains the required functions to setup and use the fans connected to 
  ******************************************************************************
**/

// 25kHz
// TIM4 CH1


#define error1 0b001
#define error2 0b010


#pragma once

#include "stm32f413xx.h"
#include <assert.h>
#include "device_descriptor.h"

#define PWM_ticks 1000*SYSCLK/PWMCLK/2

class fans{
    private:

        fans_error_t error = fans_error_t::no_error;

    public:

        fans(uint32_t i);
        
        void init(void);

        void enable(void);

        void disable(void);

        void set_speed(uint32_t speed_rpm);

        uint32_t get_speed_1(void);

        uint32_t get_speed_2(void);

        uint32_t get_errors(void);

  
};




