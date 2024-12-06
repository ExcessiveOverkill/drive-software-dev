/**
  ******************************************************************************
  * @file           : fans.h
  * @brief          : Header for fans.cpp file.
  *                   contains the required functions to setup and use the fans connected to
  *                  San Ace 40 9GA0424P3H001 Fans 
  ******************************************************************************
**/

#pragma once

#include "stm32f413xx.h"
#include <assert.h>
#include "device_descriptor.h"
#include "errors.h"

#define PWM_ticks 1000*SYSCLK/PWMCLK/2

class fans{
    private:

      bool FAN_SPEED_HIGH   = errors::error_severity::WARNING;
      bool FAN_SPEED_LOW    = errors::error_severity::WARNING;
      bool FAN_SPEED_ZERO   = errors::error_severity::WARNING;
      bool FAN_TIMER_FAULT  = errors::error_severity::ERROR;

    public:

        fans(void);
        
        uint32_t set_speed(uint32_t speed_rpm);

        uint32_t get_speed_1(void);

        uint32_t get_speed_2(void);

        uint32_t get_errors(void);
};



