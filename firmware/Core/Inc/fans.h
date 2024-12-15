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

      bool FAN_SPEED_HIGH   = errors::severity::WARNING;
      bool FAN_SPEED_LOW    = errors::severity::WARNING;
      bool FAN_SPEED_ZERO   = errors::severity::WARNING;
      bool FAN_TIMER_FAULT  = errors::severity::ERROR;

      volatile uint16_t tachometer_1_value;
      volatile uint16_t tachometer_2_value;

      void configure_GPIOB6_for_PWM(void);
      void configure_TIM4_for_PWM(void);

      void configure_GPIOC6_for_tachometer(void);
      void configure_TIM3_for_tachometer(void);

      void configure_GPIOC7_for_tachometer(void);
      void configure_TIM8_for_tachometer(void);

    public:

        fans(void);
        
        uint32_t set_speed(uint32_t speed_rpm);

        uint32_t get_fan_1_speed(void);

        uint32_t get_fan_2_speed(void);

        uint32_t get_errors(void);
};



