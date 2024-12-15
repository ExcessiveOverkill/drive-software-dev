/**
  ******************************************************************************
  * @file           : fans.h
  * @brief          : Header for fans.cpp file.
  *                   contains the required functions to setup and use San Ace 40 9GA0424P3H001 Fans 
  ******************************************************************************
**/

#pragma once

#include "stm32f413xx.h"
#include <assert.h>
#include "device_descriptor.h"
#include "logging.h"

#define PWM_ticks 1000*SYSCLK/PWMCLK/2

#define MESSAGE   0b0001 << 28
#define WARNING   0b0010 << 28
#define ERROR     0b0100 << 28
#define CRITICAL  0b1000 << 28


class fans{
    private:

      uint16_t class_identifer = 0xFA65;

      logging* logs;

      void log_message(uint32_t message);

      uint32_t set_speed_rpm = 0;
      uint16_t tachometer_1_rpm = 0;
      uint16_t tachometer_2_rpm = 0;

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

        void SysTick_Handler(uint32_t sysTick_counter);
};



