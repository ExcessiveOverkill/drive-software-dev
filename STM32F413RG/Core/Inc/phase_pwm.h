/**
  ******************************************************************************
  * @file           : phase_pwm.h
  * @brief          : Header for phase_pwm.cpp file.
  *                   phase_pwm files contain the requred functions to setup and use the TIM1 peripheral for outputting phase PWM
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PHASE_PWM_H
#define __PHASE_PWM_H

#include "stm32f413xx.h"
#include <assert.h>
#include "main.h"
#include "device_descriptor.h"
#define PWM_ticks 1000*SYSCLK/PWMCLK/2

// Class for managing phase outpuit PWM hardware
class phase_pwm{
    private:


    public:

        phase_pwm(uint32_t i);
        
        void init(void);

        void enable(void);

        void disable(void);

        bool break_flag_triggered(void);

        void clear_break_flag(void);

        void set(uint32_t U, uint32_t V, uint32_t W);

        //uint32_t get_errors(void);

};

#endif