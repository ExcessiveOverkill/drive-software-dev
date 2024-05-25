/**
  ******************************************************************************
  * @file           : adc.h
  * @brief          : Header for adc.cpp file.
  *                   adc files contain the requred functions to setup and use the ADC to read various analog voltages
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADC_H
#define __ADC_H

#include "stm32f413xx.h"
#include <assert.h>


// Class for managing ADC hardware
class adc{
    private:
        

    public:

        adc();
        
        void init(void);
        
        uint32_t get_errors(void);

};

#endif