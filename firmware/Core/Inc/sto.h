/**
  ******************************************************************************
  * @file           : sto.h
  * @brief          : Header for sto.cpp file.
  *                   sto files contain the requred functions to setup and use STO inputs/outputs
  ******************************************************************************
**/

#pragma once

#include "stm32f413xx.h"
#include <assert.h>

// Class for managing STO hardware
class sto{
    private:


    public:

        sto(uint32_t i);
        
        // severity init(void);

        // severity enable(void);

        // severity disable(void);

        bool output_allowed(void);

        uint32_t check_fault(void);

        //uint32_t get_errors(void);

};