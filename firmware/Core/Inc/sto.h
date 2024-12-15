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
#include "logging.h"

// Class for managing STO hardware
class sto{
    private:

        uint16_t class_identifer = 0x0510;
        
        logging* log;

        void log_message(uint32_t message);

    public:

        sto(uint32_t i);
        
        bool output_allowed(void);

        uint32_t check_fault(void);

};