/**
  ******************************************************************************
  * @file           : sto.h
  * @brief          : Header for sto.cpp file.
  *                   sto files contain the requred functions to setup and use STO inputs/outputs
  ******************************************************************************
**/

#pramga once

#include "stm32f413xx.h"
#include <assert.h>
#include "main.h"

// Class for managing STO hardware
class sto{
    private:


    public:

        sto(uint32_t i);
        
        void init(void);

        void enable(void);

        void disable(void);

        bool output_allowed(void);

        uint32_t check_fault(void);

        //uint32_t get_errors(void);

};