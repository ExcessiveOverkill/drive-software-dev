/**
  ******************************************************************************
  * @file           : errors.h
  * @brief          : Header for errors.cpp file.
  *                   error files contain the requred functions to setup and use error handling
  ******************************************************************************
**/


#pragma once

#include "device_descriptor.h"
#include "stm32f413xx.h"


class errors {
public:
    enum class error_severity : unsigned int {
        WARNING = 0b001 << 24,
        ERROR = 0b010 << 24,
        CRITICAL = 0b100 << 24
    };

    
    enum class error_types : unsigned int {   // will later be defined by device_descriptor.h
        FAN_SPEED_LOW = 0 | uint32_t(error_severity::WARNING),
        FAN_SPEED_ZERO = 1 | uint32_t(error_severity::WARNING),
        FAN_TIMER_FAULT = 2 | uint32_t(error_severity::ERROR)
    };
    

private:
    static const int error_types_count = 3;  // will later be defined by device_descriptor.h
    unsigned int last_active_error_times_millis[error_types_count];
    unsigned int last_ok_error_times_millis[error_types_count];
    unsigned int active_severity = 0;
    unsigned int last_reset_time_millis = 0;

public:
    error_severity get_active_severity(void);
    unsigned int get_last_reset_time_millis(void);
    unsigned int get_last_active_error_time_millis(error_types error_type);
    void clear_all_errors(void);
    error_severity add_error(error_types error_type);
    error_severity persistent_error_active(error_types error_type);
    error_severity persistent_error_inactive(error_types error_type);
};
