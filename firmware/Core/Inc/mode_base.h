#include "errors.h"
#include "fans.h"
#include "user_io.h"
#include "current_sense_interface.h"
#include "phase_pwm.h"
#include "adc_interface.h"
#include "sto.h"
#include "communication.h"

# pragma once
// base class for all modes
// all modes should inherit from this class
// contains all objects that the modes could need to access


class Mode {
    public:

        // interrupt handlers, these will be called from their respective interrupt handlers when the mode is enabled
        void TIM1_UP_TIM10_IRQHandler();
        // TODO: add other interrupt handlers

        errors::error_severity update();  // run an update, called from the SYS_TICK handler at low frequency (high frequency updates should be triggered by interrupts)

        errors::error_severity enter_mode();  // enable the mode, called when the mode is entered
        errors::error_severity exit_mode();  // disable the mode, called when the mode is exited

        errors::error_severity enter_idle();  // enable the idle state
        errors::error_severity enter_run();  // enable the run state

    protected:

        // access to hardware classes
        fans* Fans;
        user_io* UserIO;
        current_sense_interface* CurrentSense;
        phase_pwm* PhasePWM;
        adc_interface* Adc;
        sto* Sto;
        
        errors* Errors;     // access to error handling

        communication* Comm;    // access to communication, this allows access to all registers accessible by the controller






};