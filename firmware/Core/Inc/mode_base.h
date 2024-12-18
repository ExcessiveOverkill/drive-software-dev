#include "logging.h"
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

        message_severities update();  // run an update, called from the SYS_TICK handler at low frequency (high frequency updates should be triggered by interrupts)

        message_severities enter_mode();  // enable the mode, called when the mode is entered
        message_severities exit_mode();  // disable the mode, called when the mode is exited

        message_severities enter_idle();  // enable the idle state
        message_severities enter_run();  // enable the run state

    protected:

        // access to hardware classes
        fans* Fans;
        user_io* UserIO;
        current_sense_interface* CurrentSense;
        phase_pwm* PhasePWM;
        adc_interface* Adc;
        sto* Sto;
        
        logging* logs;     // access to message handling

        communication* Comm;    // access to communication, this allows access to all registers accessible by the controller

};