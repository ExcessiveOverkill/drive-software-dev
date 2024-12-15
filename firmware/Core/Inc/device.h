#include "mode_base.h"
// other modes should be included here

#include "device_descriptor.h"



// main device class that contains all the other classes

class device {

    public:
        void run(); // main loop, should be called from main()

        // interrupt handlers
        void SysTick_Handler();
        // TODO: add other interrupt handlers


    private:

        communication Comm; // communication with the controller
        errors Errors; // error/warning handling
        fans Fans; // fan control
        user_io UserIO; // user interface (DIP switches and LEDs)
        current_sense_interface CurrentSense; // current sensing
        phase_pwm PhasePWM; // PWM generation
        adc_interface Adc; // ADC sampling
        sto Sto; // safe torque off
        // other global/hardware classes should be added here
        

        Mode* modes[MODE_COUNT]; // array of all modes
        Mode* current_mode; // pointer to the current mode

        void init(); // initialize all hardware and modes

        void update(); // low frequency update, called from SysTick_Handler

        void critical_shutdown(); // immediately disable system and enter a safe state

        void enter_mode(Mode* mode); // enter a new mode

};

