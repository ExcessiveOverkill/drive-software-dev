
//////////////////// AUTO GENERATED FILE ////////////////////

// run autogen.py to update (called at each build by default)

// last updated at 2024-07-18 18:07:30.589633

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEVICE_DESCRIPTOR_H
#define __DEVICE_DESCRIPTOR_H


#define HARDWARE_TYPE 0
#define HARDWARE_VERSION 0
#define FIRMWARE_VERSION 0


#define CURRENT_SHUNT_RESISTANCE 5		// (milliohms) Value of the resistors used to measure phase current. These may be changed to better support lower current motors but it is not recomended.
#define MIN_DC_BUS_VOLTAGE 24		// (volts) Undervoltage protection will trip if the bus drops below this value.
#define MAX_DC_BUS_VOLTAGE 400		// (volts) Overvoltage protection will trip if the bus rises above this value.
#define MAX_PHASE_CURRENT 10000		// (milliamps) Maximum current allowed through an output phase before overcurrent protection is tripped.
#define HEATSINK_OVER_TEMP_LIMIT 50		// (celsius) Heatsink temperature at which overheat protection will trip
#define BOARD_OVER_TEMP_LIMIT 50		// (celsius) PCB temperature at which overheat protection will trip


#define INT8_T_REGISTER_COUNT 0
#define INT16_T_REGISTER_COUNT 0
#define INT32_T_REGISTER_COUNT 4
#define UINT8_T_REGISTER_COUNT 1
#define UINT16_T_REGISTER_COUNT 3
#define UINT32_T_REGISTER_COUNT 0
#define FLOAT_REGISTER_COUNT 3


// link each register to a global address
enum int8_t_rw: uint16_t{		// read/write int8_t registers
};
enum int8_t_r: uint16_t{		// read int8_t registers
};
enum int8_t_w: uint16_t{		// write int8_t registers
};

enum int16_t_rw: uint16_t{		// read/write int16_t registers
};
enum int16_t_r: uint16_t{		// read int16_t registers
};
enum int16_t_w: uint16_t{		// write int16_t registers
};

enum int32_t_rw: uint16_t{		// read/write int32_t registers
	current_command_q = 0,		// (milliamps) q setpoint for the internal current control
	current_command_d = 1,		// (milliamps) d setpoint for the internal current control
};
enum int32_t_r: uint16_t{		// read int32_t registers
	current_measured_q = 2,		// (milliamps) Actual measured q current through motor
	current_measured_d = 3,		// (milliamps) Actual measured d current through motor
};
enum int32_t_w: uint16_t{		// write int32_t registers
};

enum uint8_t_rw: uint16_t{		// read/write uint8_t registers
	requested_state = 4,		// () FAULT: 1 IDLE: 2 RUN: 3
};
enum uint8_t_r: uint16_t{		// read uint8_t registers
};
enum uint8_t_w: uint16_t{		// write uint8_t registers
};

enum uint16_t_rw: uint16_t{		// read/write uint16_t registers
	commutation_command = 5,		// () Value 0-65525 representing the current electrical angle of the motor
	max_ouput_voltage = 6,		// (volts) Maximum output voltage (PWM dutycycle)
};
enum uint16_t_r: uint16_t{		// read uint16_t registers
	dc_bus_voltage = 7,		// (volts) Measured DC bus voltage
};
enum uint16_t_w: uint16_t{		// write uint16_t registers
};

enum uint32_t_rw: uint16_t{		// read/write uint32_t registers
};
enum uint32_t_r: uint16_t{		// read uint32_t registers
};
enum uint32_t_w: uint16_t{		// write uint32_t registers
};

enum float_rw: uint16_t{		// read/write float registers
	current_loop_p_gain = 8,		// () Proportional gain for current loop
	current_loop_i_gain = 9,		// () Integral gain for current loop
	current_loop_i_limit = 10,		// () Integral limit for current loop
};
enum float_r: uint16_t{		// read float registers
};
enum float_w: uint16_t{		// write float registers
};


#endif