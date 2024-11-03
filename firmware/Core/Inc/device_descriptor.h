
//////////////////// AUTO GENERATED FILE ////////////////////

// do not manually modify this file, modify "device_descriptor.yaml" instead
// run autogen.py to update (called at each build by default)

// last updated at 2024-11-03 12:25:07.643178

#pragma once


#define HARDWARE_TYPE 0
#define HARDWARE_VERSION 0
#define FIRMWARE_VERSION 0


#define SHUNT_RESISTANCE 0.005		// (ohms) Value of the resistors used to measure phase current. These may be changed to better support lower current motors but it is not recomended.
#define MIN_DC_BUS_VOLTAGE 24		// (volts) Undervoltage protection will trip if the bus drops below this value.
#define MAX_DC_BUS_VOLTAGE 400		// (volts) Overvoltage protection will trip if the bus rises above this value.
#define MAX_PHASE_CURRENT 10000		// (milliamps) Maximum current allowed through an output phase before overcurrent protection is tripped.
#define HEATSINK_OVER_TEMP_LIMIT 50		// (celsius) Heatsink temperature at which overheat protection will trip
#define BOARD_OVER_TEMP_LIMIT 50		// (celsius) PCB temperature at which overheat protection will trip
#define SYSCLK 100		// (Mhz) System clock frequency
#define SYSTICK_FREQUENCY 100		// (Hz) System tick frequency, do not set above 1khz
#define DEADTIME 100		// (ns) Phase PWM deadtime to prevent high and low side fets being on at the same time
#define PWMCLK 25		// (khz) Phase PWM output frequency
#define ADC_ENABLE_DELAY_CYCLES 30		// (count) How many cycles it takes for the ADCs to power on after phase PWM is enabled
#define ADCCLK 20		// (MHz) Phase ADC clock frequency
#define ADC_VOLTAGE 0.25		// (volts) ADC optimal shunt input voltage range
#define ADC_MIN_VALUE_PERCENT 0.1094
#define DFSDM_DIVISIONS 8388608		// (count) 
#define DFSDM_SHORT_CIRCUIT_BIT_COUNT 127		// (count) Number of consecutive same bits to receive from the ADC before triggering short circuit detection. Max is 127 (ADCs send a different bit every 128 cycles even at full scale)
#define DEVICE_STARTING_ADDRESS 127		// (address) rs422 to address to start at, DIP switches may be used to increment it
#define CYCLIC_ADDRESS_COUNT 32		// (count) Maximum number of cyclic addresses that can be configured


#define INT8_T_REGISTER_COUNT 0
#define INT16_T_REGISTER_COUNT 0
#define INT32_T_REGISTER_COUNT 4
#define UINT8_T_REGISTER_COUNT 2
#define UINT16_T_REGISTER_COUNT 68
#define UINT32_T_REGISTER_COUNT 4
#define FLOAT_REGISTER_COUNT 3

#define INT8_T_REGISTER_OFFSET 0
#define INT16_T_REGISTER_OFFSET 0
#define INT32_T_REGISTER_OFFSET 0
#define UINT8_T_REGISTER_OFFSET 4
#define UINT16_T_REGISTER_OFFSET 6
#define UINT32_T_REGISTER_OFFSET 74
#define FLOAT_REGISTER_OFFSET 78

inline int8_t int8_t_data_array[1] = {0};
inline int16_t int16_t_data_array[1] = {0};
inline int32_t int32_t_data_array[4] = {0, 0, 0, 0};
inline uint8_t uint8_t_data_array[2] = {0, 0};
inline uint16_t uint16_t_data_array[68] = {0, 50, 1, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 65535, 0};
inline uint32_t uint32_t_data_array[4] = {0, 0, 0, 0};
inline float float_data_array[3] = {0.0, 0.0, 0.0};


// link each register to a global address
enum int8_t_register_rw: uint16_t{		// read/write int8_t registers
	int8_t_register_rw_start = 0,
	int8_t_register_rw_end = 0,
};
enum int8_t_register_r: uint16_t{		// read int8_t registers
	int8_t_register_r_start = 1,
	int8_t_register_r_end = 1,
};
enum int8_t_register_w: uint16_t{		// write int8_t registers
	int8_t_register_w_start = 2,
	int8_t_register_w_end = 2,
};

enum int16_t_register_rw: uint16_t{		// read/write int16_t registers
	int16_t_register_rw_start = 3,
	int16_t_register_rw_end = 3,
};
enum int16_t_register_r: uint16_t{		// read int16_t registers
	int16_t_register_r_start = 4,
	int16_t_register_r_end = 4,
};
enum int16_t_register_w: uint16_t{		// write int16_t registers
	int16_t_register_w_start = 5,
	int16_t_register_w_end = 5,
};

enum int32_t_register_rw: uint16_t{		// read/write int32_t registers
	int32_t_register_rw_start = 6,
	current_command_q_register = 6,		// (milliamps) q setpoint for the internal current control
	current_command_d_register = 7,		// (milliamps) d setpoint for the internal current control
	int32_t_register_rw_end = 7,
};
enum int32_t_register_r: uint16_t{		// read int32_t registers
	int32_t_register_r_start = 8,
	current_measured_q_register = 8,		// (milliamps) Actual measured q current through motor
	current_measured_d_register = 9,		// (milliamps) Actual measured d current through motor
	int32_t_register_r_end = 9,
};
enum int32_t_register_w: uint16_t{		// write int32_t registers
	int32_t_register_w_start = 10,
	int32_t_register_w_end = 10,
};

enum uint8_t_register_rw: uint16_t{		// read/write uint8_t registers
	uint8_t_register_rw_start = 11,
	requested_state_register = 11,		// FAULT: 1 IDLE: 2 RUN: 3
	enable_cyclic_data_register = 12,		// Enable sending/receiving configured cyclic data
	uint8_t_register_rw_end = 12,
};
enum uint8_t_register_r: uint16_t{		// read uint8_t registers
	uint8_t_register_r_start = 13,
	uint8_t_register_r_end = 13,
};
enum uint8_t_register_w: uint16_t{		// write uint8_t registers
	uint8_t_register_w_start = 14,
	uint8_t_register_w_end = 14,
};

enum uint16_t_register_rw: uint16_t{		// read/write uint16_t registers
	uint16_t_register_rw_start = 15,
	commutation_command_register = 15,		// (counts) Value 0-65525 representing the current electrical angle of the motor
	max_ouput_voltage_register = 16,		// (volts) Maximum output voltage (PWM dutycycle)
	device_communication_watchdog_timeout_register = 17,		// (milliseconds) How long before a timeout occurs due to no valid packet transfers
	cyclic_read_address_0_register = 18,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_1_register = 19,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_2_register = 20,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_3_register = 21,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_4_register = 22,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_5_register = 23,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_6_register = 24,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_7_register = 25,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_8_register = 26,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_9_register = 27,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_10_register = 28,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_11_register = 29,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_12_register = 30,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_13_register = 31,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_14_register = 32,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_15_register = 33,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_16_register = 34,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_17_register = 35,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_18_register = 36,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_19_register = 37,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_20_register = 38,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_21_register = 39,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_22_register = 40,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_23_register = 41,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_24_register = 42,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_25_register = 43,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_26_register = 44,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_27_register = 45,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_28_register = 46,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_29_register = 47,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_30_register = 48,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_31_register = 49,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_write_address_0_register = 50,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_1_register = 51,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_2_register = 52,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_3_register = 53,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_4_register = 54,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_5_register = 55,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_6_register = 56,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_7_register = 57,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_8_register = 58,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_9_register = 59,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_10_register = 60,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_11_register = 61,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_12_register = 62,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_13_register = 63,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_14_register = 64,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_15_register = 65,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_16_register = 66,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_17_register = 67,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_18_register = 68,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_19_register = 69,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_20_register = 70,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_21_register = 71,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_22_register = 72,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_23_register = 73,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_24_register = 74,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_25_register = 75,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_26_register = 76,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_27_register = 77,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_28_register = 78,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_29_register = 79,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_30_register = 80,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_31_register = 81,		// (address) Set to a register address to enable continuous cyclic writing
	uint16_t_register_rw_end = 81,
};
enum uint16_t_register_r: uint16_t{		// read uint16_t registers
	uint16_t_register_r_start = 82,
	dc_bus_voltage_register = 82,		// (volts) Measured DC bus voltage
	uint16_t_register_r_end = 82,
};
enum uint16_t_register_w: uint16_t{		// write uint16_t registers
	uint16_t_register_w_start = 83,
	uint16_t_register_w_end = 83,
};

enum uint32_t_register_rw: uint16_t{		// read/write uint32_t registers
	uint32_t_register_rw_start = 84,
	firmware_update_data_0_register = 84,		// (binary data) Used to transfer firmware data during a device firmware update
	firmware_update_data_1_register = 85,		// (binary data) Used to transfer firmware data during a device firmware update
	firmware_update_data_2_register = 86,		// (binary data) Used to transfer firmware data during a device firmware update
	firmware_update_data_3_register = 87,		// (binary data) Used to transfer firmware data during a device firmware update
	uint32_t_register_rw_end = 87,
};
enum uint32_t_register_r: uint16_t{		// read uint32_t registers
	uint32_t_register_r_start = 88,
	uint32_t_register_r_end = 88,
};
enum uint32_t_register_w: uint16_t{		// write uint32_t registers
	uint32_t_register_w_start = 89,
	uint32_t_register_w_end = 89,
};

enum float_register_rw: uint16_t{		// read/write float registers
	float_register_rw_start = 90,
	current_loop_p_gain_register = 90,		// Proportional gain for current loop
	current_loop_i_gain_register = 91,		// Integral gain for current loop
	current_loop_i_limit_register = 92,		// Integral limit for current loop
	float_register_rw_end = 92,
};
enum float_register_r: uint16_t{		// read float registers
	float_register_r_start = 93,
	float_register_r_end = 93,
};
enum float_register_w: uint16_t{		// write float registers
	float_register_w_start = 94,
	float_register_w_end = 94,
};
