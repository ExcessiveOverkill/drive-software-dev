
//////////////////// AUTO GENERATED FILE ////////////////////

// do not manually modify this file, modify "device_descriptor.yaml" instead
// run autogen.py to update (called at each build by default)

// last updated at 2024-07-19 19:31:34.135087

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DEVICE_DESCRIPTOR_H
#define __DEVICE_DESCRIPTOR_H


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
#define MAX_PACKET_SIZE 36		// (32bit words) Maximum expected communication packet size
#define DEVICE_STARTING_ADDRESS 127		// (address) rs422 to address to start at, DIP switches may be used to increment it


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
	current_command_q = 6,		// (milliamps) q setpoint for the internal current control
	current_command_d = 7,		// (milliamps) d setpoint for the internal current control
	int32_t_register_rw_end = 7,
};
enum int32_t_register_r: uint16_t{		// read int32_t registers
	int32_t_register_r_start = 8,
	current_measured_q = 8,		// (milliamps) Actual measured q current through motor
	current_measured_d = 9,		// (milliamps) Actual measured d current through motor
	int32_t_register_r_end = 9,
};
enum int32_t_register_w: uint16_t{		// write int32_t registers
	int32_t_register_w_start = 10,
	int32_t_register_w_end = 10,
};

enum uint8_t_register_rw: uint16_t{		// read/write uint8_t registers
	uint8_t_register_rw_start = 11,
	requested_state = 11,		// FAULT: 1 IDLE: 2 RUN: 3
	enable_cyclic_data = 12,		// Enable sending/receiving configured cyclic data
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
	commutation_command = 15,		// Value 0-65525 representing the current electrical angle of the motor
	max_ouput_voltage = 16,		// (volts) Maximum output voltage (PWM dutycycle)
	device_communication_watchdog_timeout = 17,		// (milliseconds) How long before a timeout occurs due to no valid packet transfers
	cyclic_read_address_0 = 18,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_1 = 19,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_2 = 20,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_3 = 21,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_4 = 22,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_5 = 23,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_6 = 24,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_7 = 25,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_8 = 26,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_9 = 27,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_10 = 28,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_11 = 29,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_12 = 30,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_13 = 31,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_14 = 32,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_15 = 33,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_16 = 34,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_17 = 35,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_18 = 36,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_19 = 37,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_20 = 38,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_21 = 39,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_22 = 40,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_23 = 41,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_24 = 42,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_25 = 43,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_26 = 44,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_27 = 45,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_28 = 46,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_29 = 47,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_30 = 48,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_read_address_31 = 49,		// (address) Set to a register address to enable continuous cyclic reading
	cyclic_write_address_0 = 50,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_1 = 51,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_2 = 52,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_3 = 53,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_4 = 54,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_5 = 55,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_6 = 56,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_7 = 57,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_8 = 58,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_9 = 59,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_10 = 60,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_11 = 61,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_12 = 62,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_13 = 63,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_14 = 64,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_15 = 65,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_16 = 66,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_17 = 67,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_18 = 68,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_19 = 69,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_20 = 70,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_21 = 71,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_22 = 72,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_23 = 73,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_24 = 74,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_25 = 75,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_26 = 76,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_27 = 77,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_28 = 78,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_29 = 79,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_30 = 80,		// (address) Set to a register address to enable continuous cyclic writing
	cyclic_write_address_31 = 81,		// (address) Set to a register address to enable continuous cyclic writing
	uint16_t_register_rw_end = 81,
};
enum uint16_t_register_r: uint16_t{		// read uint16_t registers
	uint16_t_register_r_start = 82,
	dc_bus_voltage = 82,		// (volts) Measured DC bus voltage
	uint16_t_register_r_end = 82,
};
enum uint16_t_register_w: uint16_t{		// write uint16_t registers
	uint16_t_register_w_start = 83,
	uint16_t_register_w_end = 83,
};

enum uint32_t_register_rw: uint16_t{		// read/write uint32_t registers
	uint32_t_register_rw_start = 84,
	firmware_update_data_0 = 84,		// (binary data) Used to transfer firmware data during a device firmware update
	firmware_update_data_1 = 85,		// (binary data) Used to transfer firmware data during a device firmware update
	firmware_update_data_2 = 86,		// (binary data) Used to transfer firmware data during a device firmware update
	firmware_update_data_3 = 87,		// (binary data) Used to transfer firmware data during a device firmware update
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
	current_loop_p_gain = 90,		// Proportional gain for current loop
	current_loop_i_gain = 91,		// Integral gain for current loop
	current_loop_i_limit = 92,		// Integral limit for current loop
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


#endif