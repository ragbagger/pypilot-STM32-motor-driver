/*
 * Author: Brad Kauffman
 * Date: 12/1/2023
 * Credits: Where they are due: https://github.com/seandepagnier is the original author of the Pypilot motor code.
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__





enum commands {COMMAND_CODE=0xc7, RESET_CODE=0xe7, MAX_CURRENT_CODE=0x1e, MAX_CONTROLLER_TEMP_CODE=0xa4, MAX_MOTOR_TEMP_CODE=0x5a, RUDDER_RANGE_CODE=0xb6, RUDDER_MIN_CODE=0x2b, RUDDER_MAX_CODE=0x4d, REPROGRAM_CODE=0x19, DISENGAGE_CODE=0x68, MAX_SLEW_CODE=0x71, EEPROM_READ_CODE=0x91, EEPROM_WRITE_CODE=0x53};

enum results {CURRENT_CODE=0x1c, VOLTAGE_CODE=0xb3, CONTROLLER_TEMP_CODE=0xf9, MOTOR_TEMP_CODE=0x48, RUDDER_SENSE_CODE=0xa7, FLAGS_CODE=0x8f, EEPROM_VALUE_CODE=0x9a};

enum {SYNC=1, OVERTEMP_FAULT=2, OVERCURRENT_FAULT=4, ENGAGED=8, INVALID=16*1, PORT_PIN_FAULT=16*2, STARBOARD_PIN_FAULT=16*4, BADVOLTAGE_FAULT=16*8, MIN_RUDDER_FAULT=256*1, MAX_RUDDER_FAULT=256*2, CURRENT_RANGE=256*4, BAD_FUSES=256*8};


//enum {CONTROLLER_TEMP, MOTOR_TEMP};
enum {CURRENT, VOLTAGE, CONTROLLER_TEMP, MOTOR_TEMP, RUDDER, CHANNEL_COUNT};

// Debug removal of watchdog
//comment out for production use
//#define NO_WATCH_DOG
/*
 * ATTACHED SENSORS
 */
#define DISABLE_TEMP_SENSE    // if no temp sensors avoid errors
#define DISABLE_VOLTAGE_SENSE // if no voltage sense
#define DISABLE_CURRENT_SENSE // if no motor current sensor is installed or used
//#define DISABLE_RUDDER_SENSE  // if no rudder sense
#define DISABLE_ENDSTOPS // if no endstops are installed we won't have a forward and reverse faults

#define DISABLE_DEBUGGING_DISPLAY // If a debugging TFT display is attached to the controller, comment this out


/*
 * Current configuration
 */
#define LOW_CURRENT // gives 2000 mA. Comment to get 4000 mA max current


// *************************************************************************************************** //
// ********************************* GENERAL CONFIGURATION VALUES ************************************ //
// *************************************************************************************************** //
#define PWM_DEADBAND 8
// These rudder values are set at startup. After that they are set by commands from pypilot
// Setting these to what pypilot supplies will give the corrrect values out of a reset
#define RUDDER_MIN 1192 // The minimum value at which RUDDER_MIN flag is being set
#define RUDDER_MAX 2771 // The maximum value at which RUDDER_MAX flag is being set
#define TEMPERATURE_CONTROLLER_MAX 7000 // 70 deg C
#define TEMPERATURE_MOTOR_MAX 7000 // 70 deg C
#define CURRENT_MOTOR_MAX 2000 // don't allow more than 20Amps
#define SPEEDUP_SLEW_RATE 2 // Slew rate when accelerating the motor / pump / linear actuator
#define SLOWDOWN_SLEW_RATE 2 // Slew rate when decelerating the motor / pump / linear actuator

//Analog exponential filter coefficients
// A lower value will provide more filtering but a longer reaction time

#define ALPHA_RUDDER 0.15f
#define ALPHA_VOLTAGE 0.15f
#define ALPHA_CURRENT 0.3f
#define ALPHA_TEMPCNTRL 0.15f
#define ALPHA_TEMPMOTOR 0.15f


// *************************************************************************************************** //
// ********************************* Thermocouple Variables ****************************************** //
// *************************************************************************************************** //
// Check https://learn.adafruit.com/thermistor/using-a-thermistor for more info
// resistance at 25 degrees C
#define THERMISTORNOMINAL 100000 // 100k thermistor
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 4700 // 4.7k resistor

// *************************************************************************************************** //
// ********************************* Battery Voltage Measurement Variables *************************** //
// *************************************************************************************************** //
// Maximum input voltage on the supply pin
#define VIN_MAX 1800
// Minimum input voltage on the supply pin
#define VIN_MIN 900
// Wow the ADC is configured to measure: 0V - 5V
#define ADC_RANGE 5.0f
// Bit deapth of the ADC being used
#define BIT_DEPTH 1024.0f
// Calculated voltage separation per bit
#define V_SEPARATION ADC_RANGE / BIT_DEPTH
// Voltage divider resistor 1
#define R1_1 22000.0f
// Voltage divider resistor 2
#define R2_1 4700.0f
// Constant calculated from resistor values for simplified voltage calculation
#define RESISTOR_CONSTANT_1 (R2_1/(R1_1 + R2_1))

// *************************************************************************************************** //
// ********************************* Motor Current Measurement Variables ***************************** //
// *************************************************************************************************** //
// Maximum input voltage on the supply pin
#define A_MAX 4300
// Voltage divider resistor 1
#define R1_2 47000.0f
// Voltage divider resistor 2
#define R2_2 150000.0f
// Constant calculated from resistor values for simplified voltage calculation
#define RESISTOR_CONSTANT_2 (R2_1/(R1_1 + R2_1))

#endif
