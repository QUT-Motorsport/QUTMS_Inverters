/**
 * @brief Defines and outlines all varibles and definitions related to the error system.
 * 
 * @file chassisError.h
 * @author Jonn Dillon
 * @date 2018-04-16
 */

#ifndef CHASSIS_ERROR_H
#define CHASSIS_ERROR_H

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>

#define MAX_ERROR_MESSAGE_LENGTH 40 /**< Error messages (excluding the severity level string) should not exceed this length */

// Old
// #define ERROR_PEDALS 			1 /**< OLD ERROR CODE Error with Pedals*/
// #define ERROR_BRAKE_PRESSURE 	2 /**< OLD ERROR CODE Error with Brake Pressure*/
// #define ERROR_INVERTER_RESPONSE  3 /**< OLD ERROR CODE Error with Inverter*/
// #define ERROR_CAN2_RESPONSE		4 /**< OLD ERROR CODE Error with CAN2 Response*/

// Error Codes
#define ERROR_GENERAL                       (0)     /**< Error Code: Generic error for the system */

#define ERROR_CANBUS_1_NO_RESPONSE          (111)   /**< Error Code: CANBUS 1 has no response */
#define ERROR_CANBUS_1_RESPONSE_MALFORMED   (112)   /**< Error Code: CANBUS 1's response was malformed */
#define ERROR_CANBUS_2_NO_RESPONSE          (121)   /**< Error Code: CANBUS 2 has no response */
#define ERROR_CANBUS_2_RESPONSE_MALFORMED   (122)   /**< Error Code: CANBUS 2's reponse was malformed */
#define ERROR_CANBUS_3_NO_RESPONSE          (131)   /**< Error Code: CANBUS 3 has no response */
#define ERROR_CANBUS_3_RESPONSE_MALFORMED   (132)   /**< Error Code: CANBUS 3's reponse was malformed */

#define ERROR_GEARBOX_TEMPERATURE_LOW 	    (2)     /**< Error Code: Gearbox X Temperature Too Low */
#define ERROR_GEARBOX_TEMPERATURE_HIGH 	    (3)     /**< Error Code: Gearbox X Temperature Too High*/
#define ERROR_RADIATOR_TEMPERATURE_LOW	    (4)     /**< Error Code: Radiator X Temperature Too Low */
#define ERROR_RADIATOR_TEMPERATURE_HIGH	    (5)     /**< Error Code: Radiator X Temperature Too High */
#define ERROR_BRAKES_PRESSURE_LOW		    (6)     /**< Error Code: Brake X Pressure Too Low */
#define ERROR_BRAKES_PRESSURE_HIGH		    (7)     /**< Error Code: Brake X Pressure Too High */
#define ERROR_BRAKES_PRESSURE		        (8)     /**< Error Code: Brake X Pressure Too High */
#define ERROR_BRAKES_POSITION_LOW		    (9)     /**< Error Code: Brake X Position Too Low */
#define ERROR_BRAKES_POSITION_HIGH		    (10)    /**< Error Code: Brake X Position Too High */
#define ERROR_BRAKES_POSITION		        (11)    /**< Error Code: Brake X Position Too High */
#define ERROR_STEERING_ANGLE_LEFT		    (12)    /**< Error Code: Steering Wheel Turned Too Far Left */
#define ERROR_STEERING_ANGLE_RIGHT		    (13)    /**< Error Code: Steering Wheel Turned Too Far Right */
#define ERROR_THROTTLE_ERROR			    (14)    /**< Error Code: Problem With Throttle */
#define ERROR_CAR_UPRIGHT_ERROR			    (15)    /**< Error Code: Car No Longer Upgright (Has Rolled) */
#define ERROR_BATTERY_CHARGE			    (16)    /**< Error Code: Problem with Battery Charge */
#define ERROR_BATTERY_TEMPERATURE_LOW	    (17)    /**< Error Code: Battery Temperature Too Low */
#define ERROR_BATTERY_TEMPERATURE_HIGH	    (18)    /**< Error Code: Battery Temperature Too High */

// Error Severity Levels
#define ERROR_LEVEL_INFO    (1)         /**< Low Severity Level - Everything is all gravy baby */
#define ERROR_LEVEL_WARN    (2)         /**< Moderate Severity Level - This is cause for concern */
#define ERROR_LEVEL_ERROR   (3)         /**< Critical Severity Level - Consider shutting the car down */

#define ERROR_STRING_INFO   "INFO"      /**< String Representation of Low Severity Level */
#define ERROR_STRING_WARN   "!WARNING!" /**< String Representation of Moderate Severity Level */
#define ERROR_STRING_ERROR  "##ERROR##" /**< String Representation of Critical Severity Level */

#define SHDN_BRAKE_OVERTRAVEL			0
#define SHDN_DRIVER_ESTOP				1
#define SHDN_INERTIA_SWITCH				2
#define SHDN_LEFT_FRONT_UPRIGHT			3
#define SHDN_RIGHT_FRONT_UPRIGHT		4

/*#define STOP_BRAKE_OVERTRAVEL			(shutdownState >7> 0) & 1UL
#define STOP_DRIVER_ESTOP           (shutdownState >> 1) & 1UL
#define STOP_INERTIA_SWITCH         (shutdownState >> 2) & 1UL
#define STOP_LEFT_FRONT_UPRIGHT     (shutdownState >> 3) & 1UL
#define STOP_RIGHT_FRONT_UPRIGHT    (shutdownState >> 4) & 1UL*/

/**
 * @brief Generates an error message, and takes the necessary actions depending on the severity and error. 
 * 
 * Uses 0 as the part number.
 * 
 * Example Code: 
 * 
 * throw_error_code(WARN, STEERING_ANGLE_LEFT);
 * 
 * @author Shane Havers 
 * 
 * @param severity Severity of the error (INFO, WARN, or ERROR)
 * @param error_code Error to report 
 * 
 * @return void
 */
void throw_error_code(uint16_t severity, uint16_t error_code);

/**
 * @brief Generatres an error message, and takes the necessary actions depending on the severity and error.
 * 
 * Example Code:
 * 
 * throw_error_code_with_number(ERROR, GEARBOX_TEMPERATURE_HIGH, 3);
 * 
 * @author Shane Havers
 * 
 * @param severity Severity of the error (INFO, WARN, or ERROR)
 * @param error_code Error to report
 * @param part_number The number of the part generating the error (i.e. Gearbox 1, Radiator 2 etc.)
 * 
 * @return void
 * 
 */
void throw_error_code_with_number(uint16_t severity, uint16_t error_code, uint16_t part_number);

/**
 * @brief Formats a 50-character long log message, and sends it over UART
 * 
 * This method is called by throw_error_code() and throw_error_code_with_number().
 * There shouldn't be any need to call it anywhere else. 
 * 
 * @author Shane Havers
 * 
 * @param start String representation of log level
 * @param message Message to send - should not exceed 40 characters
 * @param part_number The number of the part generating the error (i.e. Gearbox 1, Radiator 2 etc.)
 * 
 * @return void
 */
void send_error_message(char start[], char message[], uint16_t part_number);

void shutdown_probe();

void shutdown_state(uint16_t shutdownFlag);

#endif // CHASSIS_ERROR_H
