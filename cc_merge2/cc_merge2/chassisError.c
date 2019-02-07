/**
 * @brief Handles functions, calls and utilities related to the Error system
 * 
 * @file chassisError.c
 * @author Jonn Dillon
 * @date 2018-04-16
 * 
 */

#include "chassisError.h"
#include "uart.h"
#include <string.h>

uint8_t SHDN_FLAGS = 0;

/**
 * throw_error_code()
 * Input:	severity	- Severity of the error (INFO, WARN, or ERROR)
 * 			error_code		- Error to report
 * Returns: none
 * 
 * Generates an error message, and takes the necessary actions depending on the severity and error.
 * Uses 0 as the part number
 **/
void throw_error_code(uint16_t severity, uint16_t error_code)
{	
	throw_error_code_with_number(severity, error_code, 0);
}

/**
 * throw_error_code_with_number()
 * Input:	severity	- Severity of the error (INFO, WARN, or ERROR)
 * 			error_code	- Error to report
 * 			part_number - The number of the part generating the error (i.e. Gearbox 1, Radiator 2 etc.)
 * Returns: none
 * 
 * Generates an error message, and takes the necessary actions depending on the severity and error
 * 
 * */
void throw_error_code_with_number(uint16_t severity, uint16_t error_code, uint16_t part_number) 
{
	switch (error_code) {
		
        // case ERROR_CODE:					
		// 	switch (severity) {
		// 		case ERROR_STRING_INFO:
		// 		// Logging behaviour for Info-level logs
		// 		send_error_message(ERROR_STRING_INFO, "Info Description", part_number);
		// 		// Additional Info Handling
		// 		break;

		// 		case ERROR_LEVEL_WARN:
		// 		// Loggging behaviour for Warning-level logs
		// 		send_error_message(ERROR_STRING_WARN, "Warning Description!", part_number);
		// 		// Additional Warning Handling
		// 		break;

		// 		case ERROR_LEVEL_ERROR:
		// 		// Logging behaviour for Error-level logs
		// 		send_error_message(ERROR_STRING_ERROR, "ERROR DESCRIPTION", part_number);
		// 		// Additional Error Handling
		// 		break;
		//     }			
		// break;

        case ERROR_GENERAL:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Error", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "Error!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "ERROR", part_number);
				// Additional Error Handling
				break;
		    }			
		break;

        // CANBUS based errors
        // --------------------------------------------------------------------

        case ERROR_CANBUS_1_NO_RESPONSE:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "CANBUS 1 No Response", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "CANBUS 1 No Response!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "CANBUS 1 CRITICAL NO RESPONSE", part_number);
				// Additional Error Handling
				break;
		    }			
		break;

        case ERROR_CANBUS_1_RESPONSE_MALFORMED:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "CANBUS 1 Malformed Packet", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "CANBUS 1 Malformed Packet!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "CANBUS 1 CRITICAL MALFORMED PACKET", part_number);
				// Additional Error Handling
				break;
		    }			
		break;

        case ERROR_CANBUS_2_NO_RESPONSE:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "CANBUS 2 No Response", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "CANBUS 2 No Response!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "CANBUS 2 CRITICAL NO RESPONSE", part_number);
				// Additional Error Handling
				break;
		    }			
		break;

        case ERROR_CANBUS_2_RESPONSE_MALFORMED:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "CANBUS 2 Malformed Packet", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "CANBUS 2 Malformed Packet!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "CANBUS 2 CRITICAL MALFORMED PACKET", part_number);
				// Additional Error Handling
				break;
		    }			
		break;

        case ERROR_CANBUS_3_NO_RESPONSE:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "CANBUS 3 No Response", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "CANBUS 3 No Response!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "CANBUS 3 CRITICAL NO RESPONSE", part_number);
				// Additional Error Handling
				break;
		    }			
		break;

        case ERROR_CANBUS_3_RESPONSE_MALFORMED:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "CANBUS 3 Malformed Packet", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "CANBUS 3 Malformed Packet!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "CANBUS 3 CRITICAL MALFORMED PACKET", part_number);
				// Additional Error Handling
				break;
		    }			
		break;

        // Sensor based errors
        // --------------------------------------------------------------------

		case ERROR_GEARBOX_TEMPERATURE_LOW:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Gearbox %d Temperature Normal", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "Gearbox %d Temperature Abnormally Low!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "GEARBOX %d TEMPERATURE CRITICALLY LOW", part_number);
				// Additional Error Handling
				break;
			}			
		break;

		case ERROR_GEARBOX_TEMPERATURE_HIGH:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Gearbox %d Temperature Normal", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "Gearbox %d Temperature Abnormally High!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "GEARBOX %d TEMPERATURE CRITICALLY HIGH", part_number);
				// Additional Error Handling
				break;
			}			
		break;

		case ERROR_RADIATOR_TEMPERATURE_LOW:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Radiator %d Temperature Normal", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "Radiator %d Temperature Abnormally Low!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "RADIATOR %d TEMPERATURE CRITICALLY LOW", part_number);
				// Additional Error Handling
				break;
			}			
		break;

		case ERROR_RADIATOR_TEMPERATURE_HIGH:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Radiator %d Temperature Normal", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "Radiator %d Temperature Abnormally High!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "RADIATOR %d TEMPERATURE CRITICALLY HIGH", part_number);
				// Additional Error Handling
				break;
			}			
		break;

		case ERROR_BRAKES_PRESSURE_LOW:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Brakes %d Pressure Normal", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "Brakes %d Pressure Abnormally Low!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "BRAKES %d PRESSURE CRITICALLY LOW", part_number);
				// Additional Error Handling
				break;
			}			
		break;

		case ERROR_BRAKES_PRESSURE_HIGH:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Brakes %d Pressure Normal", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "Brakes %d Pressure Abnormally High!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "BRAKES %d PRESSURE CRITICALLY HIGH", part_number);
				// Additional Error Handling
				break;
			}			
		break;

        case ERROR_BRAKES_PRESSURE:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Brakes %d Pressure Normal", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "Brakes %d Pressure Warnning!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "BRAKES %d PRESSURE CRITICAL!", part_number);
				// Additional Error Handling
				break;
			}			
		break;

		case ERROR_BRAKES_POSITION_LOW:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Brakes %d Position Normal", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "Brakes %d Position Abnormally Low!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "BRAKES %d POSITION CRITICALLY LOW", part_number);
				// Additional Error Handling
				break;
			}			
		break;

		case ERROR_BRAKES_POSITION_HIGH:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Brakes %d Pressure Normal", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "Brakes %d Pressure Abnormally High!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "BRAKES %d PRESSURE CRITICALLY HIGH", part_number);
				// Additional Error Handling
				break;
			}			
		break;

        case ERROR_BRAKES_POSITION:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Brakes %d Position Normal", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "Brakes %d Position Warning!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "BRAKES %d POSITION CRITICAL", part_number);
				// Additional Error Handling
				break;
			}			
		break;

		case ERROR_STEERING_ANGLE_LEFT:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Steering Angle Normal", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "STEERING ANGLE OUTSIDE LEFT PARAMETER", part_number);
				// Additional Error Handling
				break;
			}			
		break;

		case ERROR_STEERING_ANGLE_RIGHT:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Steering Angle Normal", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "STEERING ANGLE OUTSIDE RIGHT PARAMETER", part_number);
				// Additional Error Handling
				break;
			}			
		break;

		case ERROR_THROTTLE_ERROR:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Throttle Function Normal", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "THROTTLE CRITICAL MALFUNCTION", part_number);
				// Additional Error Handling
				break;
			}			
		break;

		case ERROR_CAR_UPRIGHT_ERROR:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Car is upright", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "CAR HAS OVERTURNED", part_number);
				// Additional Error Handling
				break;
			}			
		break;

		case ERROR_BATTERY_CHARGE:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Battery %d Charge Normal", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "Battery %d Charge Low", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "BATTERY %d CHARGE CRITICAL", part_number);
				// Additional Error Handling
				break;
			}			
		break;

		case ERROR_BATTERY_TEMPERATURE_LOW:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Battery %d Temperature Normal", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "Battery %d Temperature Abnormally Low!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "BATTERY %d TEMPERATURE CRITICALLY LOW", part_number);
				// Additional Error Handling
				break;
			}			
		break;

		case ERROR_BATTERY_TEMPERATURE_HIGH:					
			switch (severity) {
				case ERROR_LEVEL_INFO:
				// Logging behaviour for Info-level logs
				send_error_message(ERROR_STRING_INFO, "Battery %d Temperature Normal", part_number);
				// Additional Info Handling
				break;

				case ERROR_LEVEL_WARN:
				// Loggging behaviour for Warning-level logs
				send_error_message(ERROR_STRING_WARN, "Battery %d Temperature Abnormally High!", part_number);
				// Additional Warning Handling
				break;

				case ERROR_LEVEL_ERROR:
				// Logging behaviour for Error-level logs
				send_error_message(ERROR_STRING_ERROR, "BATTERY %d TEMPERATURE CRITICALLY HIGH", part_number);
				// Additional Error Handling
				break;
			}			
		break;

		// Template for new error codes, copy and uncomment to use
		// If the description contains a single %d, it can be used to indicate a part number
		// case ERROR_CODE:					
		// 	switch (severity) {
		// 		case ERROR_STRING_INFO:
		// 		// Logging behaviour for Info-level logs
		// 		send_error_message(ERROR_STRING_INFO, "Info Description", part_number);
		//		// Additional Info Handling
		// 		break;

		// 		case ERROR_LEVEL_WARN:
		// 		// Loggging behaviour for Warning-level logs
		// 		send_error_message(ERROR_STRING_WARN, "Warning Description!", part_number);
		//		// Additional Warning Handling
		// 		break;

		// 		case ERROR_LEVEL_ERROR:
		// 		// Logging behaviour for Error-level logs
		// 		send_error_message(ERROR_STRING_ERROR, "ERROR DESCRIPTION", part_number);
		//		// Additional Error Handling
		// 		break;
		// 	}			
		// break;

	}
}

/**
 * send_error_message()
 * Input: 	start		- String representation of log level
 * 			message		- Message to send - should not exceed 40 characters
 * 			part_number - The number of the part generating the error (i.e. Gearbox 1, Radiator 2 etc.)
 * Returns: none
 * 
 * Formats a 50-character long log message, and sends it over UART.
 * */
void send_error_message(char start[], char message[], uint16_t part_number) 
{
	unsigned char *messageWithNumber;
	unsigned char *errorData;
	
	// Format error message with part number if given
	snprintf(messageWithNumber, MAX_ERROR_MESSAGE_LENGTH, message, part_number);

	// Create error data
	snprintf(errorData, MAX_ERROR_MESSAGE_LENGTH + 10, "%s %s", start, messageWithNumber);
	
	// Send error data over UART
	UART_sendPacket(errorData, MAX_ERROR_MESSAGE_LENGTH + 10);
}

/**
 * shutdown_probe()
 * Input:	none
 * Returns: none
 * 
 * Checks if a shutdown switch has been activated. Sends the method of shutdown to be recorded
 * 
 * Reference: ATmega Datasheet Chapter 13 (I/O-Ports)
 **/
void shutdown_probe()
{
	
	// if(STOP_BRAKE_OVERTRAVEL)shutdown_state(SHDN_BRAKE_OVERTRAVEL);
	// if(STOP_DRIVER_ESTOP)shutdown_state(SHDN_DRIVER_ESTOP);
 	// if(STOP_INERTIA_SWITCH)shutdown_state(SHDN_INERTIA_SWITCH);
 	// if(STOP_LEFT_FRONT_UPRIGHT)shutdown_state(SHDN_LEFT_FRONT_UPRIGHT);
 	// if(STOP_RIGHT_FRONT_UPRIGHT)shutdown_state(SHDN_RIGHT_FRONT_UPRIGHT);
}

/**
 * shutdown_state()
 * Input:	None
 * Returns: None
 * 
 * Reads shutdownState defined in main.h. If bit is set, send message over UART.
 * 
 **/
void shutdown_state(uint16_t shutdownFlag)
{

}