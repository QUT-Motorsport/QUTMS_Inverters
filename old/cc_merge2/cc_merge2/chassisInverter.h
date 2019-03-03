/**
 * @file chassisInverter.h
 * @author Jonn Dillon
 * @date 16/4/2018
 * @brief Defines and outlines all varibles and definitions related to the Inverter/Motor Controller board
 **/

#ifndef CHASSIS_INVERTER_H
#define CHASSIS_INVERTER_H

#include <stdlib.h>
#include <avr/io.h>

#define NUM_INVERTERS 4
/**
 * @brief Data Structure for Inverter Data
 * 
 */
typedef struct Inverter
{
	uint8_t ID; /**< Inverter Identifier */
	uint16_t RPM; /**< RPM of inverter */
	uint16_t temperature; /**< Inverter temperature */
	uint16_t current; /**< Inverter current draw */
	uint16_t duty; /**< Inverter duty cycle */
}Inverter;

/**
 * @brief Central storage for inverter data
 * 
 */
struct Inverter inverters[NUM_INVERTERS];

/**
 * @brief Inverter Status flag
 * 
 */
unsigned char inverterStatus = 0;

/**
 * @brief Appears unused?
 * 
 */
unsigned int currentRPM = 0;

/**
 * @brief Appears unused?
 * 
 */
unsigned int inverterTemp = 0;

/**
 * @brief Appears unused?
 * 
 */
unsigned int inverterDCcurrent = 0;

/**
 * @brief Process the data that was received from the inverters and saves it to the proper variables
 * 
 * @param data The data received from the inverters in the CAN bus
 * @param ID Message Identifier
 * @return uint8_t 0 if there was something wrong with the CAN packet resulting in no inverter being found from the ID given. 1 if execution is normal
 */
uint8_t inverters_save_data(uint8_t* data, uint32_t* ID);

#endif // CHASSIS_INVERTER_H