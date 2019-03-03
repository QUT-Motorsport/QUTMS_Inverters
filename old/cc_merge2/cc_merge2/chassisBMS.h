/**
 * @file chassisBMS.h
 * @author Jonn Dillon
 * @date 16/4/2018
 * @brief Defines and outlines all varibles and definitions related to the Battery Mangement System
 **/
#ifndef CHASSIS_BMS_H
#define CHASSIS_BMS_H

#include <stdlib.h>
#include <avr/io.h>

// Sets up CMU structs
/**
 * @brief 
 * 
 */
typedef struct CMU
{
	uint8_t CMU_num, /**< */
			CMU_ID; /**< */
	uint16_t voltages[CMU_CELL_COUNT];	/*< will change with change of parameter. Unlikely to. */
	uint16_t temperatures[CMU_CELL_COUNT]; /*< */
}CMU;

/**
 * @brief
 * 
 */
typedef struct Accumulator
{
	uint8_t flags[4];
	uint8_t ID;
	uint16_t MinV;
	uint16_t MaxV;
	uint16_t AvgV;
	uint16_t AvgT;
	uint16_t MinT;
	uint16_t MaxT;
	CMU cmus[NUM_CMU_PER_AMU];
} Accumulator;
struct Accumulator	accumulators[NUM_ACCUMULATORS]; /**< */

// Defines global var for 
unsigned int avgVolts = 0; /**< */
unsigned int avgTemp = 0; /**< */
unsigned int minVolts = 0; /**< */
unsigned int minTemp = 0; /**< */
unsigned int maxVolts = 0; /**< */
unsigned int maxTemp = 0; /**< */

unsigned char cellBalancing = 0; /**< */

#endif // CHASSIS_BMS_H