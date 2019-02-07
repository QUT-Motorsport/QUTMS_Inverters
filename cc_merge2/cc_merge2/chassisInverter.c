/**
 * @file chassisInverter.c
 * @author Jonn Dillon
 * @date 16/4/2018
 * @brief Handles functions, calls and utilities related to the Inverters/Motor Controllers.
 **/

#include "chassisInverter.h";
#define NUM_INVERTERS 4
/**
 * inverters_save_data()
 * Input:	data	-	The data received from the inverters in the CAN bus
 * Returns: 0 if there was something wrong with the CAN packet resulting in no inverter being found from the ID given. 1 if execution is normal
 * 
 * Process the data that was received from the inverters and saves it to the proper variables
 **/
uint8_t inverters_save_data(uint8_t* data, uint32_t* ID)
{
	//loop through all inverters
	for(uint8_t i = 0; i < NUM_INVERTERS; i++)
	{
		//if the ID contains 0b0001, 0b0010, 0b0100 or 0b1000, then assign the data from the can packet to this inverter.
		if(((*ID>>13)&0b1111) & 1<<i)
		{
			inverters[i].RPM = data[1];
			inverters[i].RPM |= data[0]<<8;
			inverters[i].temperature = data[3];
			inverters[i].temperature |= data[2]<<8;
			inverters[i].current = data[4];
			inverters[i].current |= data[5]<<8;
			inverters[i].duty = data[7];
			inverters[i].duty |= data[6];
			return 1;
		}
	}
	return 0;
}