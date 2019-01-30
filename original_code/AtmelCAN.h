/*
 * AtmelCAN.h
 *
 * Created: 12/5/2015 5:14:12 PM
 *  Author: julius
 */ 
#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#ifndef ATMELCAN_H_
#define ATMELCAN_H_
#include <util/delay.h>
#include <avr/io.h>
int8_t CAN_sendTest();
uint8_t CAN_init();
int8_t CAN_findFreeTXMOB();
void CAN_RXInit(int8_t mob, uint8_t numBytes, uint32_t IDmsk, uint32_t ID, uint8_t CanType);
void CAN_TXMOB(int8_t mob, uint8_t numBytes, uint8_t * data, uint32_t ID, uint8_t ms_loop_until_TXOK);

#endif /* ATMELCAN_H_ */