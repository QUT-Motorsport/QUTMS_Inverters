/*
 * MotorControl_2018.c
 *
 * Created: 9/18/2018 12:12:01 AM
 * Author : Ant
 * Modified: 3/07/2019 6:00 PM
 * Author : Jonn Dillon
 */ 
#define F_CPU 16000000UL

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#include <math.h>

#include "AtmelCAN.h"

#define LED_STATUS_ON		PORTB |= 0b00001000		// Status LED ON (Port B3)
#define LED_STATUS_OFF		PORTB &= ~(0b00001000)	// Status LED OFF (Port B3)
#define LED_STATUS_TOGGLE	PORTB ^= 0b00001000		// Status LED Toggle (Port B3)

#define PHASE_U_LOW_ON		PORTD |= 1
#define PHASE_V_LOW_ON		PORTC |= 1
#define PHASE_W_LOW_ON		PORTB |= 1
#define PHASE_U_LOW_OFF		PORTD &= ~1
#define PHASE_V_LOW_OFF		PORTC &= ~1
#define PHASE_W_LOW_OFF		PORTB &= ~1

#define PHASES_ALL_HIGH_OFF POC = 0b00000000
#define PHASE_U_HIGH_ON		POC = 0b00000011
#define PHASE_V_HIGH_ON		POC = 0b00001100
#define PHASE_W_HIGH_ON		POC = 0b00110000

uint8_t getMotorPosition(void);
void kickMotor(void);
uint8_t testChar = 0;

volatile uint16_t rotationCounter = 0;
volatile uint8_t motorCommand = 0;
ISR(CAN_INT_vect)
{
	//CAN_RXInit(5,8,0x0000000, 0x4000000, 1);
	//uint8_t authority;
	int8_t mob;
	if((CANSIT2 & (1 << 5)))	//we received a CAN message on the reverse switch mob
	{
		//LED_STATUS_TOGGLE;
		
		CANPAGE = (5 << 4);			//set the canpage to the receiver MOB
		testChar = CANMSG;
		uint8_t tempChar1 = CANMSG;
		tempChar1 = CANMSG;
		tempChar1 = CANMSG;
		tempChar1 = CANMSG;
		tempChar1 = CANMSG;
		tempChar1 = CANMSG;
		tempChar1 = CANMSG;

		CAN_RXInit(5,8,0x4000000, 0x4000000, 1);
	}
}

int main(void)
{
	DDRB |= 0b11001011;				// make the status LED an output
	DDRC |= 0b00000101;
	DDRD |= 0b10000001;				// PD7 is CAN STB
	
	PORTB &= ~0b11001011;
	PORTC &= ~0b00000001;
	PORTD &= ~0b00000001;
	
	PORTB |= 0b00100100;	//turn hall pullups on
	PORTD |= 0b01000000;
	
	//PLL
	PLLCSR = 0x02;			//start PLL at 32MHz
	
	//INTERRUPTS
	EICRA = 0b00010101;		//turn INT0, INT1 and INT2 either edge sensing on
	EIMSK = 0b00000111;		//enable INTs 2, 1, 0
	
	//PSC
	POCR_RB = 256;
	POCR0SA = POCR1SA = POCR2SA = 220;
	POCR0SB = POCR1SB = POCR2SB = 210;
	PCNF = 0b00011100;						//centre-aligned mode
	PCTL = 0b00100001;						//select PLL clock with no prescale, turn the PSC on
	
	// start the CAN interface
	CAN_init(); // Initialise CAN
	CAN_RXInit(5,8,0x4000000, 0x4000000, 1);
	
	// start the interrupts
	sei();	
	
	//turn the outputs off
	PHASES_ALL_HIGH_OFF;
	PHASE_U_LOW_OFF;
	PHASE_V_LOW_OFF;
	PHASE_W_LOW_OFF;
	
	//kickMotor();
	
	uint8_t motorState = 0;
	
	while(1)
	{
		_delay_ms(500);
		LED_STATUS_TOGGLE;
		/*
		motorCommand = 250 - testChar;
		if(motorCommand < 190) {
			motorCommand = 190;
		}
		if(motorState == 1) {
			POCR0SA = POCR1SA = POCR2SA = motorCommand;
			POCR0SB = POCR1SB = POCR2SB = motorCommand - 10;
		} else {
			POC = 0b00000000;
			PHASE_U_HIGH_ON;
			PHASE_V_LOW_OFF;
			PHASE_W_LOW_OFF;
		}
		
		if(rotationCounter < 100) {
			rotationCounter ++;
		}
		
		if(rotationCounter > 99) {
			motorState = 0;
			PORTB &= ~8;
		} else {
			motorState = 1;
			PORTB |= 8;
		}

		if((motorState == 0) && (motorCommand < 225)) {
			kickMotor();
			motorState = 1;	
			rotationCounter = 0;		
		}
		*/
	}
}

// skateboard ISRs below \/\/\/

ISR(INT0_vect)	//if INT0 is going high + - Z   else if INT0 going low - + Z
{	
	rotationCounter = 0;
	PHASES_ALL_HIGH_OFF;
	PHASE_U_LOW_OFF;
	PHASE_V_LOW_OFF;
	PHASE_W_LOW_OFF;
		
	if ((PIND & 64) == 64) {
		PHASE_U_LOW_ON;
		PHASE_W_HIGH_ON;
		} else {
		PHASE_W_LOW_ON;
		PHASE_U_HIGH_ON;
	}
}

ISR(INT1_vect) //if INT1 is going high - Z +   else if INT1  going low + Z -
{
	rotationCounter = 0;
	PHASES_ALL_HIGH_OFF;
	PHASE_U_LOW_OFF;
	PHASE_V_LOW_OFF;
	PHASE_W_LOW_OFF;
		
	if ((PINB & 4) == 4) {
		PHASE_V_LOW_ON;
		PHASE_U_HIGH_ON;
		} else {
		PHASE_U_LOW_ON;
		PHASE_V_HIGH_ON;
	}
}

ISR(INT2_vect) //if INT2 is going high Z + -   else if INT2 going low  Z - +
{
	rotationCounter = 0;
	PHASES_ALL_HIGH_OFF;
	PHASE_U_LOW_OFF;
	PHASE_V_LOW_OFF;
	PHASE_W_LOW_OFF;
		
	if ((PINB & 32) == 32) {
		PHASE_W_LOW_ON;
		PHASE_V_HIGH_ON;
		} else {
		PHASE_V_LOW_ON;
		PHASE_W_HIGH_ON;
	}
}

void kickMotor(void)
{
	switch (getMotorPosition())
	{
		case 1:
		PHASE_U_LOW_ON;
		PHASE_V_HIGH_ON;
		break;
		case 2:
		PHASE_V_LOW_ON;
		PHASE_W_HIGH_ON;
		case 3:
		PHASE_U_LOW_ON;
		PHASE_W_HIGH_ON;
		break;
		case 4:
		PHASE_W_LOW_ON;
		PHASE_U_HIGH_ON;
		break;
		case 5:
		PHASE_W_LOW_ON;
		PHASE_V_HIGH_ON;
		break;
		case 6:
		PHASE_V_LOW_ON;
		PHASE_U_HIGH_ON;
		break;
		default:
		break;
	}
}
// gets an encoded version of motor position based on hall states
// adds 4 if HALL3 is high, 2 if HALL2 is HIGH and 1 if HALL 1 is high giving positions 1..6 in total
uint8_t getMotorPosition(void)
{
	return (((PINB & 32) / 8) + ((PINB & 4) / 2) + ((PIND & 64) / 64));
}