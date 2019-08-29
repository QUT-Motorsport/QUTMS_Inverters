/*
 * Created: 9/18/2018 12:12:01 AM
 * Inverter Code V2
 * Author : Ant
 * Modified: Aziz, Jonn
 */ 
#define F_CPU 16000000

#include <avr/pgmspace.h>
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#include <math.h>

#include "AtmelCAN.h"

#define TESTMOTOR 0 // COMMENT OUT THIS LINE TO MAKE THE PLETENBUG MOTORS RUN
#define CLOCKWISE 0 // COMMENT OUT THIS LINE TO MAKE THEM RUN ANTI-CLOCKWISE

#define PHASE_PULSE_DELAY 5000

#define PHASE_U_LOW_ON PORTD |= 1
#define PHASE_V_LOW_ON PORTC |= 1
#define PHASE_W_LOW_ON PORTB |= 1
#define PHASE_U_LOW_OFF PORTD &= ~1
#define PHASE_V_LOW_OFF PORTC &= ~1
#define PHASE_W_LOW_OFF PORTB &= ~1

#define PHASES_ALL_HIGH_OFF POC = 0b00000000
#define PHASE_U_HIGH_ON POC = 0b00000011
#define PHASE_V_HIGH_ON POC = 0b00001100
#define PHASE_W_HIGH_ON POC = 0b00110000

uint8_t getMotorPosition(void);
void kickMotor(void);
uint8_t testChar = 0;

volatile uint16_t rotationCounter = 0;
volatile uint8_t motorCommand = 0;
uint8_t motorState = 0;

uint8_t canArray[4] = {0,0,0,0};
volatile uint8_t canFlag = 0;

/*
 * Test Motor Commutation Order
 *
 *		Counter-Clockwise
 *		Motor					Sensor (CCW)
 *		Y(U)	G(V)	B(W)	Y	G	B
 * -----------------------------------------
 * 1	+		-		X		+	+	-
 * 2	+		X		-		-	+	-
 * 3	X		+		-		-	+	+
 * 4	-		+		X		-	-	+
 * 5	-		X		+		+	-	+
 * 6	X		-		+		+	-	-
 * 
 * 
 *		Clockwise
 *		Motor					Sensor (CW)
 *		Y(U)	G(V)	B(W)	Y	G	B
 * -----------------------------------------
 * 1	X		-		+		+	-	+
 * 2	-		X		+		-	-	+
 * 3	-		+		X		-	+	+
 * 4	X		+		-		-	+	-
 * 5	+		X		-		+	+	-
 * 6	+		-		X		+	-	-
 */



/*
ISR(CAN_INT_vect)
{
	//uint8_t authority;
	int8_t mob;
	if((CANSIT2 & (1 << 5)))	//we received a CAN message on the reverse switch mob
	{
		
		CANPAGE = (5 << 4);			//set the canpage to the receiver MOB
		testChar = CANMSG;
		uint8_t tempChar1 = CANMSG;
		tempChar1 = CANMSG;
		tempChar1 = CANMSG;
		tempChar1 = CANMSG;
		tempChar1 = CANMSG;
		tempChar1 = CANMSG;
		tempChar1 = CANMSG;

		canFlag = 1;

		CAN_RXInit(5,8,0x4000000, 0x4000000, 1);
	}
}*/

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
	// CAN_init();		// Initialise CAN
	// CAN_RXInit(5,8,0x4000000, 0x4000000, 1);
	
	// start the interrupts
	// sei();	
	
	//turn the outputs off
	PHASES_ALL_HIGH_OFF;
	PHASE_U_LOW_OFF;
	PHASE_V_LOW_OFF;
	PHASE_W_LOW_OFF;
	
	while(1)
	{

		kickMotorSelect(1); // First Actual Order
		_delay_ms(PHASE_PULSE_DELAY);
		kickMotorSelect(5); // Second Actual Order
		_delay_ms(PHASE_PULSE_DELAY);
		kickMotorSelect(4); // Thrid Actual Order
		_delay_ms(PHASE_PULSE_DELAY);
		kickMotorSelect(6); // Fourth Actual Order
		_delay_ms(PHASE_PULSE_DELAY);
		kickMotorSelect(2); // Fifth Actual Order
		_delay_ms(PHASE_PULSE_DELAY);
		kickMotorSelect(3); // Sixth Actual Order
		_delay_ms(PHASE_PULSE_DELAY);
		/*
		motorCommand = 250 - testChar;
		if(motorCommand < 190) motorCommand = 190;
		if(motorState == 1)
		{
			POCR0SA = POCR1SA = POCR2SA = motorCommand;
			POCR0SB = POCR1SB = POCR2SB = motorCommand - 10;
		}
		
		else
		{
			POC = 0b00000000;
			PHASE_U_HIGH_ON;
			PHASE_V_LOW_OFF;
			PHASE_W_LOW_OFF;
		}
		if(rotationCounter < 1000) rotationCounter ++;
		if(rotationCounter > 999)
		{
			motorState = 0;
			PORTB &= ~8;
		}
		else 
		{
			motorState = 1;
			PORTB |= 8;
		}
		// motorState = 1; // Part of debugging ignore
		
		if((motorState == 0) && (motorCommand < 225))
		{
			kickMotor();
			motorState = 1;	
			rotationCounter = 0;		
		}

		if(canFlag == 1) {
			canFlag = 0;
			canArray[0] = motorState;
			canArray[1] = motorCommand;
			canArray[2] = rotationCounter >> 8;
			canArray[3] = rotationCounter;
			CAN_TXMOB(1, 4, canArray, 0x5555555, 0x0100);
		}*/
	}
}

/*
ISR(INT0_vect)	//if INT0 is going high + - Z   else if INT0 going low - + Z
{	
	PORTB ^= 8;
	rotationCounter = 0;
	PHASES_ALL_HIGH_OFF;
	PHASE_U_LOW_OFF;
	PHASE_V_LOW_OFF;
	PHASE_W_LOW_OFF;
		
	if ((PIND & 64) == 64) {
		#ifdef TESTMOTOR // The Skateboard Motor	
			#ifdef CLOCKWISE // Running Clockwise				> 6
				PHASE_U_HIGH_ON;
				PHASE_V_LOW_ON;
			#endif
			#ifndef CLOCKWISE // Running Anti-Clockwise			> 1
				// PHASE_V_LOW_ON;
				// PHASE_U_HIGH_ON;
			#endif
		#endif
		#ifndef TESTMOTOR // The Pletenburg Motor
			#ifdef CLOCKWISE // Running Clockwise
				PHASE_U_LOW_ON;
				PHASE_W_HIGH_ON;
			#endif
			#ifndef CLOCKWISE // Running Anti-Clockwise
			
			#endif
		#endif
		
	} else {
		#ifdef TESTMOTOR // THe Skateboard Motor
			#ifdef CLOCKWISE // Running Clockwise				> 4
				PHASE_U_LOW_ON;
				PHASE_V_HIGH_ON;
			#endif
			#ifndef CLOCKWISE // Running Anti-Clockwise			> 6
				// PHASE_V_LOW_ON;
				// PHASE_W_HIGH_ON;
			#endif
		#endif
		#ifndef TESTMOTOR // The Pletenburg Motor
			#ifdef CLOCKWISE // Running Clockwise
				PHASE_W_LOW_ON;
				PHASE_U_HIGH_ON;
			#endif
			#ifndef CLOCKWISE // Running Anti-Clockwise
			
			#endif
		#endif
		
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
		
		#ifdef TESTMOTOR // THe Skateboard Motor
			#ifdef CLOCKWISE // Running Clockwise				> 3
				PHASE_V_HIGH_ON;
				PHASE_W_LOW_ON;
			#endif
			#ifndef CLOCKWISE // Running Anti-Clockwise			> 4
				// PHASE_U_LOW_ON;
				// PHASE_V_HIGH_ON;
			#endif
		#endif
		#ifndef TESTMOTOR // The Pletenburg Motor
			#ifdef CLOCKWISE // Running Clockwise
				PHASE_V_LOW_ON;
				PHASE_U_HIGH_ON;
			#endif
			#ifndef CLOCKWISE // Running Anti-Clockwise
			
			#endif
		#endif
	} else {
		
		#ifdef TESTMOTOR // THe Skateboard Motor
			#ifdef CLOCKWISE // Running Clockwise				> 1
				PHASE_V_LOW_ON;
				PHASE_W_HIGH_ON;
			#endif
			#ifndef CLOCKWISE // Running Anti-Clockwise			> 3
				// PHASE_W_LOW_ON;
				// PHASE_V_HIGH_ON;
			#endif
		#endif
		#ifndef TESTMOTOR // The Pletenburg Motor
			#ifdef CLOCKWISE // Running Clockwise
				PHASE_U_LOW_ON;
				PHASE_V_HIGH_ON;
			#endif
			#ifndef CLOCKWISE // Running Anti-Clockwise
			
			#endif
		#endif
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
		
		#ifdef TESTMOTOR // THe Skateboard Motor
			#ifdef CLOCKWISE // Running Clockwise				> 2
				PHASE_W_HIGH_ON;
				PHASE_U_LOW_ON;
			#endif
			#ifndef CLOCKWISE // Running Anti-Clockwise			> 5
				// PHASE_U_LOW_ON;
				// PHASE_W_HIGH_ON;
			#endif
		#endif
		#ifndef TESTMOTOR // The Pletenburg Motor
			#ifdef CLOCKWISE // Running Clockwise
				PHASE_W_LOW_ON;
				PHASE_V_HIGH_ON;
			#endif
			#ifndef CLOCKWISE // Running Anti-Clockwise
			
			#endif
		#endif
	} else {
		#ifdef TESTMOTOR // THe Skateboard Motor
			#ifdef CLOCKWISE // Running Clockwise				> 5
				PHASE_U_HIGH_ON;
				PHASE_W_LOW_ON;
			#endif
			#ifndef CLOCKWISE // Running Anti-Clockwise			> 2
				// PHASE_W_LOW_ON;
				// PHASE_U_HIGH_ON;
			#endif
		#endif
		#ifndef TESTMOTOR // The Pletenburg Motor
			#ifdef CLOCKWISE // Running Clockwise
				PHASE_V_LOW_ON;
				PHASE_W_HIGH_ON;
			#endif
			#ifndef CLOCKWISE // Running Anti-Clockwise
			
			#endif
		#endif
	}
}

void kickMotor(void)
{
	switch (getMotorPosition())
	{
		case 1:
			#ifdef TESTMOTOR // THe Skateboard Motor
				#ifdef CLOCKWISE // Running Clockwise			> 1
					PHASE_V_LOW_ON;
					PHASE_W_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise		> 3
					// PHASE_W_LOW_ON;
					// PHASE_V_HIGH_ON;
				#endif
			#endif
			#ifndef TESTMOTOR // The Pletenburg Motor
				#ifdef CLOCKWISE // Running Clockwise
					PHASE_U_LOW_ON;
					PHASE_V_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise
				
				#endif
			#endif
			break;
		case 2:
			#ifdef TESTMOTOR // THe Skateboard Motor
				#ifdef CLOCKWISE // Running Clockwise			> 5
					PHASE_U_HIGH_ON;
					PHASE_W_LOW_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise		> 2
					// PHASE_W_LOW_ON;
					// PHASE_U_HIGH_ON;
				#endif
			#endif
			#ifndef TESTMOTOR // The Pletenburg Motor
				#ifdef CLOCKWISE // Running Clockwise
					PHASE_V_LOW_ON;
					PHASE_W_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise
		
				#endif
			#endif
			break;
		case 3:
			#ifdef TESTMOTOR // THe Skateboard Motor
				#ifdef CLOCKWISE // Running Clockwise			> 6
					PHASE_U_HIGH_ON;
					PHASE_V_LOW_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise		> 1
					// PHASE_V_LOW_ON;
					// PHASE_U_HIGH_ON;
				#endif
			#endif
			#ifndef TESTMOTOR // The Pletenburg Motor
				#ifdef CLOCKWISE // Running Clockwise
					PHASE_U_LOW_ON;
					PHASE_W_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise
			
				#endif
			#endif
			break;
		case 4:
			#ifdef TESTMOTOR // THe Skateboard Motor
				#ifdef CLOCKWISE // Running Clockwise			> 3
					PHASE_U_LOW_ON;
					PHASE_V_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise		> 6
					// PHASE_V_LOW_ON;
					// PHASE_W_HIGH_ON;
				#endif
			#endif
			#ifndef TESTMOTOR // The Pletenburg Motor
				#ifdef CLOCKWISE // Running Clockwise
					PHASE_W_LOW_ON;
					PHASE_U_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise
			
				#endif
			#endif
			break;
		case 5:
			#ifdef TESTMOTOR // THe Skateboard Motor
				#ifdef CLOCKWISE // Running Clockwise			> 2
					PHASE_U_LOW_ON;
					PHASE_W_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise		> 5
					// PHASE_U_LOW_ON;
					// PHASE_W_HIGH_ON;
				#endif
			#endif
			#ifndef TESTMOTOR // The Pletenburg Motor
				#ifdef CLOCKWISE // Running Clockwise
					PHASE_W_LOW_ON;
					PHASE_V_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise
			
				#endif
			#endif
			break;
		case 6:
			#ifdef TESTMOTOR // THe Skateboard Motor
				#ifdef CLOCKWISE // Running Clockwise			> 4
					PHASE_W_LOW_ON;
					PHASE_V_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise		> 4
					// PHASE_U_LOW_ON;
					// PHASE_V_HIGH_ON;
				#endif
			#endif
			#ifndef TESTMOTOR // The Pletenburg Motor
				#ifdef CLOCKWISE // Running Clockwise
					PHASE_V_LOW_ON;
					PHASE_U_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise
			
				#endif
			#endif
			break;
		default:
		break;
	}
}*/


void kickMotorSelect(uint8_t caseSelect)
{

	PHASES_ALL_HIGH_OFF;
	PHASE_U_LOW_OFF;
	PHASE_V_LOW_OFF;
	PHASE_W_LOW_OFF;

	switch (caseSelect)
	{
		case 1:
			#ifdef TESTMOTOR // THe Skateboard Motor
				#ifdef CLOCKWISE // Running Clockwise			> 1
					PHASE_V_LOW_ON;
					PHASE_W_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise		> 3
					// PHASE_W_LOW_ON;
					// PHASE_V_HIGH_ON;
				#endif
			#endif
			#ifndef TESTMOTOR // The Pletenburg Motor
				#ifdef CLOCKWISE // Running Clockwise
					PHASE_U_LOW_ON;
					PHASE_V_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise
		
				#endif
			#endif
			break;
		case 2:
			#ifdef TESTMOTOR // THe Skateboard Motor
				#ifdef CLOCKWISE // Running Clockwise			> 5
					PHASE_U_HIGH_ON;
					PHASE_W_LOW_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise		> 2
					// PHASE_W_LOW_ON;
					// PHASE_U_HIGH_ON;
				#endif
			#endif
			#ifndef TESTMOTOR // The Pletenburg Motor
				#ifdef CLOCKWISE // Running Clockwise
					PHASE_V_LOW_ON;
					PHASE_W_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise
		
				#endif
			#endif
		break;
		case 3:
			#ifdef TESTMOTOR // THe Skateboard Motor
				#ifdef CLOCKWISE // Running Clockwise			> 6
					PHASE_U_HIGH_ON;
					PHASE_V_LOW_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise		> 1
					// PHASE_V_LOW_ON;
					// PHASE_U_HIGH_ON;
				#endif
			#endif
			#ifndef TESTMOTOR // The Pletenburg Motor
				#ifdef CLOCKWISE // Running Clockwise
					PHASE_U_LOW_ON;
					PHASE_W_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise
		
				#endif
			#endif
			break;
		case 4:
			#ifdef TESTMOTOR // THe Skateboard Motor
				#ifdef CLOCKWISE // Running Clockwise			> 3
					PHASE_U_LOW_ON;
					PHASE_V_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise		> 6
					// PHASE_V_LOW_ON;
					// PHASE_W_HIGH_ON;
				#endif
			#endif
			#ifndef TESTMOTOR // The Pletenburg Motor
				#ifdef CLOCKWISE // Running Clockwise
					PHASE_W_LOW_ON;
					PHASE_U_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise
		
				#endif
			#endif
			break;
		case 5:
			#ifdef TESTMOTOR // THe Skateboard Motor
				#ifdef CLOCKWISE // Running Clockwise			> 2
					PHASE_U_LOW_ON;
					PHASE_W_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise		> 5
					// PHASE_U_LOW_ON;
					// PHASE_W_HIGH_ON;
				#endif
			#endif
			#ifndef TESTMOTOR // The Pletenburg Motor
				#ifdef CLOCKWISE // Running Clockwise
					PHASE_W_LOW_ON;
					PHASE_V_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise
		
				#endif
			#endif
			break;
		case 6:
			#ifdef TESTMOTOR // THe Skateboard Motor
				#ifdef CLOCKWISE // Running Clockwise			> 4
					PHASE_W_LOW_ON;
					PHASE_V_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise		> 4
					// PHASE_U_LOW_ON;
					// PHASE_V_HIGH_ON;
				#endif
			#endif
			#ifndef TESTMOTOR // The Pletenburg Motor
				#ifdef CLOCKWISE // Running Clockwise
					PHASE_V_LOW_ON;
					PHASE_U_HIGH_ON;
				#endif
				#ifndef CLOCKWISE // Running Anti-Clockwise
		
				#endif
			#endif
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