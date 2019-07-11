/*
 * MotorControl_2018.c
 *
 * Created: 9/18/2018 12:12:01 AM
 * Author : Ant
 */ 

#define F_CPU 16000000

/* Built-in libraries */
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <string.h>
#include <math.h>

/* Custom libraries */
#include "AtmelCAN.h"

#define LED0_MASK (1 << 0)
#define LED1_MASK (1 << 1)
 
/* Phases control */
#define PHASE_U_LOW_ON PORTD |= 1 // 0b??????1?
#define PHASE_V_LOW_ON PORTC |= 1 // 0b??????1?
#define PHASE_W_LOW_ON PORTB |= 1 // 0b??????1?
#define PHASE_U_LOW_OFF PORTD &= ~1 // 0b??????0?
#define PHASE_V_LOW_OFF PORTC &= ~1 // 0b??????0?
#define PHASE_W_LOW_OFF PORTB &= ~1 // 0b??????0?

#define PHASES_ALL_HIGH_OFF POC = 0b00000000
#define PHASE_U_HIGH_ON POC = 0b00000011
#define PHASE_V_HIGH_ON POC = 0b00001100
#define PHASE_W_HIGH_ON POC = 0b00110000

/* Define functions */
uint8_t getMotorPosition(void);
void kickMotor(void);
void toggle_led(void);
void setupInverter(void);

uint8_t WHICH_DIRECTION = 0;
uint8_t CLOCK_WISE = 0;
uint8_t ANTI_CLOCK_WISE = 1;

uint8_t testChar = 0;
uint8_t tempStorage = 0;
uint8_t cleanMotorCommand = 0;
uint8_t startPhase = -1;
uint8_t revolutions = 0;

uint8_t CAN_SEND_BACK_COUNTER = 0;
uint8_t CAN_SEND_BACK_MAX = 5;

volatile uint16_t rotationCounter = 0;
volatile uint8_t motorCommand = 0;
uint8_t tData2[8] = {65,66,67,68,69,70,71,72};
int8_t response_CAN = 0;
//uint8_t tData [1] = {111};
	
#define LED_STATUS_ON		PORTB |= 0b00001000		// Status LED ON (Port B3)
#define LED_STATUS_OFF		PORTB &= ~(0b00001000)	// Status LED OFF (Port B3)
#define LED_STATUS_TOGGLE	PORTB ^= 0b00001000		// Status LED Toggle (Port B3)

ISR(CAN_INT_vect)
{
	int8_t savecanpage;

	savecanpage = CANPAGE;
	
	if(CANSTMOB & ( 1 << RXOK)){
		if((CANSIT2 & (1 << 5)))	//we received a CAN message on the reverse switch mob
		{
		
			CANPAGE = (5 << 4);			//set the canpage to the receiver MOB
			testChar = CANMSG;
			response_CAN = 1;
			// LED_STATUS_TOGGLE;

			//CAN_RXInit(5,8,0x4000000, 0x4000000, 1);
		}
	}

	CANSTMOB = 0x00;
	CANPAGE = savecanpage;
	
}

/*
ISR(TIMER1_COMPA_vect)
{
	
	//oneKHzTimer();
	//uart_puts("HelloWorld!");
	//char msg[12];
	//sprintf(msg, "r: %d", out);
	//uart_puts(msg);
}
*/

int main(void)
{
	// Choose which direction the motor will spin on
	// CLOCK_WISE
	// ANTI_CLOCK_WISE
	WHICH_DIRECTION = CLOCK_WISE;
	
	setupInverter();
	
	uint8_t motorState = 0;
	uint8_t motorMaxAccelration = 6;
	
	while(1)
	{
		if(response_CAN == 1){
			CAN_TXMOB(1, 8, tData2, 0x5555555, 0x0100);
			CAN_RXInit(5,8,0x4000000, 0x4000000, 1);
			response_CAN = 0;
			//toggle_led();
		}

		cleanMotorCommand = testChar;
		/*
		This will ensure the motors will
			not jump more than 10% in torque power
		
		range: 250 to 190. 10% of the range is 6. thus, the 6.
			Change as and if needed
		*/
		if (cleanMotorCommand - tempStorage > motorMaxAccelration){
			cleanMotorCommand = tempStorage + motorMaxAccelration;
		}
		
		// Make sure the range is correct
		if (cleanMotorCommand < 0){
			cleanMotorCommand = 0;
		}

		tempStorage = cleanMotorCommand; // 250
		
		motorCommand = 250 - cleanMotorCommand;
		
		// Check motor command range
		if(motorCommand < 190){
			motorCommand = 190;
		}
		if(motorCommand > 250){
			motorCommand = 250;
		}
		
		if(motorState == 1)
		{
			// PWM code.
			//POCR0SA = POCR1SA = POCR2SA = motorCommand;
			//POCR0SB = POCR1SB = POCR2SB = motorCommand - 10;
		}
		else
		{
			POC = 0b00000000;
			PHASE_U_HIGH_ON;
			PHASE_V_LOW_OFF;
			PHASE_W_LOW_OFF;
		}
		
		// Add one to the while-loop-counter signifying no motor movements
		if(rotationCounter < 100)
		{
			rotationCounter ++;
		}
		
		// Check if this while loop went 100 times with no motor movements
		if(rotationCounter > 99)
		{
			motorState = 0;
			//toggle_led();
			//CAN_TXMOB(mob, 1, &POCR0SA, 0, 2); //transmit registration and do not wait for finish
			//PORTB &= ~8; // turn all port B off
			PORTB &= ~8;
		}
		else 
		{
			motorState = 1;
			//toggle_led();
			//CAN_TXMOB(mob, 1, &POCR0SA, 0, 2); //transmit registration and do not wait for finish
			//PORTB |= 8; // turn all port B on
			PORTB |= 8;
		}

		
		if((motorState == 0) && (motorCommand < 225))
		{
			//kickMotor();
			//motorState = 1;
			rotationCounter = 0;
		}
		
	}
}

void setupInverter(void)
{
	DDRB |= 0b11001011;				// make the status LED an output
	DDRC |= 0b00000101;
	DDRD |= 0b10000001;				// PD7 is CAN STB
	
	PORTB &= ~0b11001011;
	PORTC &= ~0b00000001;
	PORTD &= ~0b00000001;
	
	PORTB |= 0b00100100;	//turn hall pullups on
	PORTD |= 0b01000000;
	
	
	PLLCSR = 0x02;			//start PLL at INTERRUPTS
	
	//32MHz
	EICRA = 0b00010101;		//turn INT0, INT1 and INT2 either edge sensing on
	EIMSK = 0b00000111;		//enable INTs 2, 1, 0 TIMER0_OVF_vect

	// start the CAN interface
	CAN_init();		// Initialise CAN
	CAN_RXInit(5,8,0x4000000, 0x4000000, 1);  // Receive a message

	// start the interrupts
	sei();

	//PSC
	// PWM setup code.
	
	POCR_RB = 256;
	POCR0SA = POCR1SA = POCR2SA = 220;
	POCR0SB = POCR1SB = POCR2SB = 210;
	PCNF = 0b00011100;						//centre-aligned mode
	PCTL = 0b00100001;						//select PLL clock with no prescale, turn the PSC on

	//turn the outputs off
	PHASES_ALL_HIGH_OFF;
	PHASE_U_LOW_OFF;
	PHASE_V_LOW_OFF;
	PHASE_W_LOW_OFF;

	/*
	//set timer1 interrupt at 1Hz
	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCNT1  = 0;//initialize counter value to 0
	// set compare match register for 1hz increments
	OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)
	// turn on CTC mode
	TCCR1B |= (1 << WGM12);
	// Set CS12 and CS10 bits for 1024 prescaler
	TCCR1B |= (1 << CS12) | (1 << CS10);
	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);
	*/
}

// skateboard ISRs below  vvv

ISR(INT0_vect)	//if INT0 is going high + - Z   else if INT0 going low - + Z
{
	//testing only
	//LED_STATUS_TOGGLE;
	//_delay_ms(500);
	//LED_STATUS_TOGGLE;


	rotationCounter = 0;
	PHASES_ALL_HIGH_OFF;
	PHASE_U_LOW_OFF;
	PHASE_V_LOW_OFF;
	PHASE_W_LOW_OFF;
		
	if ((PIND & 64) == 64) {
		//_delay_ms(0.25);
		// 3
		if (WHICH_DIRECTION == ANTI_CLOCK_WISE)
		{
			//anti-clock wise
			//PHASE_U_HIGH_ON;
			//PHASE_W_LOW_ON;
			PHASE_V_LOW_ON;
			PHASE_U_HIGH_ON;
		}else{
			//clock wise
			//PHASE_U_LOW_ON;
			//PHASE_W_HIGH_ON;
			PHASE_U_LOW_ON;
			PHASE_V_HIGH_ON;
		}
		if(startPhase == 3){
			revolutions++;
		}
	} else {
		//_delay_ms(0.25);
		// 4
		if (WHICH_DIRECTION == ANTI_CLOCK_WISE)
		{
			//anti-clock wise
			//PHASE_U_LOW_ON;
			//PHASE_W_HIGH_ON;
			PHASE_V_LOW_ON;
			PHASE_W_HIGH_ON;
		}else{
			//clock wise
			//PHASE_W_LOW_ON;
			//PHASE_U_HIGH_ON;
			PHASE_W_LOW_ON;
			PHASE_V_HIGH_ON;
		}
		if(startPhase == 4){
			revolutions++;
		}
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
		// 6
		//_delay_ms(0.25);
		if (WHICH_DIRECTION == ANTI_CLOCK_WISE)
		{
			//anti-clock wise
			//PHASE_U_LOW_ON;
			//PHASE_V_HIGH_ON;
			PHASE_U_LOW_ON;
			PHASE_V_HIGH_ON;
		}else{
			//clock wise
			//PHASE_V_LOW_ON;
			//PHASE_U_HIGH_ON;
			PHASE_V_LOW_ON;
			PHASE_U_HIGH_ON;
		}
		if(startPhase == 6){
			revolutions++;
		}
	} else {
		// 1
		//_delay_ms(0.25);
		if (WHICH_DIRECTION == ANTI_CLOCK_WISE)
		{
			//anti-clock wise
			//PHASE_U_HIGH_ON;
			//PHASE_V_LOW_ON;
			PHASE_W_LOW_ON;
			PHASE_V_HIGH_ON;
		}else{
			//clock wise
			//PHASE_U_LOW_ON;
			//PHASE_V_HIGH_ON;
			PHASE_V_LOW_ON;
			PHASE_W_HIGH_ON;
		}
		if(startPhase == 1){
			revolutions++;
		}
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
		// 5
		//_delay_ms(0.25);
		if (WHICH_DIRECTION == ANTI_CLOCK_WISE)
		{
			//anti-clock wise
			//PHASE_W_HIGH_ON;
			//PHASE_V_LOW_ON;
			PHASE_U_LOW_ON;
			PHASE_W_HIGH_ON;
		}else{
			//clock wise
			//PHASE_W_LOW_ON;
			//PHASE_V_HIGH_ON;
			PHASE_W_LOW_ON;
			PHASE_U_HIGH_ON;
		}
		if(startPhase == 5){
			revolutions++;
		}
	} else {
		// 2
		//_delay_ms(0.25);
		if (WHICH_DIRECTION == ANTI_CLOCK_WISE)
		{
			//anti-clock wise
			//PHASE_V_HIGH_ON;
			//PHASE_W_LOW_ON;
			PHASE_W_LOW_ON;
			PHASE_U_HIGH_ON;
		}else{
			//clock wise
			//PHASE_V_LOW_ON;
			//PHASE_W_HIGH_ON;
			PHASE_U_LOW_ON;
			PHASE_W_HIGH_ON;
		}
		if(startPhase == 2){
			revolutions++;
		}
	}
}

void kickMotor(void)
{
	switch (getMotorPosition())
	{
		case 1:
			// 1
			//_delay_ms(0.25);
			if (WHICH_DIRECTION == ANTI_CLOCK_WISE)
			{
				//anti-clock wise
				//PHASE_U_HIGH_ON;
				//PHASE_V_LOW_ON;
				PHASE_W_LOW_ON;
				PHASE_V_HIGH_ON;
			}else{
				//clock wise
				//PHASE_U_LOW_ON;
				//PHASE_V_HIGH_ON;
				PHASE_V_LOW_ON;
				PHASE_W_HIGH_ON;
			}
			LED_STATUS_TOGGLE;
			startPhase = 1;
			break;
		case 2:
			// 2
			//_delay_ms(0.25);
			if (WHICH_DIRECTION == ANTI_CLOCK_WISE)
			{
				//anti-clock wise
				//PHASE_V_HIGH_ON;
				//PHASE_W_LOW_ON;
				PHASE_W_LOW_ON;
				PHASE_U_HIGH_ON;
			}else{
				//clock wise
				//PHASE_V_LOW_ON;
				//PHASE_W_HIGH_ON;
				PHASE_U_LOW_ON;
				PHASE_W_HIGH_ON;
			}
			LED_STATUS_TOGGLE;
			startPhase = 2;
		case 3:
			// 3
			//_delay_ms(0.25);
			if (WHICH_DIRECTION == ANTI_CLOCK_WISE)
			{
				//anti-clock wise
				//PHASE_U_HIGH_ON;
				//PHASE_W_LOW_ON;
				PHASE_V_LOW_ON;
				PHASE_U_HIGH_ON;
			}else{
				//clock wise
				//PHASE_U_LOW_ON;
				//PHASE_W_HIGH_ON;
				PHASE_U_LOW_ON;
				PHASE_V_HIGH_ON;
		}
		LED_STATUS_TOGGLE;
			startPhase = 3;
		break;
		case 4:
			// 4
			//_delay_ms(0.25);
			if (WHICH_DIRECTION == ANTI_CLOCK_WISE)
			{
				//anti-clock wise
				//PHASE_U_LOW_ON;
				//PHASE_W_HIGH_ON;
				PHASE_V_LOW_ON;
				PHASE_W_HIGH_ON;
			}else{
				//clock wise
				//PHASE_W_LOW_ON;
				//PHASE_U_HIGH_ON;
				PHASE_W_LOW_ON;
				PHASE_V_HIGH_ON;
			}
			LED_STATUS_TOGGLE;
			startPhase = 4;
		break;
		case 5:
			// 5
			//_delay_ms(0.25);
			if (WHICH_DIRECTION == ANTI_CLOCK_WISE)
			{
				//anti-clock wise
				//PHASE_W_HIGH_ON;
				//PHASE_V_LOW_ON;
				PHASE_U_LOW_ON;
				PHASE_W_HIGH_ON;
			}else{
				//clock wise
				//PHASE_W_LOW_ON;
				//PHASE_V_HIGH_ON;
				PHASE_W_LOW_ON;
				PHASE_U_HIGH_ON;
			}
			LED_STATUS_TOGGLE;
			startPhase = 5;
		break;
		case 6:
			// 6
			//_delay_ms(0.25);
			if (WHICH_DIRECTION == ANTI_CLOCK_WISE)
			{
				//anti-clock wise
				//PHASE_U_LOW_ON;
				//PHASE_V_HIGH_ON;
				PHASE_U_LOW_ON;
				PHASE_V_HIGH_ON;
			}else{
				//clock wise
				//PHASE_V_LOW_ON;
				//PHASE_U_HIGH_ON;
				PHASE_V_LOW_ON;
				PHASE_U_HIGH_ON;
			}
			startPhase = 6;
			LED_STATUS_TOGGLE;
		break;
		default:
		break;
	}
}

// gets an encoded version of motor position based on hall states
// adds 4 if HALL3 is high, 2 if HALL2 is HIGH and 1 if HALL 1 is high giving positions 1..6 in total
uint8_t getMotorPosition(void)
{		// ((? & 100000) / 8) +  ((? & 100) / 2)   + ((? & 1000000) / 64)
	return (((PINB & 32) / 8) + ((PINB & 4) / 2) + ((PIND & 64) / 64));
}


void toggle_led(void){
	PORTB ^= 0b00001000;
}

/************************************************************************/
/*					CODE CEMETERY                                       */
/************************************************************************/
/*

//DDRD = (1<<DDD2);

DDRD   = 1<<DDD3;    // OC0A
TCCR0A = (1<<WGM01); // ctc mode
TCCR0B = (1<<CS02);  // presc 256
OCR0A  = 0x1388;       // 1000 ms period


//Intial_Timer();
//////////////////////////////////////
void mil_timer(uint8_t millis){
	TCCR0A |= (1<<WGM01);
	TCCR1B |= (1<<CS02)|(1<<CS00);

	OCR0A = millis*7;

	TIMSK0 |= (1<<OCIE0A);
}


int main(void)
{
	DDRB |= (1<<DDB5);
	DDRB &= ~(1<<DDB7);

	sei();

	while(1){

	}

}

//////////////////////////////////////


// TIMER
// Set up clock source according to configuration
ASSR = TC_TIMEOUT_ASSR_MASK;

PORTB = 0b00000000;
DDRB |= 0b00001000;				// make the status LED an output
DDRB |= (1<<DDB5);
DDRB &= ~(1<<DDB7);

TCCR1B |= (1<<CS12);
TIMSK1 |= (1<TOIE1);

*/

/*
	while(1){
	_delay_ms(1000);
	toggle_led();
	*/
		/*
		 //while(!(TIFR1 & (1<<TOV1)));// Wait until cycle completes
		 while(!(TIFR0 & (1<<OCF0A)));
		 DDRD   = 1<<DDD3;    // OC0A
		 TCCR0A = (1<<WGM01); // ctc mode
		 TCCR0B = (1<<CS02);  // presc 256
		 OCR0A  = 0x3E8;       // 1000 ms period
		 */
		 //toggle_led();
	//}

	/*
	PWM code. DO NOT CHANGE
	
	//PLL 32MHz, for 15.62kHz PWM
	//RPM = 15620 * (2  / 48 ) * 60
	//RPM = Hz * (2  / poles ) * 60
	// Hz = Cycles / sec
	//


	
	// 64MHz PLL input / 2048 = 31.25kHz PWM

	void InitializePSC()
	{
		
		PLLCSR  = BIT(PLLF) | BIT(PLLE);  // PLL set to 64MHz enabled
		
		POC     = BIT(POEN0A);    // enable PSCOUT0A only
		PSYNC   = 0x00;
		PCNF    = BIT(POPA);     // Active HIGH
		POCR_RB = 2048;          // modulo for counter
		POCR0SA = 0x0000;        // start cycle high
		POCR0SB = 0x0fff;        // never set B high
		PCTL    = BIT(PCLKSEL) | BIT(PRUN);   // Select PLL clock no div
	}

	void SetPSCOutput( WORD w ) // 0-2047
	{
		POCR0RA = w;  // Set outa low @ this count
	}
	


	
ISR(TIMER1_OVF_vect)
{
	//toggle_led();
}

void set_bits_func (volatile uint8_t *port, uint8_t mask)
{
	*port |= mask;
}

*/