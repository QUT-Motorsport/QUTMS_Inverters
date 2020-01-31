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
#include "adc.h"

#define CAN_INV_ID 0x4000000

//#define TESTMOTOR 0 // COMMENT OUT THIS LINE TO MAKE THE PLETENBUG MOTORS RUN
#define PLETTENMOTOR 0 // COMMENT OUT THIS LINE TO MAKE THE PLETENBUG MOTORS RUN
// #define CLOCKWISE 0 // COMMENT OUT THIS LINE TO MAKE THEM RUN ANTI-CLOCKWISE
#define ANTICLOCKWISE 0 // COMMENT OUT THIS LINE TO MAKE THEM RUN ANTI-CLOCKWISE

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

uint8_t canArray[8] = {0, 0, 0, 0, 0, 0, 0, 0};
volatile uint8_t canFlag = 0;

#define ADC_SAMPLES 10
uint8_t adcBufferCountPhaseW = 0;
uint8_t adcBufferPhaseW[ADC_SAMPLES];
uint8_t adcBufferCountPhaseV = 0;
uint8_t adcBufferPhaseV[ADC_SAMPLES];
uint8_t adcBufferCountPowerB = 0;
uint8_t adcBufferPowerB[ADC_SAMPLES];
uint8_t adcBufferCountTemp = 0;
uint8_t adcBufferTemp[ADC_SAMPLES];

uint8_t currentPhaseW = 0;
uint8_t currentPhaseV = 0;
uint8_t currentPhaseU = 0;
uint8_t currentPowerB = 0;

/*
 * Test Motor Commutation Order
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
 *
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
 */

/*
 * Plettenburg Commutation Order
 *
 *		Clockwise
 *		Motor					Sensor (CW)
 *		Y(U)	G(V)	B(W)	Y	G	B
 * -----------------------------------------
 * 1	+		-		X		+	+	-
 * 2	X		X		-		-	+	-
 * 3	-		+		-		-	+	+
 * 4	-		+		X		-	-	+
 * 5	X		X		+		+	-	+
 * 6	+		-		+		+	-	-
 *
 *
 *		Counter-Clockwise
 *		Motor					Sensor (CCW)
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
void spi_init()
{
	//MCUCR &= ~(1<<SPIPS);		//make sure we are getting spi not on the _A spi pins.
	SPCR = (0<<SPIE)|(1<<SPE)|(0<<DORD)|(1<<MSTR)|(0<<CPOL)|(0<<CPHA)|(0<<SPR0)|(0<<SPR1);
	//set interrupt, enable SPI, MSB mode, set as master, low is idle, read on leading edge, Set to speed(fosc/4) SPR0=1, SPR1=0
	SPSR = (1<<SPI2X);	//set whether we want 2x speed or not (1=2x speed).
	SPDR = 0x00;		//ensure data register has nothing in it
}

uint8_t spi_send_byte(uint8_t c)
{
	SPDR = c;					//write data to the transmission register. Writing to this initiates transmission.
	while(!(SPSR & (1<<SPIF)));
	return SPDR;				//return any data that was shifted into this register upon transmission.
}

void ADC_init()
{
	ADMUX=(1<<REFS0)|(1<<AREFEN);						// For Aref = AVcc with external capacitor;
	ADMUX &= ~(1<<ADLAR);								// Make sure adlar is not set.
	ADCSRA=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);	// Prescaler div factor = 128, 125kHz --> lowest we can go for best accuracy.
}
*/

/*
 * Channels to sense for current sensing
 * B	= Pin 22, PC6, ADC 10 = Channel 10
 * W	= Pin 18, PC5, ADC 9  = Channel 9
 * V	= Pin 17, PC4, ADC 8  = Channel 8
 * Temp = Pin 13, PD5, ADC 2  = Channel 2
 *//*
uint16_t ADC_read(uint8_t channel)
{
	channel = (ADMUX & 0xe0)|(channel & 0x1F); //ADMUX | 0b11100000 and channel | 0b00011111 --> this keeps all bits of ADMUX the same except for the bits signalling which channel to use.
	ADMUX = channel;
	ADCSRA |= (1<<ADSC);							//ADSC (single conversion bit) is set to 1 to start the conversion process.
	while(!(ADCSRA & (1<<ADIF)));				//run a loop while the conversion is taking place.
	uint16_t result = 0;
	result = ADCL;								//read ADCL first, ADCH after --> order is important! --> also not sure if this code is correct. other ADC examples return 'ADC' instead.
	result |= ((3 & ADCH) << 8);
	ADCSRA|=(1<<ADIF);							//once read and done, clear the 'complete' status by writing 1 to the ADIF bit.
	return result;								//pass the 10 bit ADC number to requesting function.
}
*/
/*
uint8_t ADC_read_phase_W() {
	static uint16_t outPhaseW = 0;
	adcBufferPhaseW[adcBufferCountPhaseW] = ADC_read(9) >> 2;
	adcBufferCountPhaseW++;
	adcBufferCountPhaseW = adcBufferCountPhaseW % ADC_SAMPLES;
	for(uint8_t i = 0; i < ADC_SAMPLES; i++) {
		outPhaseW = outPhaseW + adcBufferPhaseW[i];
	}
	outPhaseW = outPhaseW / ADC_SAMPLES;
	return outPhaseW;
}

uint8_t ADC_read_phase_V() {
	static uint16_t outPhaseV = 0;
	adcBufferPhaseV[adcBufferCountPhaseV] = ADC_read(8) >> 2;
	adcBufferCountPhaseV++;
	adcBufferCountPhaseV = adcBufferCountPhaseV % ADC_SAMPLES;
	for(uint8_t i = 0; i < ADC_SAMPLES; i++) {
		outPhaseV = outPhaseV + adcBufferPhaseV[i];
	}
	outPhaseV = outPhaseV / ADC_SAMPLES;
	return outPhaseV;
}

uint8_t ADC_read_power_B() {
	static uint16_t outPowerB = 0;
	adcBufferPowerB[adcBufferCountPowerB] = ADC_read(10) >> 2;
	adcBufferCountPowerB++;
	adcBufferCountPowerB = adcBufferCountPowerB % ADC_SAMPLES;
	for(uint8_t i = 0; i < ADC_SAMPLES; i++) {
		outPowerB = outPowerB + adcBufferPowerB[i];
	}
	outPowerB = outPowerB / ADC_SAMPLES;
	return outPowerB;
}

uint8_t ADC_read_temp() {
	static uint16_t outTemp = 0;
	adcBufferTemp[adcBufferCountTemp] = ADC_read(2) >> 2;
	adcBufferCountTemp++;
	adcBufferCountTemp = adcBufferCountTemp % ADC_SAMPLES;
	for(uint8_t i = 0; i < ADC_SAMPLES; i++) {
		outTemp = outTemp + adcBufferTemp[i];
	}
	outTemp = outTemp / ADC_SAMPLES;
	return outTemp;
}
*/

/* UNTESTED
uint16_t ADC_read_motor_temp() {
	// Use SPI to query the other ADC Chip -> MCP3002

	// MCP3002 SPI Config Bits
	//				| Config Bits			| Channel Selection
	//				| SGL/DIFF	| ODD/SIGN	| CH 0		| CH 1
	// -----------------------------------------------------------
	// Single Ended	| 1			| 0			| +			|
	// Mode			| 1			| 1			|			| +
	// -----------------------------------------------------------
	// Pesudo		| 0			| 0			| IN+		| IN-
	// Differential	| 0			| 1			| IN-		| IN+
	// Mode			|			|			|

	// CH 0 => Gearbox Temp
	// CH 1 => High Voltage

	// Bit Order
	//                                  | Most Significant Bit First |
	//    | Start | SGL/DIFF | ODD/SIGN	| MSBF                       | 0 | 0 | 0 | 0
	// 0b | 1	  | 1        | 0        | 1                          | 0 | 0 | 0 | 0
	// Bin => 0b11010000
	// Hex => D0
	// Dec => 208

	PORTC &= 0b10000000;
	uint16_t motorTemp = spi_send_byte(0b11010000); // Apparently all that is needed?
	PORTC |= 0b00000000;
	return motorTemp;
}*/

/* UNTESTED
void currents_calculate() {
	currentPhaseW = ADC_read_phase_W();
	currentPhaseV = ADC_read_phase_V();
	currentPowerB = ADC_read_power_B();
	currentPhaseU = currentPowerB - (currentPhaseW + currentPhaseV);
} */

ISR(CAN_INT_vect)
{
    //uint8_t authority;
    //uint8_t mob;
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

        // canFlag = 1;

        CAN_RXInit(5, 8, 0x4000000, 0x4000000, 1);
    }
}

uint16_t readVoltage() {
    uint16_t adcRawValue = adc_read(10);
    // 2560 is ref voltage from datasheet lol
    uint16_t analogeMV = adcRawValue * (2560) / 1024;
    return analogeMV;
}

uint16_t canCount = 0;
uint8_t tempCount = 0;

int main(void)
{
    DDRB |= 0b11001011;				// make the status LED an output
    DDRC |= 0b10000101;
    DDRD |= 0b10000001;				// PD7 is CAN STB

    PORTB &= ~0b11001011;
    PORTC &= ~0b00000001;		// Byte 7 = ADC_CS pin
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

    adc_init();

    // Init ADC
    // ADC_init();
    // Init SPI
    // spi_init();

    // start the CAN interface
    CAN_init();		// Initialise CAN
    CAN_RXInit(5, 8, 0x4000000, 0x4000000, 1);

    // start the interrupts
    sei();

    //turn the outputs off
    PHASES_ALL_HIGH_OFF;
    PHASE_U_LOW_OFF;
    PHASE_V_LOW_OFF;
    PHASE_W_LOW_OFF;

    while(1)
    {
        // Motor Speed Calculation
        // --------------------------------------------------------------------

        motorCommand = 250 - (testChar * 2.2); // Generating PWM duty cycle - Lower value = more on time cycle wise
        if(motorCommand < 20) motorCommand = 20; // Limit for power applied
        // If we are spinning the motor
        if(motorState == 1) {
            // Set PWM, dead time etc to motor speed command
            POCR0SA = POCR1SA = POCR2SA = motorCommand;
            POCR0SB = POCR1SB = POCR2SB = motorCommand - 10;
        } else {
            // else  off all outputs now
            POC = 0b00000000;
            PHASE_U_LOW_OFF;
            PHASE_V_LOW_OFF;
            PHASE_W_LOW_OFF;
        }

        // Roll stop code
        // --------------------------------------------------------------------

        // If counter is under 1000, increment.
        if(rotationCounter < 1000) rotationCounter ++;
        // If rotation counter is at or greater than 1000
        if(rotationCounter > 999) {
            // (at 1000) Turn of motor state machine
            motorState = 0;
            // Turn on LED
            PORTB &= ~8;
        } else {
            // else (less than 1000) Turn on motor state machine
            motorState = 1;
            PORTB |= 8;
        }

        // Start up code
        // --------------------------------------------------------------------

        // IF statement to handle start up
        if((motorState == 0) && (motorCommand < 150)) { // If we are 'off' and the motor is not spinning at some 'fast speed'. Was motorCommand < 225, but assmed for high speed 90V testing
            // This function would run always, but its not supposed to run at high speeds to save clock cycles for the interupt driven operation
            kickMotor(); // Run the low speed force sense and rotate function
            motorState = 1;	// We are now running
            rotationCounter = 0; // Reset rotation counter
        }


        canCount++;

        if(canCount > 100) {
            canCount = 0;
            uint16_t voltVal = readVoltage();
            canArray[0] = voltVal >> 8;
            canArray[1] = voltVal;
            canArray[2] = 0x01;
            canArray[3] = tempCount;
            CANGCON &= ~(1 << ABRQ);
            CAN_TXMOB(CAN_findFreeTXMOB(), 4, canArray, CAN_INV_ID, 0x0100);
            tempCount++;
        }

        // if(canFlag == 1) {
        // canFlag = 0;
        // canArray[0] = motorState;
        // canArray[1] = motorCommand;
        // canArray[2] = rotationCounter >> 8;
        // canArray[3] = rotationCounter;
        // canArray[4] = ADC_read_phase_W();
        // canArray[5] = ADC_read_phase_V();
        // canArray[6] = ADC_read_power_B();
        // canArray[7] = ADC_read_temp();
        // CAN_TXMOB(CAN_findFreeTXMOB(), 8, canArray, 0x5000000, 0x0100);
        // }
    }
}

ISR(INT0_vect)	//if INT0 is going high + - Z   else if INT0 going low - + Z
{
    PORTB ^= 8;
    rotationCounter = 0;
    PHASES_ALL_HIGH_OFF;
    PHASE_U_LOW_OFF;
    PHASE_V_LOW_OFF;
    PHASE_W_LOW_OFF;

    if ((PIND & 64) == 64) { // GOING HIGH
#ifdef TESTMOTOR // The Skateboard Motor	
#ifdef CLOCKWISE // Running Clockwise				> STEP 6
        PHASE_V_LOW_ON;
        PHASE_W_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 1
        // PHASE_V_LOW_ON;
        // PHASE_U_HIGH_ON;
#endif
#endif
#ifdef PLETTENMOTOR // The Pletenburg Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 4
        PHASE_U_LOW_ON;
        PHASE_W_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 2
        PHASE_V_LOW_ON;
        PHASE_W_HIGH_ON;
#endif
#endif
    } else { // GOING LOW
#ifdef TESTMOTOR // THe Skateboard Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 3
        PHASE_V_HIGH_ON;
        PHASE_W_LOW_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 6
        // PHASE_V_LOW_ON;
        // PHASE_W_HIGH_ON;
#endif
#endif
#ifdef PLETTENMOTOR // The Pletenburg Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 1
        PHASE_W_LOW_ON;
        PHASE_U_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 5
        PHASE_V_HIGH_ON;
        PHASE_W_LOW_ON;
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
#ifdef CLOCKWISE // Running Clockwise				> STEP 2
        PHASE_U_HIGH_ON;
        PHASE_W_LOW_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 4
        // PHASE_U_LOW_ON;
        // PHASE_V_HIGH_ON;
#endif
#endif
#ifdef PLETTENMOTOR // The Pletenburg Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 6
        PHASE_V_LOW_ON;
        PHASE_U_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 6
        PHASE_U_HIGH_ON;
        PHASE_W_LOW_ON;
#endif
#endif
    } else {
#ifdef TESTMOTOR // THe Skateboard Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 5
        PHASE_U_LOW_ON;
        PHASE_W_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 3
        // PHASE_W_LOW_ON;
        // PHASE_V_HIGH_ON;
#endif
#endif
#ifdef PLETTENMOTOR // The Pletenburg Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 3
        PHASE_U_LOW_ON;
        PHASE_V_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 3
        PHASE_W_HIGH_ON;
        PHASE_U_LOW_ON;
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
#ifdef CLOCKWISE // Running Clockwise				> STEP 4
        PHASE_U_LOW_ON;
        PHASE_V_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 5
        // PHASE_U_LOW_ON;
        // PHASE_W_HIGH_ON;
#endif
#endif
#ifdef PLETTENMOTOR // The Pletenburg Motor				> STEP 2
#ifdef CLOCKWISE // Running Clockwise
        PHASE_W_LOW_ON;
        PHASE_V_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 4
        PHASE_V_HIGH_ON;
        PHASE_U_LOW_ON;
#endif
#endif
    } else {
#ifdef TESTMOTOR // THe Skateboard Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 1
        PHASE_U_HIGH_ON;
        PHASE_V_LOW_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 2
        // PHASE_W_LOW_ON;
        // PHASE_U_HIGH_ON;
#endif
#endif
#ifdef PLETTENMOTOR // The Pletenburg Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 5
        PHASE_V_LOW_ON;
        PHASE_W_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 1
        PHASE_V_LOW_ON;
        PHASE_U_HIGH_ON;
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
#ifdef CLOCKWISE // Running Clockwise				> STEP 1
        PHASE_U_HIGH_ON;
        PHASE_V_LOW_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 3
        // PHASE_W_LOW_ON;
        // PHASE_V_HIGH_ON;
#endif
#endif
#ifdef PLETTENMOTOR // The Pletenburg Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 3
        PHASE_U_LOW_ON;
        PHASE_V_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 3
        PHASE_U_HIGH_ON;
        PHASE_V_LOW_ON;
#endif
#endif
        break;
    case 2:
#ifdef TESTMOTOR // THe Skateboard Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 5
        PHASE_U_LOW_ON;
        PHASE_W_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 2
        // PHASE_W_LOW_ON;
        // PHASE_U_HIGH_ON;
#endif
#endif
#ifdef PLETTENMOTOR // The Pletenburg Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 5
        PHASE_V_LOW_ON;
        PHASE_W_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 1
        PHASE_W_LOW_ON;
        PHASE_V_HIGH_ON;
#endif
#endif
        break;
    case 3:
#ifdef TESTMOTOR // THe Skateboard Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 6
        PHASE_V_LOW_ON;
        PHASE_W_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 1
        // PHASE_V_LOW_ON;
        // PHASE_U_HIGH_ON;
#endif
#endif
#ifdef PLETTENMOTOR // The Pletenburg Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 4
        PHASE_U_LOW_ON;
        PHASE_W_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 2
        PHASE_W_LOW_ON;
        PHASE_U_HIGH_ON;
#endif
#endif
        break;
    case 4:
#ifdef TESTMOTOR // THe Skateboard Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 3
        PHASE_V_HIGH_ON;
        PHASE_W_LOW_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 6
        // PHASE_V_LOW_ON;
        // PHASE_W_HIGH_ON;
#endif
#endif
#ifdef PLETTENMOTOR // The Pletenburg Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 1
        PHASE_W_LOW_ON;
        PHASE_U_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 5
        PHASE_W_HIGH_ON;
        PHASE_U_LOW_ON;
#endif
#endif
        break;
    case 5:
#ifdef TESTMOTOR // THe Skateboard Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 2
        PHASE_U_HIGH_ON;
        PHASE_W_LOW_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 5
        // PHASE_U_LOW_ON;
        // PHASE_W_HIGH_ON;
#endif
#endif
#ifdef PLETTENMOTOR // The Pletenburg Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 2
        PHASE_W_LOW_ON;
        PHASE_V_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 4
        PHASE_W_HIGH_ON;
        PHASE_V_LOW_ON;
#endif
#endif
        break;
    case 6:
#ifdef TESTMOTOR // THe Skateboard Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 4
        PHASE_U_LOW_ON;
        PHASE_V_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 4
        // PHASE_U_LOW_ON;
        // PHASE_V_HIGH_ON;
#endif
#endif
#ifdef PLETTENMOTOR // The Pletenburg Motor
#ifdef CLOCKWISE // Running Clockwise				> STEP 6
        PHASE_V_LOW_ON;
        PHASE_U_HIGH_ON;
#endif
#ifdef ANTICLOCKWISE // Running Anti-Clockwise		> STEP 6
        PHASE_U_LOW_ON;
        PHASE_V_HIGH_ON;
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