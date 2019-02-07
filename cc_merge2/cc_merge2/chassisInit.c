
/**
 * @file chassisInit.c
 * @author Jonn Dillon
 * @date 16/4/2018
 * @brief Handles all initialisation processes for the chassis controller
 **/


#include "chassisInit.h"
#include "main.h"

/**
 * @brief Sets up the microcontroller to allow external interrupts. The External Interrupts are triggered by the INT7:0 pin or any of the PCINT23:0 pins.
 * 
 * Reference: ATmega Datasheet Chapter 15 (External Interrupts)
 * 
 */
void external_interrupt_init()
{
	//INT1 for CAN1, INT0 for CAN2, PCINT7 for CAN3
	EICRA |= (2<<ISC00)|(2<<ISC10);
	EIMSK  = (1<<INT0) | (1<<INT1);
	//Enable interrupts for PCINT7:0
	PCICR  = (1<<PCIE0)|(0<<PCIE1);
	//CAN C interrupt
	PCMSK0 = (1<<PCINT7);
	// Dig In #1 interrupt - not needed now
	//PCMSK1 = (1<<PCINT15);
}

/**
 * @brief Configures the pins required for IO. Each port pin consists of three register bits: DDxn, PORTxn, and PINxn set by the following registers
 * 
 * 	DDRx   - Sets the direction of PINxn in bit DDxn (1 -> output, 0 -> input)
 * 
 * 	PORTx  - Activates the pull-up resistor for PINxn in bit PORTxn. The pull-up resistor will only be activated if written as logic 1 AND set as an input
 * 
 *  PINx   - The Port Input Pins I/O location is read only. 
 *           Writing a logic one to a bit in the PINx Register, will result in a toggle in the corresponding bit in the Data Register.
 * 		     Writing a logic one to PINxn toggles the value of PORTxn, independent on the value of DDRxn
 * 
 * Reference: ATmega Datasheet Chapter 13 (I/O-Ports)
 * 
 */
void io_init()
{
	//pins 8, 9 and 14 for MCP2515_STB high so the things respond (PE6, PE7, PH2)
	DDRE  = 0b11000010;		//PE0 = RS232 RX1; PE1 = RS232 TX1;  PE6 = STB_CAN1; PE7 = STB_CAN2;
	PORTE = 0b00000000;		
	DDRH  = 0b00000111;		//PH0 = CS_CAN1; PH1 = CS_CAN2; PH2 = STB_CAN3
	PORTH = 0b00000011;		//CS_CAN1 high; CS_CAN2 high;
	//pins 12, 13, 19 for the CS for each MCP2515 PH0, PH1, PB0)

	//pin 21 for MOSI, pin 20  for SCK (PB2, PB1)
	DDRB  = 0b01100111;		//PB0 = CS_CAN3; PB1 = SCK; PB2 = MOSI; PB3 = MISO; PB5 = High drive A; PB6 = Low drive A; PB7 = CAN3_INT; 
	PORTB = 0b00000001;		//set CS_CAN3 high;
	
	DDRL  = 0b00011000;		//PB3 = High drive B; PB4 = Low Drive B;
	PORTL = 0b00000000;
	
	DDRD  = 0b11001000;		//PD0 = CAN2_INT; PD1 = CAN1_INT; PD2 = RS232 RX2; PD3 = RS232 TX2; PD6 = CAN1_TXPIN; PD7 = CAN2_TXPIN;
	DDRD  = 0b00000000;
	
	DDRC  = 0b00001000;		//PC3 = CAN3_TXPIN;
	PORTC = 0b00000000;
	
	DDRJ  = 0b00000000;		//PORTJ is used for digital input;
	PORTJ |= 64;			//turn the pull-ups on for digital inputs
	
	DDRA  = 0b00011000;		//PA3 = ENABLE_B; PA4 = ENABLE_A; PA1 = dig input; PA2 = dig input;
	PORTA = 0b00010000;		

	DDRK  = 0b00100000;		//PK5 = debugging LED;
	// PORTK = 0b00100000;
	
	// Enable external interrupts in order for the CAN bus to communicate with us
	external_interrupt_init();
}

/**
 * @brief Set up all devices in the ATmega and MCP2515. Initiates structs to hold data from other devices
 * 
 */
void firmware_init()
{
	io_init();
	SPI_init();
    // // uart_init(UART_BAUD_SELECT(19200, 16UL));
    uart_init(19200);
    //uart1_init(19200);
	a2dInit(ADC_PRESCALE_DIV64, ADC_REFERENCE_AVCC); // Turns ON ADC
	MCP2515_init(TRACTIVE_CAN);
	MCP2515_init(POWER_CAN);
	MCP2515_init(DATA_CAN);

	// Enable the pullup on the input. This allows the pin to be active low
	PORTJ |= (1<<PINJ6);

	// Initialise inverter structs
	// for(uint8_t i = 0; i < NUM_INVERTERS; i++)
	// {
	// 	inverters[i].ID=1<<i;
	// 	inverters[i].current = 0;
	// 	inverters[i].duty = 0;
	// 	inverters[i].RPM = 0;
	// 	inverters[i].temperature = 0;
	// }

	// accumulators[0].ID=ACCUMULATOR_FRONT;
}

/**
 * @brief Initiates a timer set on Clear Timer Compare Match (CTC) Mode.
 * 
 * Reference: ATmega Datasheet Chapter 17 (16-bit Timer/Counter)
 * 
 */
void timer_init()
{
    // http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-2549-8-bit-AVR-Microcontroller-ATmega640-1280-1281-2560-2561_datasheet.pdf
    // Set up 1Khz timer
    TCCR0A |= (1 << WGM01);                 // Setting CTC on timer0
    TCCR0B |= (1 << CS01)|(1 << CS00);      // Set up 64 prescaler (16Mhz / 64)
    OCR0A = 250;                            // (16*10^6) / (1000 * 64) assuming 16Mhz chip = 1Khz
    TIMSK0 |= (1 << OCIE0A);                // Enable COMPA interupt

    // Set up 50Hz comm timer
	TCCR1A = 0b00000000;			//
	TCCR1B = 0b00001101;			// CTC mode & prescale clock by 1024
	OCR1A =  15000;					// 312 gives 50Hz main comms speed
	TIMSK1 = 0b00000010;			// Turn on compare interrupt for OCR1A
}