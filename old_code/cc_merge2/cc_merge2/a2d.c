/*! \file a2d.c \brief Analog-to-Digital converter function library. */
//*****************************************************************************
//
// File Name	: 'a2d.c'
// Title		: Analog-to-digital converter functions
// Author		: William Bevan - Copyright (C) 2008
// Created		: 2008-05-16
// Revised		: 2008-05-16
// Version		: 1.0
// Target MCU	: Atmel AVR series
//
// This code is distributed under the GNU Public License
//		which can be found at http://www.gnu.org/licenses/gpl.txt
//
//*****************************************************************************

#include <avr/io.h>

#include "a2d.h"
#include <stdio.h>
#include <string.h>			// include string support

// functions

// initialize a2d converter
void a2dInit(unsigned char prescale, unsigned char ref)
{
	// Set to Single Mode, ADC Auto Trigger OFF, ADC Int OFF, Prescale 0
	ADCSRA = 0; //clear!

	// Right adjust result
	ADMUX &= ~(1<<ADLAR); //clear

	// Set prescaler
	a2dSetPrescaler(prescale);

	// Set default reference
	a2dSetReference(ref);	

	// Turn ADC on
	a2dOn();

	// int input = a2d_10bit();
	// input = a2d_10bit();
}

// turn On a2d converter
void a2dOn(void)
{
	ADCSRA |= (1 << ADEN);				// enable ADC (turn on ADC power), Set
}

// turn off a2d converter
void a2dOff(void)
{
	ADCSRA &= ~(1 << ADEN);				// disable ADC (turn off ADC power), Clear
}

// configure A2D converter clock division (prescaling)
void a2dSetPrescaler(unsigned char prescale)
{
	ADCSRA &= ADC_PRESCALE_MASK;
	ADCSRA |= prescale;
}

// configure A2D converter voltage reference
void a2dSetReference(unsigned char ref)
{
	ADMUX &= ADC_REFERENCE_MASK;
	ADMUX |= (ref<<6);
}

// sets the a2d input channel
void a2dSetChannel(unsigned char ch)
{
	ADMUX &= ADC_MUX_MASK;
	ADMUX |= ch;
}


// Perform a 10-bit conversion on current channel
// starts conversion, waits until conversion is done, and returns result
unsigned short a2d_10bit()
{
	// start conversion
	ADCSRA |= (1<<ADSC);

	while(ADCSRA & (1 << ADSC))
	{
		// wait until conversion complete
	}

    return ADC;                // read ADC (full 10 bits);
	// return (8 << ADCH)|(ADCL); // read ADC (full 10 bits);
    // return ADCL;                // read ADC (first 8 bits)
}

// Perform a 8-bit conversion on current channel
// starts conversion, waits until conversion is done, and returns result
unsigned char a2d_8bit()
{
	// do 10-bit conversion and return highest 8 bits
	return a2d_10bit()>>2;			// return ADC MSB byte
}




// Perform a 10-bit conversion on given channel
// starts conversion, waits until conversion is done, and returns result
// modded for mega1280 by Ant
unsigned short a2d_10bitCh(unsigned char ch)
{
	// set channel
	ADMUX &= ADC_MUX_MASK;	
	
	if (ch > 7) 
	{
		ADCSRB |= 8;
		ADMUX |= (ch - 8);
	}
	else
	{
		ADCSRB &= 247;
		ADMUX |= ch;
	}
	
	// ADMUX |= ch;	//comment out this line when using on 16ch parts

	// start conversion
	ADCSRA |= (1<<ADSC);

	while(ADCSRA & (1 << ADSC)) {} // wait until conversion complete 

	return ADC;                // read ADC (full 10 bits);
	// return (8 << ADCH)|(ADCL); // read ADC (full 10 bits);
    // return ADCL;                // read ADC (first 8 bits)
}


// Perform a 8-bit conversion on given channel
// starts conversion, waits until conversion is done, and returns result
unsigned char a2d_8bitCh(unsigned char ch)
{
	// do 10-bit conversion and return highest 8 bits
	return a2d_10bitCh(ch)>>2;			// return ADC MSB byte
}

//! Converts a supplied mV value into A/D bits
unsigned short a2dFrom_mV(unsigned short mV,unsigned short a2dmVmax,unsigned short a2dMax)
{
	unsigned long temp_u32;

	//a2dBits = (mV * a2dBits) / a2dmVmax
	temp_u32 = mV;
	temp_u32 *= a2dMax;
	temp_u32 /= a2dmVmax;
	return temp_u32;
}

//! Converts a supplied A/D value into mV
unsigned short a2dmVFrom_a2d(unsigned short a2dBits,unsigned short a2dmVmax,unsigned short a2dMax)
{
	unsigned long temp_u32;

	//a2dBits = (mV * a2dBits) / a2dmVmax
	temp_u32 = a2dBits;
	temp_u32 *= a2dmVmax;
	temp_u32 /= a2dMax;
	return temp_u32;
}



//-----------------------------------------------------------------------------
// Sets the d2a value onthe R2R port/s
//-----------------------------------------------------------------------------
void d2a_10bit(unsigned short d2a)
{
	unsigned char d2a_u8_l;
	unsigned char d2a_u8_h;
	unsigned char d2a_u8_temp;

	d2a_u8_temp = d2a & 0b0000000000000011;	//Get Low Byte (PORTD)
	d2a_u8_temp = (d2a_u8_temp << 5);
	d2a_u8_l = PORTD & 0b10011111; //clear bits 0 and 1
	d2a_u8_l |= d2a_u8_temp;		//Ready to write to PORTD

	d2a_u8_h = (d2a >> 2);		 //Get High Byte (PORTC), ready to write to PORTC

	PORTC = d2a_u8_h;
	PORTD = d2a_u8_l;
}
//-----------------------------------------------------------------------------
