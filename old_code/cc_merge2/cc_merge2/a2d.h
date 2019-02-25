/*! \file a2d.h \brief Analog-to-Digital converter function library. */
//*****************************************************************************
//
// File Name	: 'a2d.h'
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
#ifndef A2D_H
#define A2D_H

// defines

// A2D clock prescaler select
//		*selects how much the CPU clock frequency is divided
//		to create the A2D clock frequency
//		*lower division ratios make conversion go faster
//		*higher division ratios make conversions more accurate
#define ADC_PRESCALE_DIV2		0b00000000	///< 0x01,0x00 -> CPU clk/2
//#define ADC_PRESCALE_DIV2		0b00000001	///< 0x01,0x00 -> CPU clk/2
#define ADC_PRESCALE_DIV4		0b00000010	///< -> CPU clk/4
#define ADC_PRESCALE_DIV8		0b00000011	///< -> CPU clk/8
#define ADC_PRESCALE_DIV16		0b00000100	///< -> CPU clk/16
#define ADC_PRESCALE_DIV32		0b00000101	///< -> CPU clk/32
#define ADC_PRESCALE_DIV64		0b00000110	///< -> CPU clk/64
#define ADC_PRESCALE_DIV128		0b00000111	///< -> CPU clk/128
// default value
#define ADC_PRESCALE			ADC_PRESCALE_DIV64
// do not change the mask value
#define ADC_PRESCALE_MASK		0b11111000

// A2D voltage reference select
//		*this determines what is used as the
//		full-scale voltage point for A2D conversions
#define ADC_REFERENCE_AREF		0b00000000	///< -> AREF pin, internal VREF turned off
#define ADC_REFERENCE_AVCC		0b00000001	///< -> AVCC pin, internal VREF turned off
#define ADC_REFERENCE_11V		0b00000010	///< -> Internal 1.1V VREF
#define ADC_REFERENCE_256V		0b00000011	///< -> Internal 2.56V VREF
// default value
#define ADC_REFERENCE			ADC_REFERENCE_AVCC
// do not change the mask value
#define ADC_REFERENCE_MASK		0b11111100

// bit mask for A2D channel multiplexer
#define ADC_MUX_MASK			0b11100000

// channel defines (for reference and use in code)
#define ADC_CH_ADC0				0b00000000
#define ADC_CH_ADC1				0b00000001
#define ADC_CH_ADC2				0b00000010
#define ADC_CH_ADC3				0b00000011
#define ADC_CH_ADC4				0b00000100
#define ADC_CH_ADC5				0b00000101
#define ADC_CH_ADC6				0b00000110
#define ADC_CH_ADC7				0b00000100
#define ADC_CH_11V				0b00011110	///< 1.1V voltage reference
#define ADC_CH_AGND				0b00011111	///< AGND

// differential
#define ADC_CH_0_0_DIFF10X		0b00001000
#define ADC_CH_1_0_DIFF10X		0b00001001
#define ADC_CH_0_0_DIFF200X		0b00001010
#define ADC_CH_1_0_DIFF200X		0b00001011
#define ADC_CH_2_2_DIFF10X		0b00001100
#define ADC_CH_3_2_DIFF10X		0b00001101
#define ADC_CH_2_2_DIFF200X		0b00001110
#define ADC_CH_3_2_DIFF200X		0b00001111
#define ADC_CH_0_1_DIFF1X		0b00010000
#define ADC_CH_1_1_DIFF1X		0b00010001
#define ADC_CH_2_1_DIFF1X		0b00010010
#define ADC_CH_3_1_DIFF1X		0b00010011
#define ADC_CH_4_1_DIFF1X		0b00010100
#define ADC_CH_5_1_DIFF1X		0b00010101
#define ADC_CH_6_1_DIFF1X		0b00010110
#define ADC_CH_7_1_DIFF1X		0b00010111

#define ADC_CH_0_2_DIFF1X		0b00011000
#define ADC_CH_1_2_DIFF1X		0b00011001
#define ADC_CH_2_2_DIFF1X		0b00011010
#define ADC_CH_3_2_DIFF1X		0b00011011
#define ADC_CH_4_2_DIFF1X		0b00011100
#define ADC_CH_5_2_DIFF1X		0b00011110

// function prototypes

//! Initializes the A/D converter.
/// Turns ADC on and prepares it for use.
void a2dInit(unsigned char prescale, unsigned char ref);

//! Turn On A/D converter
void a2dOn(void);

//! Turn Off A/D converter
void a2dOff(void);

//! Sets the division ratio of the A/D converter clock.
void a2dSetPrescaler(unsigned char prescale);

//! Configures which voltage reference the A/D converter uses.
void a2dSetReference(unsigned char ref);

//! Sets the a2d input channel
void a2dSetChannel(unsigned char ch);

//! Reads a sample from the current A/D channel, 10bit result
unsigned short a2d_10bit();

//! Reads a sample from the current A/D channel, 8bit result
unsigned char a2d_8bit();

//! Reads a sample from the A/D channel# ch, 10bit result
unsigned short a2d_10bitCh(unsigned char ch);

//! Reads a sample from the A/D channel# ch, 8bit result
unsigned char a2d_8bitCh(unsigned char ch);

//! Converts a supplied mV value into A/D bits
unsigned short a2dFrom_mV(unsigned short mV,unsigned short a2dmVmax,unsigned short a2dMax);

//! Converts a supplied A/D value into mV
unsigned short a2dmVFrom_a2d(unsigned short a2dBits,unsigned short a2dmVmax,unsigned short a2dMax);

void d2a_10bit(unsigned short d2a); //sets the d2a value onthe R2R port/s

#endif