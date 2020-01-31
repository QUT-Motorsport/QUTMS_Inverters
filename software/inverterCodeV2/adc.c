/*****************************************************************************
* @file    firmware/QUTMS_SteeringWheel_Firmware/adc.c
* @author  Zoe Goodward
* @version V1.0.0
* @date    2/08/2019 1:12:41 PM
* @brief   Analogue to digital conversion
*****************************************************************************/

/* INCLUDES */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "adc.h"

/*============================================================================
Function:   adc_init()
------------------------------------------------------------------------------
Purpose :   initializes the analogue to digital conversion by first selecting
			the voltage reference and then setting the ADC Enable bit, ADEN
			in ADCSRA
Input   :   none
Returns :   void
Notes   :
============================================================================*/
void adc_init() {
    ADCSRA = 0; // Clear
    // Right adjust result
    ADMUX &= ~(1 << ADLAR); // Clear
    ADMUX |= (1 << REFS1) | (1 << REFS0);
    ADCSRB = (1 << AREFEN);
    ADCSRA |= (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

void adc_set_prescalar(uint8_t prescalar) {
    //ADCSRA &= ADC_PRESCALAR_MASK;
    ADCSRA |= prescalar;
}

/*============================================================================
Function:   adc_read()
------------------------------------------------------------------------------
Purpose :   reads an analogue input voltage and converts it to a 10-bit digital
			value through successive approximation
Input   :   uint8_t channel - selected analogue input channel
Returns :   result - pass the 10 bit ADC number to requesting function
Notes   :
============================================================================*/
uint16_t adc_read(uint8_t adc_channel) {
    uint16_t result = 0;
    adc_channel = (ADMUX & 0b11100000) | (adc_channel & 0b00011111);
    ADMUX = adc_channel;
    ADCSRA |= (1 << ADSC);
    while(!(ADCSRA & (1 << ADIF)));
    result = ADCL;
    result |= ((0b00000011 & ADCH) << 0b00001000);
    ADCSRA |= (1 << ADIF);
    return result;
}
