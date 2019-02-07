/**
 * @file chassisLED.c
 * @author Jonn Dillon
 * @date 16/4/2018
 * @brief Handles all operations related to the onboard LED
 **/

#include "chassisLED.h"
#include <util/delay.h>

/**
 * led_flash()
 * Input:	times - The amount of times to make the LED blink
 * Returns: none
 * 
 * Makes the LED blink a set number of times.
 * The delay between the state of the LED (on/off) is 50ms. The period of the blinks is 100ms
 * 
 * Reference: ATmega Datasheet Chapter 13 (I/O-Ports)
 **/
void led_flash(uint8_t times)
{
	for(uint8_t i = 0; i < times; i++)
	{
		led_toggle();
		_delay_ms(100);
		led_toggle();
		_delay_ms(100);
	}
}

/**
 * led_toggle()
 * Input:	none
 * Returns: none
 * 
 * Switches between the two statues of the LED (on -> off or off -> on)
 * 
 * Reference: ATmega Datasheet Chapter 13 (I/O-Ports)
 **/
void led_toggle(void)
{
    PORTK ^= 0b00100000;
	// PORTK ^= 1<<PINK5;
    //return 1;
}

/**
 * led_off()
 * Input:	none
 * Returns: none
 * 
 * Turns off the LED
 * 
 * Reference: ATmega Datasheet Chapter 13 (I/O-Ports)
 **/
void led_off(void)
{
	PORTK &= ~(0b00100000);
}

void led_on(void)
{
	PORTK |= 0b00100000;
}