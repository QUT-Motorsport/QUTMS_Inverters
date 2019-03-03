/**
 * @file chassisLED.h
 * @author Jonn Dillon
 * @date 08/07/2018
 * @brief Defines and outlines all variables and definitions related to the onboard LED
 **/

#ifndef CHASSIS_LED_H
#define CHASSIS_LED_H

#include <stdlib.h>
#include <avr/io.h>

/**
 * @brief Makes the LED blink a set number of times.
 * 
 * The delay between the state of the LED (on/off) is 50ms. The period of the blinks is 100ms
 * 
 * Reference: ATmega Datasheet Chapter 13 (I/O-Ports)
 * 
 * @param times The amount of times to make the LED blink
 */
void led_flash(unsigned char times);

/**
 * @brief  Switches between the two statues of the LED (on -> off or off -> on)
 * 
 * Reference: ATmega Datasheet Chapter 13 (I/O-Ports)
 * 
 */
void led_toggle(void);

/**
 * @brief Turns off the LED
 * 
 * Reference: ATmega Datasheet Chapter 13 (I/O-Ports)
 * 
 */
void led_off(void);

/**
 * @brief Turns on the LED
 * 
 * Reference: ATmega Datasheet Chapter 13 (I/O-Ports)
 * 
 */
void led_on(void);


#endif // CHASSIS_LED_H