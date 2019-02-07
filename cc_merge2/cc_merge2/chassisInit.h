#ifndef CHASSIS_INIT_H
#define CHASSIS_INIT_H

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>


#include "chassisLED.h"

void io_init();
void firmware_init();
void timer_init();

#endif // CHASSIS_INIT_H