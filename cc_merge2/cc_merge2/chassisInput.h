/**
 * @file chassisInput.h
 * @author Jonn Dillon
 * @date 16/4/2018
 * @brief Defines and outlines all variables and definitions related to everything related to input within the car.
 **/

/**
 * Confirmed and tested values from the ADC when working properly
 * 
 * Throttle Primary     CH1     ADC5    Right   MIN 325     MAX 497
 * Throttle Secondary   CH2     ADC2    Left    MIN 314     MAX 498
 *                                              DIF 11      DIF 1
 * 
 * Brake    Primary     CH1     ADC10   Right   MIN 426     MAX 665
 * Brake    Secondary   CH2     ADC8    Left    MIN 455     MAX 686
 *                                              DIF 29      DIF 21
 *
 *Steering	Single		CH1		ADC4			TBA			TBA
 */

#ifndef CHASSIS_INPUT_H
#define CHASSIS_INPUT_H

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include "chassisError.h"

#define INPUT_PEDAL_BRAKE_CH1 (10)                      /**< Brake Pedal POT Ch1 on ADC */
#define INPUT_PEDAL_BRAKE_CH2 (8)                       /**< Brake Pedal POT Ch2 on ADC */
#define INPUT_PEDAL_THROTTLE_CH1 (5)                    /**< Throttle Pedal POT Ch1 on ADC */ // Channel is confirmed
#define INPUT_PEDAL_THROTTLE_CH2 (2)                    /**< Throttle Pedal POT Ch2 on ADC */ // Channel is confirmed
#define INPUT_PRESSURE_BRAKE_FRONT (11)                 /**< Brake pressure front POT Ch on ADC (AN6) */
#define INPUT_PRESSURE_BRAKE_BACK  (7)                  /**< Brake pressure back POT Ch on ADC (AN7) */

#define INPUT_STEERING_ANGLE_CH (4)                     /**< Steering Angle POT CH on ADC (AN8)*/

#define INPUT_RADIATOR_RIGHT_CH (0)	                    /**< Radiator ADC CH Right (AN5) */
#define INPUT_RADIATOR_LEFT_CH (12)	                    /**< Radiator ADC CH Left (AN10) */

#define ADC_SAMPLES	(8)                                 /**< Samples required for ADC */

extern uint8_t INPUT_steeringAngle;
extern uint8_t INPUT_accelerationPedal;
extern uint8_t INPUT_brakePedal;
extern uint8_t INPUT_brakePressureFront;
extern uint8_t INPUT_brakePressureBack;




uint8_t INPUT_scaleInput(uint16_t * value, uint16_t max, uint16_t min);

/**
 * @brief INPUT_get_accelPedal
 * 
 * Provides wrapper with error calling on INPUT_read_accelPedal
 * 
 * @param val Given value to populate
 * @return uint8_t State of read
 */
uint8_t INPUT_get_accelPedal(uint8_t * val);

/**
 * @brief INPUT_get_brakePedal
 * 
 * Provides wrapper with error calling on INPUT_read_brakePedal
 * 
 * @param val Given value to populate
 * @return uint8_t State of read
 */
uint8_t INPUT_get_brakePedal(uint8_t * val);

/**
 * @brief INPUT_get_steeringWheel
 * 
 * Provides wrapper with error calling on INPUT_read_steeringWheel
 * 
 * @param val Given value to populate
 * @return uint8_t State of read
 */
uint8_t INPUT_get_steeringWheel(uint8_t * val);

/**
 * @brief INPUT_get_brakePressureFront
 * 
 * Provides wrapper with error calling on INPUT_read_brakePressureFront
 * 
 * @param val Given value to populate
 * @return uint8_t State of read
 */
uint8_t INPUT_get_brakePressureFront(uint16_t * val);

/**
 * @brief INPUT_get_brakePressureBack
 * 
 * Provides wrapper with error calling on INPUT_read_brakePressureBack
 * 
 * @param val Given value to populate
 * @return uint8_t State of read
 */
uint8_t INPUT_get_brakePressureBack(uint16_t * val);

/**
 * @brief INPUT_read_accelPedal
 * 
 * @param throttle Given uint16_t pointer to populate
 * @return uint8_t 0 if the value is out of spec
 */
uint8_t INPUT_read_accelPedal(uint16_t * throttle);

/**
 * @brief INPUT_read_brakePedal
 * 
 * @param brake Given uint16_t pointer to populate
 * @return uint8_t 0 if the value is out of spec
 */
uint8_t INPUT_read_brakePedal(uint16_t * brake);

/**
 * @brief INPUT_read_steeringWheel
 * 
 * @param brake Given uint16_t pointer to populate
 * @return uint8_t 0 if the value is out of spec
 */
uint8_t INPUT_read_steeringWheel(uint16_t * brake);

/**
 * @brief INPUT_read_brakePressureFront
 * 
 * @param fntPressure Given uint16_t pointer to populate
 * @return uint8_t 0 if the value is out of spec
 */
uint8_t INPUT_read_brakePressureFront(uint16_t * fntPressure);

/**
 * @brief INPUT_read_brakePressureBack
 * 
 * @param bkPressure Given uint16_t pointer to populate
 * @return uint8_t  0 if the value is out of spec
 */
uint8_t INPUT_read_brakePressureBack(uint16_t * bkPressure);

#endif // CHASSIS_INPUT_H