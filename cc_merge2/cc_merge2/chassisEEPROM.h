/**
 * @file chassisEEPROM.h
 * @author Joshua Copeland
 * @date file creation 23/6/2018
 * @brief Functionality for reading and writing bytes to and from EEPROM.
 * 
 * 
 * Based off ATmega datasheet section 8.3.1 code examples.
 **/

#ifndef CHASSIS_EEPROM_H
#define CHASSIS_EEPROM_H

#include <stdlib.h>
#include <avr/io.h>



// Chassis setting specification (which EEPROM addresses holds which settings)
// Follows specification outlined in QUTMS_share/ChassisControl_firmware/Chassis_Settings_v1

#define EEPROM_UINT16_THROTTLE_PEDAL_RHS_0                  0x000   /**< EEPROM address for calibration value of right-hand side throttle pedal at 0%. */
#define EEPROM_UINT16_THROTTLE_PEDAL_RHS_100                0x002   /**< EEPROM address for calibration value of right-hand side throttle pedal at 100%. */
#define EEPROM_UINT16_THROTTLE_PEDAL_LHS_0                  0x004   /**< EEPROM address for calibration value of left-hand side throttle pedal at 0%. */
#define EEPROM_UINT16_THROTTLE_PEDAL_LHS_100                0x006   /**< EEPROM address for calibration value of left-hand side throttle pedal at 100%. */

#define EEPROM_UINT16_BRAKE_PEDAL_RHS_0                     0x008   /**< EEPROM address for calibration value of right-hand side brake pedal at 0%. */
#define EEPROM_UINT16_BRAKE_PEDAL_RHS_100                   0x00a   /**< EEPROM address for calibration value of right-hand side brake pedal at 100%. */
#define EEPROM_UINT16_BRAKE_PEDAL_LHS_0                     0x00c   /**< EEPROM address for calibration value of left-hand side brake pedal at 0%. */
#define EEPROM_UINT16_BRAKE_PEDAL_LHS_100                   0x00e   /**< EEPROM address for calibration value of left-hand side brake pedal at 100%. */

#define EEPROM_UINT16_STEERING_ANGLE_ZERO_POINT             0x010   /**< EEPROM address for calibration value of steering wheel's zero point. */
#define EEPROM_UINT16_STEERING_ANGLE_RADIANS_PER_VOLT       0X012   /**< EEPROM address for how many radians the steering wheel turns per volt. Actual value is x1000. */

#define EEPROM_UINT8_BRAKE_PRESSURE_FRONT_VOLTS_PER_BAR     0x020   /**< EEPROM address for volts per bar for front brake pressure. Actual value is x100. */
#define EEPROM_UINT8_BRAKE_PRESSURE_REAR_VOLTS_PER_BAR      0x021   /**< EEPROM address for volts per bar for rear brake pressure. Actual value is x100. */

#define EEPROM_UINT8_REGEN_0_POINT                          0x030   /**< EEPROM address for calibration value of 0% regenerative braking. */
#define EEPROM_UINT8_REGEN_RAMP_AMPS_PER_PERCENT            0x031   /**< EEPROM address for amps of regenerative braking per percent. */
#define EEPROM_UINT8_REGEN_100_POINT                        0x032   /**< EEPROM address for calibration value of 100% regenerative braking. */
#define EEPROM_UINT8_MAX_TORQUE_FRONT_INVERTERS             0x033   /**< EEPROM address for maximum torque of front inverters. */
#define EEPROM_UINT8_MAX_TORQUE_REAR_INVERTERS              0x034   /**< EEPROM address for maximum torque of rear inverters. */

#define EEPROM_UINT8_MAX_REGEN_FRONT_INVERTERS              0x040   /**< EEPROM address for maximum regeneration of front inverters. */
#define EEPROM_UINT8_MAX_REGEN_REAR_INVERTERS               0x041   /**< EEPROM address for maximum regeneration of rear inverters. */

#define EEPROM_UINT8_NUMBER_OF_ACCUMULATORS_FITTED          0x050   /**< EEPROM address for number of accumulators fitted. */
#define EEPROM_UINT8_NUMBER_OF_INVERTERS_FITTED             0x051   /**< EEPROM address for number of inverters fitted. */

#define EEPROM_UINT8_T64_INITIAL_TABLE                      0x100   /**< EEPROM address for start of initial table. Table address range is 0x100-0x13f. */
#define EEPROM_UINT8_T64_SUSTAINED_TABLE                    0x200   /**< EEPROM address for start of sustained table. Table address range is 0x200-0x33f. */
#define EEPROM_UINT8_T64_FRONT_REAR_BIAS                    0x300   /**< EEPROM address for start of front/read bias table. Table address range is 0x300-0x33f. */



// EEPROM Reading

/**
 * @brief Reads a character from an EEPROM address.
 * 
 * 
 * Example Code: unsigned char Character = eeprom_read(50);
 * 
 * 
 * @param address The address of EEPROM to read from.
 * @return unsigned char The byte written to the EEPROM data register.
 */
unsigned char eeprom_read_char(uint16_t address);

/**
 * @brief Reads an 8-bit integer from an EEPROM address.
 * 
 * 
 * Example Code: unsigned char Character = eeprom_read(50);
 * 
 * 
 * @param address The address of EEPROM to read from.
 * @return unsigned char The byte written to the EEPROM data register.
 */
uint8_t eeprom_read_int8(uint16_t address);

/**
 * @brief Reads a 16-bit integer from two addresses of EEPROM.
 * 
 * 
 * Example Code: unsigned char Character = eeprom_read(50);
 * 
 * 
 * @param address The address of EEPROM to read from.
 * @return unsigned char The byte written to the EEPROM data register.
 */
uint16_t eeprom_read_int16(uint16_t address);



// EEPROM Writing

/**
 * @brief Writes a character to an address of EEPROM.
 * 
 * 
 * Example Code: eeprom_write_byte(50,'A');
 * 
 * 
 * @param address The address of EEPROM to write to.
 * @param data The character to write to EEPROM.
 * @return void
 */
void eeprom_write_char(uint16_t address, unsigned char data);

/**
 * @brief Writes a 8-bit integer to an address of EEPROM.
 * 
 * 
 * Example Code: eeprom_write_byte(50,255);
 * 
 * 
 * @param address The address of EEPROM to write to.
 * @param data The 8-bit integer to write to EEPROM.
 * @return void
 */
void eeprom_write_int8(uint16_t address, uint8_t data);

/**
 * @brief Writes a 16-bit integer across 2 EEPROM addresses.
 * 
 * 
 * Example Code: eeprom_write_int16(50,65535);
 * 
 * 
 * @param address The address of EEPROM to write to.
 * @param data The 16-bit integer to write to EEPROM.
 * @return void
 */
void eeprom_write_int16(uint16_t start_address, uint16_t data);

#endif // CHASSIS_EEPROM_H