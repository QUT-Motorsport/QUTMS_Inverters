/**
 * @file chassisEEPROM.c
 * @author Joshua Copeland
 * @date 23/6/2018
 * @brief Contains functions for reading and writing data to and from the EEPROM storage
 **/

#include "chassisEEPROM.h";



// Function for reading a character from EEPROM, see header file for more info.
unsigned char eeprom_read_char(uint16_t address)
{
    // Wait for completion of previous write (wait for write enable bit to be 0).
    while(EECR & (1 << EEPE));

    // Set address register.
    EEAR = address;

    // Start eeprom read by setting read enable bit to 1.
    EECR |= (1 << EERE);

    // Return data from data register.
    return EEDR;
}

// Function for reading an 8-bit integer from EEPROM, see header file for more info.
uint8_t eeprom_read_int8(uint16_t address)
{
    // Wait for completion of previous write (wait for write enable bit to be 0).
    while(EECR & (1 << EEPE));

    // Set address register.
    EEAR = address;

    // Start eeprom read by setting read enable bit to 1.
    EECR |= (1 << EERE);

    // Return data from data register.
    return EEDR;
}

// Function for reading a 16-bit integer from EEPROM, see header file for more info.
uint16_t eeprom_read_int16(uint16_t address)
{
    // Create array to hold read values.
    uint8_t data_segments[2];
    
    // Read the 16-bit value across two addresses as two 8-bit values.
    for (int i = 0; i < 2; i++) {
        // Wait for completion of previous write (wait for write enable bit to be 0).
        while(EECR & (1 << EEPE));

        // Set address register.
        EEAR = address + i;

        // Start eeprom read by setting read enable bit to 1.
        EECR |= (1 << EERE);

        // Save data from data register in data segment array.
        data_segments[i] = EEDR;
    }

    // Assemble the two 8-bit values into a 16-bit value and return that value.
    uint16_t data = 0x0000 | (data_segments[0] << 8);
    data = data | data_segments[1];
    return data;
}



// Function for writing a character to EEPROM, see header file for more info.
void eeprom_write_char(uint16_t address, unsigned char data) {
    // Wait for completion of previous write (wait for write enable bit to be 0).
    while(EECR & (1 << EEPE));

    // Set address and data registers.
    EEAR = address;
    EEDR = data;

    // Set master write enable bit to 1.
    EECR |= (1 << EEMPE);

    // Start EEPROM write by setting write enable bit to 1.
    EECR |= (1 << EEPE);

    // Return integer value 1 upon successful completion.
    return 1;
}

// Function for writing an 8-bit integer to EEPROM, see header file for more info.
void eeprom_write_int8(uint16_t address, uint8_t data) {
    // Wait for completion of previous write (wait for write enable bit to be 0).
    while(EECR & (1 << EEPE));

    // Set address and data registers.
    EEAR = address;
    EEDR = data;

    // Set master write enable bit to 1.
    EECR |= (1 << EEMPE);

    // Start EEPROM write by setting write enable bit to 1.
    EECR |= (1 << EEPE);

    // Return integer value 1 upon successful completion.
    return 1;
}

// Function for writing a 16-bit integer to EEPROM, see header file for more info.
void eeprom_write_int16(uint16_t address, uint16_t data) {
    // Split 16-bit integer into two 8-bit values.
    uint8_t data_segments[2];
    data_segments[0] = (data >> 8) | 0x00;
    data_segments[1] = data | 0x00;
    
    // Write the 16-bit value across two addresses as two 8-bit values.
    for (int i = 0; i < 2; i++) {
        // Wait for completion of previous write (wait for write enable bit to be 0).
        while(EECR & (1 << EEPE));

        // Set address and data registers.
        EEAR = address + i;
        EEDR = data_segments[i];

        // Set master write enable bit to 1.
        EECR |= (1 << EEMPE);

        // Start EEPROM write by setting write enable bit to 1.
        EECR |= (1 << EEPE);
    }

    // Return integer value 1 upon successful completion.
    return 1;
}