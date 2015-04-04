/*
 * ConfigReader.cpp
 *
 *  Created on: 02.04.2015
 *      Author: Jonas
 */

#include "ConfigReader.h"

ConfigReader::ConfigReader( I2C_HandleTypeDef* i2c ) {
	eeprom_i2c = i2c;
}

ConfigReader::~ConfigReader() {
	// TODO Auto-generated destructor stub
}

void ConfigReader::loadConfiguration( Status* status ) {

}

void ConfigReader::saveConfiguration( Status* status ) {

}

/*
 * Loads byteCount bytes into variable from address
 */
void ConfigReader::loadVariable( uint8_t* variable, uint16_t address, uint16_t byteCount ) {
	// Bytes aus I2C-EEPROM holen
	HAL_I2C_Mem_Read( eeprom_i2c, EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, variable, byteCount, EEPROM_I2C_TIMEOUT );
}

/*
 * Writes byteCount bytes from variable into address
 */
void ConfigReader::saveVariable( uint8_t* variable, uint16_t address, uint16_t byteCount ) {
	// Bytes in I2C-EEPROM schreiben
	HAL_I2C_Mem_Write( eeprom_i2c, EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_8BIT, variable, byteCount, EEPROM_I2C_TIMEOUT );
}
