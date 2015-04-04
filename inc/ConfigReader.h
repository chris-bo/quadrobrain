/*
 * ConfigReader.h
 *
 *  Created on: 02.04.2015
 *      Author: Jonas
 */

#ifndef CONFIGREADER_H_
#define CONFIGREADER_H_

#include "Status.h"
#include "stm32f3xx_hal.h"

// Definitions
#define EEPROM_ADDRESS		0b10100000
#define EEPROM_I2C_TIMEOUT	0xFFFFF


class ConfigReader {
public:
	ConfigReader( I2C_HandleTypeDef* i2c );
	virtual ~ConfigReader();
	void loadConfiguration( Status* status );
	void saveConfiguration( Status* status );

private:
	I2C_HandleTypeDef* eeprom_i2c;
	void loadVariable( uint8_t* variable, uint16_t address, uint16_t byteCount );
	void saveVariable( uint8_t* variable, uint16_t address, uint16_t byteCount );

};

#endif /* CONFIGREADER_H_ */
