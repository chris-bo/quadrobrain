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
#define EEPROM_ADDRESS	0b10100000
#define EEPROM_I2C_TIMEOUT		0xFFFFF


class ConfigReader {
public:
	ConfigReader( I2C_HandleTypeDef* i2c );
	virtual ~ConfigReader();
	void loadConfiguration( Status* status );
	void saveConfiguration( Status* status );
	void loadVariable( uint8_t* variable, uint16_t address );
	void loadVariable( uint16_t* variable, uint16_t address );
	void loadVariable( uint32_t* variable, uint16_t address );
	void loadVariable( float* variable, uint16_t address );
	void loadVariable( bool* variable, uint16_t address );

	void saveVariable( uint8_t* variable, uint16_t address );
	void saveVariable( uint16_t* variable, uint16_t address );
	void saveVariable( uint32_t* variable, uint16_t address );
	void saveVariable( float* variable, uint16_t address );
	void saveVariable( bool* variable, uint16_t address );
private:
	I2C_HandleTypeDef* eeprom_i2c;
	void load( uint8_t* variable, uint16_t address, uint16_t byteCount );
	void save( uint8_t* variable, uint16_t address, uint16_t byteCount );

};

#endif /* CONFIGREADER_H_ */
