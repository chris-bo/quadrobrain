/*
 * ConfigReader.h
 *
 *  Created on: 02.04.2015
 *      Author: Jonas
 */

#ifndef CONFIGREADER_H_
#define CONFIGREADER_H_

#include "Status.h"

// Definitions
#define EEPROM_ADDRESS_WRITE 	0b10100000
#define EEPROM_ADDRESS_READ 	0b10100001


class ConfigReader {
public:
	ConfigReader();
	virtual ~ConfigReader();
	void loadConfiguration( Status* status );
	void saveConfiguration( Status* status );

private:
	bool loadVariable( uint8_t* variable, uint16_t eepromAddress, uint8_t range );
	bool saveVariable( uint8_t* variable, uint16_t eepromAddress, uint8_t range );

};

#endif /* CONFIGREADER_H_ */
