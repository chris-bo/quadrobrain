/*
 * ConfigReader.cpp
 *
 *  Created on: 02.04.2015
 *      Author: Jonas
 */

#include "ConfigReader.h"

ConfigReader::ConfigReader() {
	// TODO Auto-generated constructor stub

}

ConfigReader::~ConfigReader() {
	// TODO Auto-generated destructor stub
}

void ConfigReader::loadConfiguration(Status* status) {

}

void ConfigReader::saveConfiguration(Status* status) {

}

bool ConfigReader::loadVariable( uint8_t* variable, uint16_t eepromAddress, uint8_t range ) {
	// TODO: Variable laden
	// EEPROM_ADDRESS senden
	// erste 8 Adressbits senden
	// letzte 8 Adressbits senden
	for(uint8_t i = 0; i < range; i++) {
		// Variable senden
	}
	// Stop
}

bool ConfigReader::saveVariable( uint8_t* variable, uint16_t eepromAddress, uint8_t range ) {
	// TODO: Variablen speichern
	// TODO: Variable laden
	// EEPROM_ADDRESS senden
	// erste 8 Adressbits senden
	// letzte 8 Adressbits senden
	// 8 bit empfangen
	for(uint8_t i = 0; i < range; i++) {
		// Acknowlage senden
		// 8 bit empfangen
	}
	// Stop
}
