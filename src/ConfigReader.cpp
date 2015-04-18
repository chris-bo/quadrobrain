/*
 * ConfigReader.cpp
 *
 *  Created on: 02.04.2015
 *      Author: Jonas
 */

#include "ConfigReader.h"

ConfigReader::ConfigReader(I2C_HandleTypeDef* i2c) {
	eeprom_i2c = i2c;
}

ConfigReader::~ConfigReader() {
	// TODO Auto-generated destructor stub
}

/*
 * Loads the quadrocopter configuration from eeprom into status
 */
void ConfigReader::loadConfiguration(Status* status) {
	//TODO: Weitere hinzufügen, wenn vorhanden
	loadVariable( &status->pXY, P_XY_ADDR );
	loadVariable( &status->iXY, I_XY_ADDR );
	loadVariable( &status->dXY, D_XY_ADDR );
	loadVariable( &status->pZ, P_Z_ADDR );
	loadVariable( &status->iZ, I_Z_ADDR );
	loadVariable( &status->dZ, D_Z_ADDR );
}

/*
 * Saves the quadrocopter configuration from status into eeprom
 */
void ConfigReader::saveConfiguration(Status* status) {
	//TODO: Weitere hinzufügen, wenn vorhanden
	saveVariable( &status->pXY, P_XY_ADDR );
	saveVariable( &status->iXY, I_XY_ADDR );
	saveVariable( &status->dXY, D_XY_ADDR );
	saveVariable( &status->pZ, P_Z_ADDR );
	saveVariable( &status->iZ, I_Z_ADDR );
	saveVariable( &status->dZ, D_Z_ADDR );
}

/*
 * Loads a single uint8_t from eeprom
 */
void ConfigReader::loadVariable(uint8_t* variable, uint16_t address) {
	load(variable, address, 1);
}

/*
 * Loads a single uint16_t from eeprom
 */
void ConfigReader::loadVariable(uint16_t* variable, uint16_t address) {
	uint8_t* tmp = (uint8_t*) variable;
	load(tmp, address, 2);
}

/*
 * Loads a single uint32_t from eeprom
 */
void ConfigReader::loadVariable(uint32_t* variable, uint16_t address) {
	uint8_t* tmp = (uint8_t*) variable;
	load(tmp, address, 4);
}

/*
 * Loads a single float from eeprom
 */
void ConfigReader::loadVariable(float* variable, uint16_t address) {
	uint8_t* tmp = (uint8_t*) variable;
	load(tmp, address, 4);
}

/*
 * Loads a single bool from eeprom
 */
void ConfigReader::loadVariable(bool* variable, uint16_t address) {
	uint8_t* tmp = (uint8_t*) variable;
	load(tmp, address, 1);
}

/*
 * Saves a single uint8_t to eeprom
 */
void ConfigReader::saveVariable(uint8_t* variable, uint16_t address) {
	save(variable, address, 1);
}

/*
 * Saves a single uint16_t to eeprom
 */
void ConfigReader::saveVariable(uint16_t* variable, uint16_t address) {
	uint8_t* tmp = (uint8_t*) variable;
	save(tmp, address, 2);
}

/*
 * Saves a single uint32_t to eeprom
 */
void ConfigReader::saveVariable(uint32_t* variable, uint16_t address) {
	uint8_t* tmp = (uint8_t*) variable;
	save(tmp, address, 4);
}

/*
 * Saves a single float to eeprom
 */
void ConfigReader::saveVariable(float* variable, uint16_t address) {
	uint8_t* tmp = (uint8_t*) variable;
	save(tmp, address, 4);
}

/*
 * Saves a single bool to eeprom
 */
void ConfigReader::saveVariable(bool* variable, uint16_t address) {
	uint8_t* tmp = (uint8_t*) variable;
	save(tmp, address, 1);
}

/*
 * Loads byteCount bytes from address into variable
 */
void ConfigReader::load(uint8_t* variable, uint16_t address,
		uint16_t byteCount) {
	// Bytes aus I2C-EEPROM holen
	HAL_I2C_Mem_Read(eeprom_i2c, EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_16BIT,
			variable, byteCount, EEPROM_I2C_TIMEOUT);
}

/*
 * Writes byteCount bytes from variable into address
 */
void ConfigReader::save(uint8_t* variable, uint16_t address,
		uint16_t byteCount) {
	// Bytes in I2C-EEPROM schreiben
	HAL_I2C_Mem_Write(eeprom_i2c, EEPROM_ADDRESS, address,
			I2C_MEMADD_SIZE_16BIT, variable, byteCount, EEPROM_I2C_TIMEOUT);
}
