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
}

/*
 * Loads the quadrocopter configuration from eeprom into status
 */
void ConfigReader::loadConfiguration(Status* status) {
    /* reinit i2c if communication is crashed after reset */

    HAL_I2C_Init(eeprom_i2c);

    HAL_Delay(2);

    loadVariable(&status->pXY, P_XY_ADDR);
    loadVariable(&status->iXY, I_XY_ADDR);
    loadVariable(&status->dXY, D_XY_ADDR);
    loadVariable(&status->pZ, P_Z_ADDR);
    loadVariable(&status->iZ, I_Z_ADDR);
    loadVariable(&status->dZ, D_Z_ADDR);
    loadVariable(&status->filterCoefficientXY, FILTERCOEFF_XY_ADDR);
    loadVariable(&status->filterCoefficientZ, FILTERCOEFF_Z_ADDR);

    if (HAL_I2C_GetError(eeprom_i2c) == HAL_I2C_ERROR_NONE) {
        SET_FLAG((status->globalFlags), EEPROM_OK_FLAG);
    } else {
        /*reload hardcoded values*/
        status->restoreConfig();
    }
}

/*
 * Saves the quadrocopter configuration from status into EEPROM
 */
void ConfigReader::saveConfiguration(Status* status) {


    saveVariable(&status->pXY, P_XY_ADDR, 0);
    saveVariable(&status->iXY, I_XY_ADDR, 0);
    saveVariable(&status->dXY, D_XY_ADDR, 0);
    saveVariable(&status->pZ, P_Z_ADDR, 0);
    saveVariable(&status->iZ, I_Z_ADDR, 0);
    saveVariable(&status->dZ, D_Z_ADDR, 0);
    saveVariable(&status->filterCoefficientXY, FILTERCOEFF_XY_ADDR, 0);
    saveVariable(&status->filterCoefficientZ, FILTERCOEFF_Z_ADDR, 0);
}

/*
 * Loads a single uint8_t from EEPROM
 */
void ConfigReader::loadVariable(uint8_t* variable, uint16_t address) {
    load(variable, address, 1);
}

/*
 * Loads a single uint16_t from EEPROM
 */
void ConfigReader::loadVariable(uint16_t* variable, uint16_t address) {
    uint8_t* tmp = (uint8_t*) variable;
    load(tmp, address, 2);
}

/*
 * Loads a single uint32_t from EEPROM
 */
void ConfigReader::loadVariable(uint32_t* variable, uint16_t address) {
    uint8_t* tmp = (uint8_t*) variable;
    load(tmp, address, 4);
}

/*
 * Loads a single float from EEPROM
 */
void ConfigReader::loadVariable(float* variable, uint16_t address) {
    uint8_t* tmp = (uint8_t*) variable;
    load(tmp, address, 4);
}

/*
 * Loads a single bool from EEPROM
 */
void ConfigReader::loadVariable(bool* variable, uint16_t address) {
    uint8_t* tmp = (uint8_t*) variable;
    load(tmp, address, 1);
}

/*
 * Saves a single uint8_t to EEPROM
 */
void ConfigReader::saveVariable(uint8_t* variable, uint16_t address,
            uint8_t nodelay) {
    save(variable, address, 1, nodelay);
}

/*
 * Saves a single uint16_t to EEPROM
 */
void ConfigReader::saveVariable(uint16_t* variable, uint16_t address,
            uint8_t nodelay) {
    uint8_t* tmp = (uint8_t*) variable;
    save(tmp, address, 2, nodelay);
}

/*
 * Saves a single uint32_t to EEPROM
 */
void ConfigReader::saveVariable(uint32_t* variable, uint16_t address,
            uint8_t nodelay) {
    uint8_t* tmp = (uint8_t*) variable;
    save(tmp, address, 4, nodelay);
}

/*
 * Saves a single float to EEPROM
 */
void ConfigReader::saveVariable(float* variable, uint16_t address, uint8_t nodelay) {
    uint8_t* tmp = (uint8_t*) variable;
    save(tmp, address, 4, nodelay);
}

/*
 * Saves a single bool to EEPROM
 */
void ConfigReader::saveVariable(bool* variable, uint16_t address, uint8_t nodelay) {
    uint8_t* tmp = (uint8_t*) variable;
    save(tmp, address, 1, nodelay);
}

/*
 * Loads byteCount bytes from address into variable
 */
void ConfigReader::load(uint8_t* variable, uint16_t address, uint16_t byteCount) {

    // Bytes aus I2C-EEPROM holen
    HAL_I2C_Mem_Read(eeprom_i2c, EEPROM_ADDRESS, address, I2C_MEMADD_SIZE_16BIT,
                variable, byteCount, EEPROM_I2C_TIMEOUT);

}

/*
 * Writes byteCount bytes from variable into address
 */
void ConfigReader::save(uint8_t* variable, uint16_t address, uint16_t byteCount,
            uint8_t nodelay) {
    // Bytes in I2C-EEPROM schreiben
    HAL_I2C_Mem_Write(eeprom_i2c, EEPROM_ADDRESS, address,
    I2C_MEMADD_SIZE_16BIT, variable, byteCount, EEPROM_I2C_TIMEOUT);
    if (nodelay == 0) {
        HAL_Delay(EEPROM_WAIT);
    }
}
