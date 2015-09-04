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

    loadVariable(&status->pidSettigsAngleXY.p, PID_XY_P_ADDR);
    loadVariable(&status->pidSettigsAngleXY.i, PID_XY_I_ADDR);
    loadVariable(&status->pidSettigsAngleXY.d, PID_XY_D_ADDR);
    loadVariable(&status->pidSettigsAngleXY.gain, PID_XY_GAIN_ADDR);
    loadVariable(&status->pidSettigsAngleXY.scaleSetPoint, PID_XY_SCALE_ADDR);
    loadVariable(&status->pidSettigsRotationZ.p, PID_Z_P_ADDR);
    loadVariable(&status->pidSettigsRotationZ.i, PID_Z_I_ADDR);
    loadVariable(&status->pidSettigsRotationZ.d, PID_Z_D_ADDR);
    loadVariable(&status->pidSettigsRotationZ.gain, PID_Z_GAIN_ADDR);
    loadVariable(&status->pidSettigsRotationZ.scaleSetPoint, PID_Z_SCALE_ADDR);
    loadVariable(&status->filterCoefficientXY, FILTERCOEFF_XY_ADDR);
    loadVariable(&status->filterCoefficientZ, FILTERCOEFF_Z_ADDR);

    //* load qc Configs */
  // loadVariable(&status->qcSettings.enableBuzzerWarningLowVoltage, QC_CONFIG_BUZZ_WARN_LOW_VOLT);
  //  loadVariable(&status->qcSettings.enableBuzzerWarningRCLost, QC_CONFIG_BUZZ_WARN_RC_LOST);
  //  loadVariable(&status->qcSettings.enableFlightLeds, QC_CONFIG_EN_LEDS);
  //  loadVariable(&status->qcSettings.enableMotors, QC_CONFIG_EN_MOTORS);


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

    saveVariable(&status->pidSettigsAngleXY.p, PID_XY_P_ADDR, 0);
    saveVariable(&status->pidSettigsAngleXY.i, PID_XY_I_ADDR, 0);
    saveVariable(&status->pidSettigsAngleXY.d, PID_XY_D_ADDR, 0);
    saveVariable(&status->pidSettigsAngleXY.gain, PID_XY_GAIN_ADDR, 0);
    saveVariable(&status->pidSettigsAngleXY.scaleSetPoint, PID_XY_SCALE_ADDR, 0);
    saveVariable(&status->pidSettigsRotationZ.p, PID_Z_P_ADDR, 0);
    saveVariable(&status->pidSettigsRotationZ.i, PID_Z_I_ADDR, 0);
    saveVariable(&status->pidSettigsRotationZ.d, PID_Z_D_ADDR, 0);
    saveVariable(&status->pidSettigsRotationZ.gain, PID_Z_GAIN_ADDR, 0);
    saveVariable(&status->pidSettigsRotationZ.scaleSetPoint, PID_Z_SCALE_ADDR, 0);
    saveVariable(&status->filterCoefficientXY, FILTERCOEFF_XY_ADDR, 0);
    saveVariable(&status->filterCoefficientZ, FILTERCOEFF_Z_ADDR, 0);


    //* save qc Configs */
    saveVariable(&status->qcSettings.enableBuzzerWarningLowVoltage, QC_CONFIG_BUZZ_WARN_LOW_VOLT, 0);
    saveVariable(&status->qcSettings.enableBuzzerWarningRCLost, QC_CONFIG_BUZZ_WARN_RC_LOST, 0);
    saveVariable(&status->qcSettings.enableFlightLeds, QC_CONFIG_EN_LEDS, 0);
    saveVariable(&status->qcSettings.enableMotors, QC_CONFIG_EN_MOTORS, 0);

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
