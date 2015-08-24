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
#define EEPROM_ADDRESS			0b10100000
#define EEPROM_I2C_TIMEOUT		0xFFFFF

// Pause between read / write processes in ms
#define EEPROM_WAIT 10

// Addresses
#define PID_XY_P_ADDR	                0x0000
#define PID_XY_I_ADDR	                0x0004
#define PID_XY_D_ADDR	                0x0008
#define PID_XY_GAIN_ADDR                0x000C
#define PID_XY_SCALE_ADDR               0x0010
#define PID_Z_P_ADDR                    0x0014
#define PID_Z_I_ADDR	                0x0018
#define PID_Z_D_ADDR	                0x001C
#define PID_Z_GAIN_ADDR                 0x0020
#define PID_Z_SCALE_ADDR                0x0024
#define FILTERCOEFF_XY_ADDR             0x0028
#define FILTERCOEFF_Z_ADDR              0x002C

#define QC_CONFIG_BUZZ_WARN_LOW_VOLT    0x0030
#define QC_CONFIG_BUZZ_WARN_RC_LOST     0x0031
#define QC_CONFIG_EN_LEDS               0x0032
#define QC_CONFIG_EN_MOTORS             0x0033


class ConfigReader {
public:
    ConfigReader(I2C_HandleTypeDef* i2c);
    virtual ~ConfigReader();
    void loadConfiguration(Status* status);
    void saveConfiguration(Status* status);
    void loadVariable(uint8_t* variable, uint16_t address);
    void loadVariable(uint16_t* variable, uint16_t address);
    void loadVariable(uint32_t* variable, uint16_t address);
    void loadVariable(float* variable, uint16_t address);
    void loadVariable(bool* variable, uint16_t address);
    void saveVariable(uint8_t* variable, uint16_t address, uint8_t nodelay);
    void saveVariable(uint16_t* variable, uint16_t address, uint8_t nodelay);
    void saveVariable(uint32_t* variable, uint16_t address, uint8_t nodelay);
    void saveVariable(float* variable, uint16_t address, uint8_t nodelay);
    void saveVariable(bool* variable, uint16_t address, uint8_t nodelay);

private:
    I2C_HandleTypeDef* eeprom_i2c;
    void load(uint8_t* variable, uint16_t address, uint16_t byteCount);
    void save(uint8_t* variable, uint16_t address, uint16_t byteCount,
                uint8_t nodelay);

};

#endif /* CONFIGREADER_H_ */
