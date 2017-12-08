/*
 * BMP180.h
 *
 *  Created on: Apr 28, 2015
 *      Author: bohni
 */

#ifndef BMP180_H_
#define BMP180_H_

#include <core/Task.h>
#include "stm32f3xx_hal.h"
#include "i2c.h"
#include <math.h>

/**I2C2 GPIO Configuration
 * PA9     ------> I2C2_SCL
 * PA10     ------> I2C2_SDA
 */
/********************************************************************/
/* BMP180 Register Map */
#define BMP180_XOUT_LSB 	0xF8
#define BMP180_OUT_LSB		0xF7
#define BMP180_OUT_MSB		0xF6
#define BMP180_CTRL_MEAS	0xF4
#define BMP180_CHIPID		0xD0
#define BMP180_SOFT_RESET	0xE0

#define BMP180_AC1_MSB		0xAA
#define BMP180_AC1_LSB		0xAB
#define BMP180_AC2_MSB		0xAC
#define BMP180_AC2_LSB		0xAD
#define BMP180_AC3_MSB		0xAE
#define BMP180_AC3_LSB		0xAF
#define BMP180_AC4_MSB		0xB0
#define BMP180_AC4_LSB		0xB1
#define BMP180_AC5_MSB		0xB2
#define BMP180_AC5_LSB		0xB3
#define BMP180_AC6_MSB		0xB4
#define BMP180_AC6_LSB		0xB5

#define BMP180_B1_MSB		0xB6
#define BMP180_B1_LSB		0xB7
#define BMP180_B2_MSB		0xB8
#define BMP180_B2_LSB		0xB9
#define BMP180_MB_MSB		0xBA
#define BMP180_MB_LSB		0xBB
#define BMP180_MC_MSB		0xBC
#define BMP180_MC_LSB		0xBD
#define BMP180_MD_MSB		0xBE
#define BMP180_MD_LSB		0xBF

#define BMP180_I2C_ADDRESS 	(0xee)
/* End BMP180 Register Map */
/********************************************************************/
/* Settings */

#define I_AM_BMP180							0x55

#define BMP180_INIT_TIMEOUT					0xFFFFF
#define BMP180_I2C_TIMEOUT					0xFFFFF

#define BMP180_OSS							0x2
#define CONVERSION_TIME_P_MS                13.5f
#define CONVERSION_TIME_T_MS                4.5f

/* cycles to wait for conversion complete */
#define BMP180_P_READOUT_DELAY      (uint8_t) ( CONVERSION_TIME_P_MS / SCHEDULER_INTERVALL_ms + 1 )
#define BMP180_T_READOUT_DELAY      (uint8_t) ( CONVERSION_TIME_T_MS / SCHEDULER_INTERVALL_ms + 1 )

/* Read Pressure every BMP180_READOUT_CYCLE scheduler interval
 * Read Temperature every BMP180_PRESSURE_TEMP_RATIO pressure measurement
 * */
#define BMP180_READOUT_CYCLE                3
#define BMP180_PRESSURE_TEMP_RATIO			2

/* lowpass for pressure readings*/
#define BMP180_PRESSURE_TP                  40

/* End Settings */
/********************************************************************/
/********************************************************************/

class BMP180: public Task {
public:
    BMP180(Status* statusPtr, uint8_t defaultPrio, I2C_HandleTypeDef* i2c);
    virtual ~BMP180();

    void update();
    void initialize();
    void kill();

    void receptionCompleteCallback();

private:
    I2C_HandleTypeDef* bmp_i2c;

    uint8_t getIdentification();
    void getCalibrationData();

    void getTemp();
    void getPressure();

    void calculateTemp();
    void calculatePressure();

    int32_t temp;

    /* Calibration Data */
    int16_t ac1;
    int16_t ac2;
    int16_t ac3;
    uint16_t ac4;
    uint16_t ac5;
    uint16_t ac6;

    int16_t b1;
    int16_t b2;
    int32_t b5;
    int16_t mb;
    int16_t mc;
    int16_t md;

    uint8_t cycle_counter;
    uint8_t pressure_counter;

    bool idle :1;
    bool i2cBusy :1;
    bool readingTemp :1;
    bool readingPressure :1;
    bool readingComplete :1;
    bool error :1;

    /* dummy padding variable */
    int pad :26;
};

#endif /* BMP180_H_ */
