/*
 * BMP180.cpp
 *
 *  Created on: Apr 28, 2015
 *      Author: bohni
 */

#include "BMP180.h"

uint8_t i2c2_buffer[4];

BMP180::BMP180(Status* statusPtr, uint8_t defaultPrio, I2C_HandleTypeDef* i2c) :
        Task(statusPtr, defaultPrio) {

    bmp_i2c = i2c;
    temp = 0;

    ac1 = 0;
    ac2 = 0;
    ac3 = 0;
    ac4 = 0;
    ac5 = 0;
    ac6 = 0;

    b1 = 0;
    b2 = 0;
    b5 = 0;
    mb = 0;
    mc = 0;
    md = 0;

    cycle_counter = 0;
    pressure_counter = 0;

    idle = true;
    i2cBusy = true;
    readingTemp = false;
    readingPressure = false;
    readingComplete = false;
    error = false;

    /* dummy padding variable */
    pad = 0;
}

BMP180::~BMP180() {

}

void BMP180::update() {

    if (readingComplete) {
        readingComplete = false;
        /* Data Reception was Completed */
        if (readingPressure) {
            calculatePressure();
        } else {
            calculateTemp();
        }
        idle = true;
    } else if (readingPressure || readingTemp) {
        if (!i2cBusy) {
            /* Do this only if you are currently not reading anything*/
            if ((cycle_counter == BMP180_P_READOUT_DELAY && readingPressure)
                    || (cycle_counter == BMP180_T_READOUT_DELAY && readingTemp)) {

                /* TWO cases:
                 * 1. reading pressure is set and waited readout delay for pressure
                 * 2. reading temperature is set and waited readout delay for temperature
                 */
                cycle_counter = 0;
                /* Measurement Triggered -> readout data*/
                i2cBusy = true;
                HAL_I2C_Mem_Read_IT(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_OUT_MSB,
                I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 3);
            } else {
                cycle_counter++;
            }
        }
    } else if (idle) {
        /* no measurement triggered, no data read, no reading in progress */
        if (cycle_counter == BMP180_READOUT_CYCLE) {
            if (pressure_counter == BMP180_PRESSURE_TEMP_RATIO) {
                getTemp();
                pressure_counter = 0;
            } else {
                getPressure();
                pressure_counter++;
            }
            cycle_counter = 0;
            idle = false;
        } else {
            cycle_counter++;
        }
    }
}

void BMP180::initialize() {

    if (getIdentification() == 0) {

        error = true;
        return;

    }
    getCalibrationData();

    /* init process
     *
     * read temp
     * calc temp
     * read pressure
     * calc pressure
     */

    /* get temp */
    i2c2_buffer[0] = 0x2E;
    HAL_I2C_Mem_Write(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_CTRL_MEAS,
    I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 1, BMP180_I2C_TIMEOUT);

    HAL_Delay(5);
    HAL_I2C_Mem_Read(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_OUT_MSB,
    I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 2, BMP180_I2C_TIMEOUT);
    calculateTemp();

    /* get pressure */
    i2c2_buffer[0] = (uint8_t) (0x34 | ( BMP180_OSS << 6));
    HAL_I2C_Mem_Write(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_CTRL_MEAS,
    I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 1, BMP180_I2C_TIMEOUT);

    HAL_Delay(50);
    HAL_I2C_Mem_Read(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_OUT_MSB,
    I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 3, BMP180_I2C_TIMEOUT);
    calculatePressure();

    idle = true;
    taskActive = true;
    status->globalFlags.BMP180ok = true;
}

void BMP180::receptionCompleteCallback() {
    i2cBusy = false;
    readingComplete = true;
}

uint8_t BMP180::getIdentification() {

    HAL_I2C_Mem_Read(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_CHIPID,
    I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 1, BMP180_I2C_TIMEOUT);

    if (i2c2_buffer[0] == I_AM_BMP180) {
        return 1;
    }
    return 0;
    status->globalFlags.BMP180ok = false;
}

void BMP180::getCalibrationData() {

    HAL_I2C_Mem_Read(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_AC1_MSB,
    I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 4, BMP180_I2C_TIMEOUT);
    ac1 = (int16_t) ((i2c2_buffer[0] << 8) | i2c2_buffer[1]);
    ac2 = (int16_t) ((i2c2_buffer[2] << 8) | i2c2_buffer[3]);

    HAL_I2C_Mem_Read(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_AC3_MSB,
    I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 4, BMP180_I2C_TIMEOUT);
    ac3 = (int16_t) ((i2c2_buffer[0] << 8) | i2c2_buffer[1]);
    ac4 = (uint16_t) ((i2c2_buffer[2] << 8) | i2c2_buffer[3]);

    HAL_I2C_Mem_Read(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_AC5_MSB,
    I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 4, BMP180_I2C_TIMEOUT);
    ac5 = (uint16_t) ((i2c2_buffer[0] << 8) | i2c2_buffer[1]);
    ac6 = (uint16_t) ((i2c2_buffer[2] << 8) | i2c2_buffer[3]);

    HAL_I2C_Mem_Read(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_B1_MSB,
    I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 4, BMP180_I2C_TIMEOUT);
    b1 = (int16_t) ((i2c2_buffer[0] << 8) | i2c2_buffer[1]);
    b2 = (int16_t) ((i2c2_buffer[2] << 8) | i2c2_buffer[3]);

    HAL_I2C_Mem_Read(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_MB_MSB,
    I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 4, BMP180_I2C_TIMEOUT);
    mb = (int16_t) ((i2c2_buffer[0] << 8) | i2c2_buffer[1]);
    mc = (int16_t) ((i2c2_buffer[2] << 8) | i2c2_buffer[3]);

    HAL_I2C_Mem_Read(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_MD_MSB,
    I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 2, BMP180_I2C_TIMEOUT);
    md = (int16_t) ((i2c2_buffer[0] << 8) | i2c2_buffer[1]);
}

void BMP180::getTemp() {
    i2c2_buffer[0] = 0x2E;
    HAL_I2C_Mem_Write_IT(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_CTRL_MEAS,
    I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 1);
    readingTemp = true;
}

void BMP180::getPressure() {
    i2c2_buffer[0] = (uint8_t) (0x34 | ( BMP180_OSS << 6));
    HAL_I2C_Mem_Write_IT(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_CTRL_MEAS,
    I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 1);
    readingPressure = true;
}

void BMP180::calculateTemp() {

    readingTemp = false;
    uint32_t ut;
    int32_t x1;
    int32_t x2;

    /* calc routine - see datasheet */
    ut = (uint16_t) (i2c2_buffer[0] << 8 | i2c2_buffer[1]);
    x1 = ((ut - ac6) * ac5) / 32768;
    x2 = (mc * 2048) / (x1 + md);
    b5 = x1 + x2;
    temp = (b5 + 8) / 16;
    status->temp = (float) temp / 10.0f;

}

void BMP180::calculatePressure() {
    readingPressure = false;

    uint32_t up;
    int32_t x1;
    int32_t x2;
    int32_t x3;
    int32_t b3;
    uint32_t b4;
    uint32_t b7;
    int32_t b6 = b5 - 4000;
    int32_t p;

    /* calc routine - see datasheet */
    up = (i2c2_buffer[0] << 16 | i2c2_buffer[1] << 8 | i2c2_buffer[2])
            >> (8 - BMP180_OSS);
    x1 = (b2 * (b6 * b6) / 4096) / 2048;
    x2 = (ac2 * b6) / 2048;
    x3 = x1 + x2;
    b3 = ((((int32_t) ac1 * 4 + x3) << BMP180_OSS) + 2) / 4;
    x1 = (ac3 * b6) / 32768;
    x2 = (b1 * (b6 * b6) / 4096) / 32768;
    x3 = (x1 + x2 + 2) / 4;
    b4 = ac4 * (uint32_t) (x3 + 32768) / 32768;
    b7 = ((uint32_t) up - b3) * (50000 >> BMP180_OSS);
    if (b7 < 0x80000000) {
        p = b7 * 2 / b4;
    } else {
        p = b7 / b4 * 2;
    }
    x1 = p / 256 * p / 256;
    x1 = (x1 * 3038) / 65536;
    x2 = (-7357 * p) / 65536;
    p = p + (x1 + x2 + 3791) / 16;
    if (!taskActive) {
        // initializing routine
        status->pressure = p;
    } else {
        status->pressure = (p * BMP180_PRESSURE_TP
                + status->pressure * (100 - BMP180_PRESSURE_TP)) / 100;
    }
}

void BMP180::kill() {

    reset();

    ac1 = 0;
    ac2 = 0;
    ac3 = 0;
    ac4 = 0;
    ac5 = 0;
    ac6 = 0;

    b1 = 0;
    b2 = 0;
    b5 = 0;
    mb = 0;
    mc = 0;
    md = 0;

    cycle_counter = 0;
    pressure_counter = 0;

    status->globalFlags.BMP180ok = false;
    taskActive = false;
}
