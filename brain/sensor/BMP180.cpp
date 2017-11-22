/*
 * BMP180.cpp
 *
 *  Created on: Apr 28, 2015
 *      Author: bohni
 */

#include "BMP180.h"

uint8_t i2c2_buffer[4];

BMP180::BMP180(Status* statusPtr, uint8_t defaultPrio, I2C_HandleTypeDef* i2c)
: Task(statusPtr, defaultPrio) {

    bmp_i2c = i2c;
    temp = 0;
    height_start = 0;

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
}

BMP180::~BMP180() {

}

void BMP180::update() {

    if (GET_FLAG(taskStatusFlags, BMP180_READING_DATA_COMPLETE)) {
        RESET_FLAG(taskStatusFlags, BMP180_READING_DATA_COMPLETE);
        /* Data Reception was Completed */
        if (GET_FLAG(taskStatusFlags, BMP180_READING_PRESSURE)) {
            calculatePressure();
        } else {
            calculateTemp();
        }
        SET_FLAG(taskStatusFlags, BMP180_IDLE);
    } else if (GET_FLAG(taskStatusFlags,
                BMP180_READING_PRESSURE) || GET_FLAG(taskStatusFlags,BMP180_READING_TEMP)) {
        if (!GET_FLAG(taskStatusFlags, BMP180_FLAG_I2C_BUSY)) {
            /* Do this only if you are currently not reading anything*/
            if ((cycle_counter == BMP180_P_READOUT_DELAY
                        && GET_FLAG(taskStatusFlags, BMP180_READING_PRESSURE))
                        || (cycle_counter == BMP180_T_READOUT_DELAY
                                    && GET_FLAG(taskStatusFlags, BMP180_READING_TEMP))) {

                /* TWO cases:
                 * 1. reading pressure is set and waited readout delay for pressure
                 * 2. reading temperature is set and waited readout delay for temperature
                 */
                cycle_counter = 0;
                /* Measurement Triggered -> readout data*/
                SET_FLAG(taskStatusFlags, BMP180_FLAG_I2C_BUSY);
                HAL_I2C_Mem_Read_IT(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_OUT_MSB,
                            I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 3);
            } else {
                cycle_counter++;
            }
        }
    } else if (GET_FLAG(taskStatusFlags, BMP180_IDLE)) {
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
            RESET_FLAG(taskStatusFlags, BMP180_IDLE);
        } else {
            cycle_counter++;
        }
    }
}

void BMP180::initialize() {

    if (getIdentification() == 0) {

        SET_FLAG(taskStatusFlags, BMP180_FLAG_ERROR);
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

    SET_FLAG(taskStatusFlags, BMP180_IDLE);
    SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
    SET_FLAG(status->globalFlags, BMP180_OK_FLAG);
}

void BMP180::receptionCompleteCallback() {
    RESET_FLAG(taskStatusFlags, BMP180_FLAG_I2C_BUSY);
    SET_FLAG(taskStatusFlags, BMP180_READING_DATA_COMPLETE);
}

uint8_t BMP180::getIdentification() {

    HAL_I2C_Mem_Read(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_CHIPID,
                I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 1, BMP180_I2C_TIMEOUT);

    if (i2c2_buffer[0] == I_AM_BMP180) {
        return 1;
    }
    return 0;
    RESET_FLAG(status->globalFlags, BMP180_OK_FLAG);
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
    SET_FLAG(taskStatusFlags, BMP180_READING_TEMP);
}

void BMP180::getPressure() {
    i2c2_buffer[0] = (uint8_t) (0x34 | ( BMP180_OSS << 6));
    HAL_I2C_Mem_Write_IT(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_CTRL_MEAS,
                I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 1);
    SET_FLAG(taskStatusFlags, BMP180_READING_PRESSURE);
}

void BMP180::calculateTemp() {

    RESET_FLAG(taskStatusFlags, BMP180_READING_TEMP);
    uint32_t ut;
    int32_t x1;
    int32_t x2;

    /* calc routine - see datasheet */
    ut = (uint16_t) (i2c2_buffer[0] << 8 | i2c2_buffer[1]);
    x1 = ((ut - ac6) * ac5) / 32768;
    x2 = (mc * 2048) / (x1 + md);
    b5 = x1 + x2;
    temp = (b5 + 8) / 16;

}

void BMP180::calculatePressure() {
    RESET_FLAG(taskStatusFlags, BMP180_READING_PRESSURE);

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
    if (GET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE) == 0) {
        // initializing routine
        status->pressure = p;
    } else {
        status->pressure = (p * BMP180_PRESSURE_TP
                    + status->pressure * (100 - BMP180_PRESSURE_TP)) / 100;
    }
    calculateHeight();
}

void BMP180::calculateHeight() {
    float x1 = (float) (status->pressure) / 101325.0f;
    float x2 = (1 - powf(x1, 0.190294957f));
    x2 = x2 * 44330;

    if (!GET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE)) {
        // initializing routine
        status->height = x2;
        height_start =  x2;
        status->height_rel = 0;
        status->d_h =0;
    } else {
        status->d_h = x2  - status->height;
        status->height = x2;
        status->height_rel = status->height - height_start;
        status->temp = (float) temp / 10.0f;
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
    height_start = 0;

    RESET_FLAG(status->globalFlags, BMP180_OK_FLAG);
    RESET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}
