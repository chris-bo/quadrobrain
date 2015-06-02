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

	ac1 = 0;
	ac2 = 0;
	ac3 = 0;
	ac4 = 0;
	ac5 = 0;
	ac6 = 0;

	b1 = 0;
	b2 = 0;
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
		/* Measurement Triggered -> readout data*/
		HAL_I2C_Mem_Read_IT(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_OUT_MSB,
				I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 3);
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
	resetPriority();
}

void BMP180::initialize() {

	if (getIdentification() == 0) {

		SET_FLAG(taskStatusFlags, BMP180_FLAG_ERROR);
		return;

	}

	getCalibrationData();
	SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);

}

void BMP180::receptionCompleteCallback() {

	SET_FLAG(taskStatusFlags, BMP180_READING_DATA_COMPLETE);

}

uint8_t BMP180::getIdentification() {

	HAL_I2C_Mem_Read(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_CHIPID,
			I2C_MEMADD_SIZE_8BIT, i2c2_buffer, 1, BMP180_I2C_TIMEOUT);

	if (i2c2_buffer[0] == I_AM_BMP180) {
		return 1;
	}
	return 0;
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

	HAL_I2C_Mem_Read(bmp_i2c, BMP180_I2C_ADDRESS, BMP180_AC1_MSB,
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
    /* TODO BMP180 CALC TEMP */


}

void BMP180::calculatePressure() {
	RESET_FLAG(taskStatusFlags, BMP180_READING_PRESSURE);
    /* TODO BMP180 CALC Pressure */
}
