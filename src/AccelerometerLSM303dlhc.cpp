/*
 * AccelerometerLSM303dlhc.cpp
 *
 *  Created on: Jan 6, 2015
 *      Author: bohni
 */

#include "AccelerometerLSM303dlhc.h"
uint8_t buffer[6];

Accelerometer_LSM303dlhc::Accelerometer_LSM303dlhc(Status* statusPtr,
		uint8_t defaultPrio, I2C_HandleTypeDef* i2c) : Task(statusPtr,defaultPrio){

	accel_i2c = i2c;
	rawAccelerometerValues[0] = 0;
	rawAccelerometerValues[1] = 0;
	rawAccelerometerValues[2] = 0;
	accelerometerFlags = 0;

	HAL_I2C_MspInit(accel_i2c);

#if (ACCELEROMETER_RANGE == LSM303DLHC_FULLSCALE_2G)
	scale = 1.0f;
#elif (ACCELEROMETER_RANGE == LSM303DLHC_FULLSCALE_4G)
	scale = 2.0f;
#elif (ACCELEROMETER_RANGE == LSM303DLHC_FULLSCALE_8G)
	scale = 4.0f;
#elif (ACCELEROMETER_RANGE == LSM303DLHC_FULLSCALE_16G)
	scale = 12.0f;
#else
#error "accelerometer range not set"
#endif

}

Accelerometer_LSM303dlhc::~Accelerometer_LSM303dlhc() {

}

void Accelerometer_LSM303dlhc::update() {
	if (GET_FLAG(accelerometerFlags, ACCEL_FLAG_TRANSFER_COMPLETE)) {
		status->accelX = rawAccelerometerValues[0] / scale;
		status->accelY = rawAccelerometerValues[1] / scale;
		status->accelZ = rawAccelerometerValues[2] / scale;
	}
}

void Accelerometer_LSM303dlhc::DrdyCallback() {
}

void Accelerometer_LSM303dlhc::receptionCompleteCallback() {
	if (GET_FLAG(accelerometerFlags, ACCEL_FLAG_TRANSFER_RUNNING)) {
		/* save Raw Values
		 * LSB @ lower address
		 * */
		rawAccelerometerValues[0] = (int16_t) (buffer[0] | (buffer[1] << 8));
		rawAccelerometerValues[1] = (int16_t) (buffer[2] | (buffer[3] << 8));
		rawAccelerometerValues[2] = (int16_t) (buffer[4] | (buffer[5] << 8));

		RESET_FLAG(accelerometerFlags, ACCEL_FLAG_TRANSFER_RUNNING);
		SET_FLAG(accelerometerFlags, ACCEL_FLAG_TRANSFER_COMPLETE);
	}
	/* todo error management */
}

void Accelerometer_LSM303dlhc::transmissionCompleteCallback() {
}

void Accelerometer_LSM303dlhc::initialize() {
	/* Initialize Accelerometer*/
	accelerometerInit();

	/* set transfer complete flag
	 * -> bus idle state
	 * */
	SET_FLAG(accelerometerFlags, ACCEL_FLAG_TRANSFER_COMPLETE);
}

void Accelerometer_LSM303dlhc::accelerometerInit() {

}

void Accelerometer_LSM303dlhc::getAccelerometerData() {
	/* TODO check bus idle */

	if (GET_FLAG(accelerometerFlags, ACCEL_FLAG_TRANSFER_COMPLETE)) {
		RESET_FLAG(accelerometerFlags, ACCEL_FLAG_TRANSFER_COMPLETE);
		SET_FLAG(accelerometerFlags, ACCEL_FLAG_TRANSFER_RUNNING);
		HAL_I2C_Mem_Read_IT(accel_i2c, ACC_I2C_ADDRESS,
		LSM303DLHC_OUT_X_L_A, I2C_MEMADD_SIZE_8BIT, buffer, 6);
	} else {
		/* TODO accel transfer not complete error */
	}

}
