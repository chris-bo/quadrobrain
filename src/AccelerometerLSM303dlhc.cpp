/*
 * AccelerometerLSM303dlhc.cpp
 *
 *  Created on: Jan 6, 2015
 *      Author: bohni
 */

#include "AccelerometerLSM303dlhc.h"
uint8_t buffer[6];

Accelerometer_LSM303dlhc::Accelerometer_LSM303dlhc(Status* statusPtr,
		uint8_t defaultPrio, I2C_HandleTypeDef* i2c) :
		Task(statusPtr, defaultPrio) {

	accel_i2c = i2c;
	rawAccelerometerValues[0] = 0;
	rawAccelerometerValues[1] = 0;
	rawAccelerometerValues[2] = 0;
	accelerometerFlags = 0;

#if (ACCELEROMETER_RANGE == LSM303DLHC_FULLSCALE_2G)
	scale = 0.001f;
#elif (ACCELEROMETER_RANGE == LSM303DLHC_FULLSCALE_4G)
	scale = 0.002f;
#elif (ACCELEROMETER_RANGE == LSM303DLHC_FULLSCALE_8G)
	scale = 0.004f;
#elif (ACCELEROMETER_RANGE == LSM303DLHC_FULLSCALE_16G)
	scale = 0.012f;
#else
#error "accelerometer range not set"
#endif

}

Accelerometer_LSM303dlhc::~Accelerometer_LSM303dlhc() {

}

void Accelerometer_LSM303dlhc::update() {

	status->accelX = rawAccelerometerValues[0] * scale;
	status->accelY = rawAccelerometerValues[1] * scale;
	status->accelZ = rawAccelerometerValues[2] * scale;

	if (GET_FLAG(accelerometerFlags, ACCEL_FLAG_DATA_PROCESSED)) {
		RESET_FLAG(accelerometerFlags, ACCEL_FLAG_DATA_PROCESSED);
		getAccelerometerData();
	} else {
		SET_FLAG(accelerometerFlags, ACCEL_FLAG_DATA_PROCESSED);
	}
	if (GET_FLAG(accelerometerFlags, ACCEL_FLAG_REQUEST_I2CBUS_GET_DATA)) {
		getAccelerometerData();
	}
}

void Accelerometer_LSM303dlhc::receptionCompleteCallback() {

	/* save Raw Values
	 * LSB @ lower address
	 * */
	rawAccelerometerValues[0] = (int16_t) (buffer[0] | (buffer[1] << 8));
	rawAccelerometerValues[1] = (int16_t) (buffer[2] | (buffer[3] << 8));
	rawAccelerometerValues[2] = (int16_t) (buffer[4] | (buffer[5] << 8));

	RESET_FLAG(accelerometerFlags, ACCEL_FLAG_TRANSFER_RUNNING);
	SET_FLAG(accelerometerFlags, ACCEL_FLAG_TRANSFER_COMPLETE);
	RESET_FLAG(accelerometerFlags, ACCEL_FLAG_DATA_PROCESSED);

	/* todo error management */
}

void Accelerometer_LSM303dlhc::transmissionCompleteCallback() {
}

void Accelerometer_LSM303dlhc::initialize() {

	HAL_I2C_MspInit(accel_i2c);
	/* Initialize Accelerometer*/
	accelerometerInit();
	/* set transfer complete flag
	 * -> bus idle state
	 * */
	SET_FLAG(accelerometerFlags, ACCEL_FLAG_TRANSFER_COMPLETE);

	getAccelerometerData();
}

void Accelerometer_LSM303dlhc::accelerometerInit() {

	/* Initialize Accelerometer without IT*/

	/* CTRL1
	 * Output Rate
	 * Power Mode
	 * Enable all Axes
	 * */
	buffer[0] = (ACCELEROMETER_OUTPUT_RATE | LSM303DLHC_NORMAL_MODE
			| LSM303DLHC_AXES_ENABLE);

	/* CTRL2
	 * HighPass Filter
	 */
	buffer[1] = (LSM303DLHC_HPM_NORMAL_MODE);

	/* CTRL3
	 * DRDY on int1
	 */
	buffer[2] = LSM303DLHC_IT1_DRY1;

	/* CTRL4
	 * Block data Update
	 * LSB @ lower address
	 * Fullscale setting
	 * HighResolution setting
	 */
	buffer[3] = (LSM303DLHC_BlockUpdate_Continous | LSM303DLHC_BLE_LSB
			| ACCELEROMETER_RANGE | ACCELEROMETER_HR);

	/* CTRL5 */
	buffer[4] = 0;

	/* CTRL6 */
	buffer[5] = 0;

	HAL_I2C_Mem_Write(accel_i2c, ACC_I2C_ADDRESS,
			(LSM303DLHC_MULTIPLE_BYTE_OPERATION | LSM303DLHC_CTRL_REG1_A),
			I2C_MEMADD_SIZE_8BIT, buffer, 6, ACCEL_INIT_TIMEOUT);

	/* other registers left at default */

	/* activate task */
	SET_FLAG(statusFlags, FLAG_ACTIVE);

}

void Accelerometer_LSM303dlhc::getAccelerometerData() {

	if (accel_i2c->State == HAL_I2C_STATE_READY) {
		if (GET_FLAG(accelerometerFlags, ACCEL_FLAG_TRANSFER_COMPLETE)) {
			RESET_FLAG(accelerometerFlags, ACCEL_FLAG_TRANSFER_COMPLETE);
			SET_FLAG(accelerometerFlags, ACCEL_FLAG_TRANSFER_RUNNING);

			HAL_I2C_Mem_Read_IT(accel_i2c, ACC_I2C_ADDRESS,
					(LSM303DLHC_OUT_X_L_A | LSM303DLHC_MULTIPLE_BYTE_OPERATION),
					I2C_MEMADD_SIZE_8BIT, buffer, 6);
			RESET_FLAG(accelerometerFlags, ACCEL_FLAG_REQUEST_I2CBUS_GET_DATA);
		} else {
			/* TODO accel transfer not complete error */
			/* simple fix -> set flag and recall function*/
			if (accel_i2c->State == HAL_I2C_STATE_READY) {
				SET_FLAG(accelerometerFlags, ACCEL_FLAG_TRANSFER_COMPLETE);
				getAccelerometerData();
			}
		}
	} else {
		SET_FLAG(accelerometerFlags, ACCEL_FLAG_REQUEST_I2CBUS_GET_DATA);
	}
}
