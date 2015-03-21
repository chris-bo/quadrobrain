/*
 * AccelerometerLSM303dlhc.cpp
 *
 *  Created on: Jan 6, 2015
 *      Author: bohni
 */

#include "AccelerometerLSM303dlhc.h"
uint8_t accel_transfer_buffer[6];

Accelerometer_LSM303dlhc::Accelerometer_LSM303dlhc(Status* statusPtr,
		uint8_t defaultPrio, I2C_HandleTypeDef* i2c) :
		Task(statusPtr, defaultPrio) {

	accel_i2c = i2c;
	rawAccelerometerValues[0] = 0;
	rawAccelerometerValues[1] = 0;
	rawAccelerometerValues[2] = 0;
	zeroGBias[0] = 0;
	zeroGBias[1] = 0;
	zeroGBias[2] = 0;

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

	status->accelX = (rawAccelerometerValues[0] - zeroGBias[0]) * scale;
	status->accelY = (rawAccelerometerValues[1] - zeroGBias[1]) * scale;
	status->accelZ = (rawAccelerometerValues[2] - zeroGBias[2]) * scale;

	if (GET_FLAG(taskStatusFlags, ACCEL_FLAG_DATA_PROCESSED)) {
		/* no interrupt occured since last update*/
		RESET_FLAG(taskStatusFlags, ACCEL_FLAG_DATA_PROCESSED);
		getAccelerometerData();
	} else {
		SET_FLAG(taskStatusFlags, ACCEL_FLAG_DATA_PROCESSED);
	}
	if (GET_FLAG(taskStatusFlags, ACCEL_FLAG_REQUEST_I2CBUS_GET_DATA)) {
		getAccelerometerData();
	}
}

void Accelerometer_LSM303dlhc::receptionCompleteCallback() {

	/* save Raw Values
	 * LSB @ lower address
	 * */

	rawAccelerometerValues[0] = (int16_t) (accel_transfer_buffer[0]
			| (accel_transfer_buffer[1] << 8))/16;
	rawAccelerometerValues[1] = (int16_t) (accel_transfer_buffer[2]
			| (accel_transfer_buffer[3] << 8))/16;
	rawAccelerometerValues[2] = (int16_t) (accel_transfer_buffer[4]
			| (accel_transfer_buffer[5] << 8))/16;

	RESET_FLAG(taskStatusFlags, ACCEL_FLAG_TRANSFER_RUNNING);
	SET_FLAG(taskStatusFlags, ACCEL_FLAG_TRANSFER_COMPLETE);
	RESET_FLAG(taskStatusFlags, ACCEL_FLAG_DATA_PROCESSED);

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
	SET_FLAG(taskStatusFlags, ACCEL_FLAG_TRANSFER_COMPLETE);

	getBias();
}

void Accelerometer_LSM303dlhc::accelerometerInit() {

	/* Check device */
	if (getIdentification() == 1) {

		/* Initialize Accelerometer without IT or DMA*/
		/* CTRL1
		 * Output Rate
		 * Power Mode
		 * Enable all Axes
		 * */
		accel_transfer_buffer[0] = (ACCELEROMETER_OUTPUT_RATE
				| LSM303DLHC_NORMAL_MODE | LSM303DLHC_AXES_ENABLE);

		/* CTRL2
		 * HighPass Filter
		 */
		accel_transfer_buffer[1] = (LSM303DLHC_HPM_NORMAL_MODE);

		/* CTRL3
		 * DRDY on int1
		 */
		accel_transfer_buffer[2] = LSM303DLHC_IT1_DRY1;

		/* CTRL4
		 * Block data Update
		 * LSB @ lower address
		 * Fullscale setting
		 * HighResolution setting
		 */
		accel_transfer_buffer[3] = (LSM303DLHC_BlockUpdate_Continous
				| LSM303DLHC_BLE_LSB | ACCELEROMETER_RANGE | LSM303DLHC_HR_DISABLE);

		/* CTRL5 */
		accel_transfer_buffer[4] = 0;

		/* CTRL6 */
		accel_transfer_buffer[5] = 0;

		HAL_I2C_Mem_Write(accel_i2c, ACC_I2C_ADDRESS,
				(LSM303DLHC_MULTIPLE_BYTE_OPERATION | LSM303DLHC_CTRL_REG1_A),
				I2C_MEMADD_SIZE_8BIT, accel_transfer_buffer, 6,
				ACCEL_INIT_TIMEOUT);

		/* other registers left at default */



		/* activate task */
		SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
	} else {

	}
}

void Accelerometer_LSM303dlhc::getAccelerometerData() {

	if (accel_i2c->State == HAL_I2C_STATE_READY) {
		if (GET_FLAG(taskStatusFlags, ACCEL_FLAG_TRANSFER_COMPLETE)) {
			RESET_FLAG(taskStatusFlags, ACCEL_FLAG_TRANSFER_COMPLETE);
			SET_FLAG(taskStatusFlags, ACCEL_FLAG_TRANSFER_RUNNING);

			HAL_I2C_Mem_Read_DMA(accel_i2c, ACC_I2C_ADDRESS,
					(LSM303DLHC_OUT_X_L_A | LSM303DLHC_MULTIPLE_BYTE_OPERATION),
					I2C_MEMADD_SIZE_8BIT, accel_transfer_buffer, 6);
			RESET_FLAG(taskStatusFlags, ACCEL_FLAG_REQUEST_I2CBUS_GET_DATA);
		} else {
			/* TODO accel transfer not complete error */
			/* simple fix -> set flag and recall function*/
			if (accel_i2c->State == HAL_I2C_STATE_READY) {
				SET_FLAG(taskStatusFlags, ACCEL_FLAG_TRANSFER_COMPLETE);
				getAccelerometerData();
			}
		}
	} else {
		SET_FLAG(taskStatusFlags, ACCEL_FLAG_REQUEST_I2CBUS_GET_DATA);
	}
}

uint8_t Accelerometer_LSM303dlhc::getIdentification() {

	uint8_t tmpreg;

	HAL_I2C_Mem_Read(accel_i2c, ACC_I2C_ADDRESS,
	LSM303DLHC_WHO_AM_I_ADDR,
	I2C_MEMADD_SIZE_8BIT, &tmpreg, 1, ACCEL_INIT_TIMEOUT);

	if (tmpreg == I_AM_LMS303DLHC) {
		return 1;
	} else {
		/* if not lsm303dlhc */
		RESET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
		priority = -1;
		SET_FLAG(taskStatusFlags, ACCEL_FLAG_ERROR);
		SET_FLAG(status->globalFlags, EMERGENCY_FLAG);
		/*stop program*/
		while (1)
			;
		return 0;
	}
}

void Accelerometer_LSM303dlhc::getBias() {

	int32_t tmp[3];
	getAccelerometerData();
	/* for each axis */
	tmp[0] = rawAccelerometerValues[0];
	tmp[1] = rawAccelerometerValues[1];
	tmp[2] = rawAccelerometerValues[2];

	for (uint8_t i = 0; i<100;i++){
		HAL_Delay(7);
		tmp[0] = (tmp[0] + rawAccelerometerValues[0]);
		tmp[1] = (tmp[1] + rawAccelerometerValues[1]);
		tmp[2] = (tmp[2] + rawAccelerometerValues[2]);
		getAccelerometerData();
	}

	tmp[0] /=100;
	tmp[1] /=100;
	tmp[2] /=100;
	zeroGBias[0] = (int16_t)tmp[0];
	zeroGBias[1] = (int16_t)tmp[1];
	zeroGBias[2] = 0;

	scale =  (1.0f / (float) tmp[2]);

}
