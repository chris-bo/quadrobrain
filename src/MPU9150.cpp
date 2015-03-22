/*
 * MPU9150.cpp
 *
 *  Created on: Mar 21, 2015
 *      Author: bohni
 */

#include "MPU9150.h"

MPU9150::MPU9150(Status* statusPtr, uint8_t defaultPrio, I2C_HandleTypeDef* i2c) :
		Task(statusPtr, defaultPrio) {
	mpu9150_i2c = i2c;

	rawAccelData[0] = 0;
	rawAccelData[1] = 0;
	rawAccelData[2] = 0;

	rawGyroData[0] = 0;
	rawGyroData[1] = 0;
	rawGyroData[2] = 0;

	rawMagnetData[0] = 0;
	rawMagnetData[1] = 0;
	rawMagnetData[2] = 0;

	scaleAccel = MPU9150_ACCEL_SCALE_FACTOR_2g * G;
	scaleGyro = MPU9150_GYRO_SCALE_FACTOR_250;
	scaleManget[0] = 0.3;
	scaleManget[1] = 0.3;
	scaleManget[2] = 0.3;
}

MPU9150::~MPU9150() {

}

void MPU9150::update() {
}

void MPU9150::initialize() {

	/* 1. Init i2c communication
	 *
	 * check ids
	 *
	 * 2. set mpu i2c bypass mode
	 * 		-> config magnetometer
	 * 		-> get sensitivity values for calculations
	 *
	 *
	 * 3. config mpu, incl readout of magnetometer
	 *
	 *
	 */
}

void MPU9150::receptionCompleteCallback() {
}

void MPU9150::transmissionCompleteCallback() {
}

void MPU9150::DRDYinterrupt() {
}

void MPU9150::scaleRawData() {

	status->accelX = rawAccelData[0] * scaleAccel;
	status->accelY = rawAccelData[1] * scaleAccel;
	status->accelZ = rawAccelData[2] * scaleAccel;

	status->rateX = rawGyroData[0] * scaleGyro;
	status->rateY = rawGyroData[1] * scaleGyro;;
	status->rateZ = rawGyroData[2] * scaleGyro;;

	status->magnetX = rawMagnetData[0] * scaleManget[0];
	status->magnetY = rawMagnetData[1] * scaleManget[1];
	status->magnetZ = rawMagnetData[2] * scaleManget[2];
}
