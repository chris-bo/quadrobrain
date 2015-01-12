/*
 * AccelerometerLSM303dlhc.h
 *
 *  Created on: Jan 6, 2015
 *      Author: bohni
 */

#ifndef ACCELEROMETERLSM303DLHC_H_
#define ACCELEROMETERLSM303DLHC_H_

#include "Task.h"
#include "config.h"
#include "stm32f3xx_hal.h"
#include "lsm303dlhc.h"
#include "i2c.h"

#define ACCEL_INIT_TIMEOUT							0xFFFFF

/* Accelerometer Flags */
#define ACCEL_FLAG_TRANSFER_RUNNING					0x0100
#define ACCEL_FLAG_TRANSFER_COMPLETE				0x0200
#define ACCEL_FLAG_DATA_PROCESSED					0x0400

#define ACCEL_FLAG_REQUEST_I2CBUS_GET_DATA			0x4000
#define ACCEL_FLAG_ERROR							0x8000

class Accelerometer_LSM303dlhc: public Task {
public:
	Accelerometer_LSM303dlhc(Status* statusPtr, uint8_t defaultPrio,
			I2C_HandleTypeDef* i2c);
	virtual ~Accelerometer_LSM303dlhc();

	void update();
	void receptionCompleteCallback();
	void transmissionCompleteCallback();
	void initialize();
	void getAccelerometerData();


private:

	I2C_HandleTypeDef* accel_i2c;
	float scale;
	int16_t rawAccelerometerValues[3]; /* X,Y,Z */

	void accelerometerInit();

	uint8_t getIdentification();


};

#endif /* ACCELEROMETERLSM303DLHC_H_ */