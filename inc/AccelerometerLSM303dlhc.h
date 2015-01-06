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

/* Accelerometer Flags */
#define ACCEL_FLAG_TRANSFER_RUNNING					0x01
#define ACCEL_FLAG_TRANSFER_COMPLETE				0x02

#define AccelComp_FLAG_ERROR						0x80

class Accelerometer_LSM303dlhc: public Task {
public:
	Accelerometer_LSM303dlhc(Status* statusPtr, uint8_t defaultPrio,
			I2C_HandleTypeDef* i2c);
	virtual ~Accelerometer_LSM303dlhc();

	void update();
	void DrdyCallback();
	void receptionCompleteCallback();
	void transmissionCompleteCallback();
	void initialize();
	uint8_t accelerometerFlags;

private:
	int16_t rawAccelerometerValues[3]; /* X,Y,Z */
	I2C_HandleTypeDef* accel_i2c;

	void accelerometerInit();
	void getAccelerometerData();

	float scale;

};

#endif /* ACCELEROMETERLSM303DLHC_H_ */
