/*
 * Status.h
 *
 *  Created on: Dec 10, 2014
 *      Author: bohni
 */

#ifndef STATUS_H_
#define STATUS_H_

#include "stm32f3xx_hal.h"
#include "config.h"

class Status {
public:
	Status();
	virtual ~Status();

	/* all status variables */

	/* status flags */
	uint32_t globalFlags;

	/* acceleration [m/s ^ 2]*/
	float accelX;
	float accelY;
	float accelZ;

	/* gyro rates [°/s]*/
	float rateX;
	float rateY;
	float rateZ;

	/* angle */
	float angleX;
	float angleY;
	float angleZ;

	/* receiver values */
	uint8_t RCvalues[5];

	/* Motor values */
	uint8_t motorValues[4];



};

#endif /* STATUS_H_ */
