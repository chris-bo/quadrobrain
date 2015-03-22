/*
 * Status.cpp
 *
 *  Created on: Dec 10, 2014
 *      Author: bohni
 */

#include "Status.h"

Status::Status() {
	/* init status variables */
	accelX = 0;
	accelY = 0;
	accelZ = 0;

	rateX = 0;
	rateY = 0;
	rateZ = 0;

	magnetX = 0;
	magnetY = 0;
	magnetZ = 0;

	angleX = 0;
	angleY = 0;
	angleZ = 0;

	RCvalues[0] = 0;
	RCvalues[1] = 0;
	RCvalues[2] = 0;
	RCvalues[3] = 0;
	RCvalues[4] = 0;

	motorValues[0] = 0;
	motorValues[1] = 0;
	motorValues[2] = 0;
	motorValues[3] = 0;

	globalFlags = 0;
}

Status::~Status() {
}

