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
	angleNorth = 0;

	temp = 0;

	rcSignalRoll = 0;
	rcSignalNick = 0;
	rcSignalYaw = 0;
	rcSignalThrottle = 0;
	rcSignalLinPoti = 0;
	rcSignalSwitch = 0;
	rcSignalEnable = 0;

	motorValues[0] = 0;
	motorValues[1] = 0;
	motorValues[2] = 0;
	motorValues[3] = 0;

	pXY = 0;
	iXY = 0;
	dXY = 0;

	pZ = 0;
	iZ = 0;
	dZ = 0;

	globalFlags = 0;

	akkuVoltage = 0;
}

Status::~Status() {
}

