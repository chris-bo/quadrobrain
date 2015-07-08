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

    pressure = 0;
    height = 0;
    height_rel = 0;
    d_h = 0;

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

    filterCoefficientXY = 0;
    filterCoefficientZ = 0;

    pidSettigsAngleXY.p = PID_XY_P;
    pidSettigsAngleXY.i = PID_XY_I;
    pidSettigsAngleXY.d = PID_XY_D;
    pidSettigsAngleXY.scaleSetPoint = PID_XY_SCALE;
    pidSettigsAngleXY.gain = PID_XY_GAIN;

    pidSettigsRotationZ.p = PID_Z_P;
    pidSettigsRotationZ.i = PID_Z_I;
    pidSettigsRotationZ.d = PID_Z_D;
    pidSettigsRotationZ.scaleSetPoint = PID_Z_SCALE;
    pidSettigsRotationZ.gain = PID_Z_GAIN;

    motorSetPointX = 0;
    motorSetPointY = 0;
    motorSetPointZ = 0;

    globalFlags = 0;

    akkuVoltage = 0;

    cpuLoad = 0;

    buzzer1Busy = false;
    buzzer2Busy = false;
}

Status::~Status() {
}

void Status::reset() {
    /* reset status variables */
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

    pressure = 0;
    height = 0;
    height_rel = 0;
    d_h = 0;

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

    filterCoefficientXY = 0;
    filterCoefficientZ = 0;

    pidSettigsAngleXY.p = PID_XY_P;
    pidSettigsAngleXY.i = PID_XY_I;
    pidSettigsAngleXY.d = PID_XY_D;
    pidSettigsAngleXY.scaleSetPoint = PID_XY_SCALE;
    pidSettigsAngleXY.gain = PID_XY_GAIN;

    pidSettigsRotationZ.p = PID_Z_P;
    pidSettigsRotationZ.i = PID_Z_I;
    pidSettigsRotationZ.d = PID_Z_D;
    pidSettigsRotationZ.scaleSetPoint = PID_Z_SCALE;
    pidSettigsRotationZ.gain = PID_Z_GAIN;


    motorSetPointX = 0;
    motorSetPointY = 0;
    motorSetPointZ = 0;

    globalFlags = 0;

    akkuVoltage = 0;

}

void Status::restoreConfig() {
    /* restores hardcoded values */

    filterCoefficientXY = FILTER_COEFFICIENT_XY;
    filterCoefficientZ = FILTER_COEFFICIENT_Z;

    pidSettigsAngleXY.p = PID_XY_P;
    pidSettigsAngleXY.i = PID_XY_I;
    pidSettigsAngleXY.d = PID_XY_D;
    pidSettigsAngleXY.scaleSetPoint = PID_XY_SCALE;
    pidSettigsAngleXY.gain = PID_XY_GAIN;

    pidSettigsRotationZ.p = PID_Z_P;
    pidSettigsRotationZ.i = PID_Z_I;
    pidSettigsRotationZ.d = PID_Z_D;
    pidSettigsRotationZ.scaleSetPoint = PID_Z_SCALE;
    pidSettigsRotationZ.gain = PID_Z_GAIN;

}
