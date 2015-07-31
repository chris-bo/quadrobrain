/*
 * Status.cpp
 *
 *  Created on: Dec 10, 2014
 *      Author: bohni
 */

#include "Status.h"

Status::Status() {
    /* init status variables */

    uptime = 0;

    accel = {0,0,0};

    rate = {0,0,0};

    magnetfield = {0,0,0};

    angle = {0,0,0};
    angleSetpoint = {0,0,0};

    velocity = {0,0,0};
    velocitySetpoint = {0,0,0};

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

    pidSettingsVelocity.p = PID_VELOCITY_P;
    pidSettingsVelocity.i = PID_VELOCITY_I;
    pidSettingsVelocity.d = PID_VELOCITY_D;
    pidSettingsVelocity.scaleSetPoint = PID_VELOCITY_SCALE;
    pidSettingsVelocity.gain = PID_VELOCITY_GAIN;

    pidSettingsacceleration.p = PID_ACCELERATION_P;
    pidSettingsacceleration.i = PID_ACCELERATION_I;
    pidSettingsacceleration.d = PID_ACCELERATION_D;
    pidSettingsacceleration.scaleSetPoint = PID_ACCELERATION_SCALE;
    pidSettingsacceleration.gain = PID_ACCELERATION_GAIN;

    motorSetpoint = {0,0,0};

    globalFlags = 0;

    akkuVoltage = 0;

    cpuLoad = 0;

    buzzerQueue1 = {};
    buzzerQueue2 = {};

    gpsData = {};
    qcSettings = {};

    /* Initialize Hardcoded Settings */
#ifndef DISABLE_RC_SIGNAL_LOST_BUZZER_WARNING
    qcSettings.enableBuzzerWarningRCLost = 1;
#else
    qcSettings.enableBuzzerWarningRCLost = 0;
#endif
#ifndef DISABLE_LOW_VOLTAGE_BUZZER_WARNING
    qcSettings.enableBuzzerWarningLowVoltage = 1;
#else
    qcSettings.enableBuzzerWarningLowVoltage = 0;
#endif

}

Status::~Status() {
}

void Status::reset() {
    /* reset status variables */
    accel = {0,0,0};

    rate = {0,0,0};

    magnetfield = {0,0,0};

    angle = {0,0,0};
    angleSetpoint = {0,0,0};

    velocity = {0,0,0};
    velocitySetpoint = {0,0,0};

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

    motorSetpoint = {0,0,0};

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

    pidSettingsVelocity.p = PID_VELOCITY_P;
    pidSettingsVelocity.i = PID_VELOCITY_I;
    pidSettingsVelocity.d = PID_VELOCITY_D;
    pidSettingsVelocity.scaleSetPoint = PID_VELOCITY_SCALE;
    pidSettingsVelocity.gain = PID_VELOCITY_GAIN;

    pidSettingsacceleration.p = PID_ACCELERATION_P;
    pidSettingsacceleration.i = PID_ACCELERATION_I;
    pidSettingsacceleration.d = PID_ACCELERATION_D;
    pidSettingsacceleration.scaleSetPoint = PID_ACCELERATION_SCALE;
    pidSettingsacceleration.gain = PID_ACCELERATION_GAIN;

}

void Status::addToneToQueue(Buzzer_Queue_t* queue, float frequency,
            uint16_t length) {

    /* goto next address */
    queue->index++;
    if (queue->index == BUZZER_QUEUE_SIZE) {
        queue->index = 0;
    }

    /* add tone */
    queue->frequency[queue->index] = frequency;
    queue->lenght[queue->index] = length;

}
