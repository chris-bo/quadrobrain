/*
 * Status.cpp
 *
 *  Created on: Dec 10, 2014
 *      Author: bohni
 */

#include "Status.h"

Status::Status() {
    /* init status variables */

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

    motorSetpoint = {0,0,0};
    motorValues[0] = 0;
    motorValues[1] = 0;
    motorValues[2] = 0;
    motorValues[3] = 0;

    gpsData = {};

    uptime = 0;
    akkuVoltage = 0;
    globalFlags = 0;
    cpuLoad = 0;

    buzzerQueue1 = {};
    buzzerQueue2 = {};

    filterCoefficientXY = FILTER_COEFFICIENT_XY;
    filterCoefficientZ = FILTER_COEFFICIENT_Z;

    pidSettingsAngleXY.p = PID_XY_P;
    pidSettingsAngleXY.i = PID_XY_I;
    pidSettingsAngleXY.d = PID_XY_D;
    pidSettingsAngleXY.scaleSetPoint = PID_XY_SCALE;
    pidSettingsAngleXY.gain = PID_XY_GAIN;

    pidSettingsRotationZ.p = PID_Z_P;
    pidSettingsRotationZ.i = PID_Z_I;
    pidSettingsRotationZ.d = PID_Z_D;
    pidSettingsRotationZ.scaleSetPoint = PID_Z_SCALE;
    pidSettingsRotationZ.gain = PID_Z_GAIN;

    pidSettingsVelocity.p = PID_VELOCITY_P;
    pidSettingsVelocity.i = PID_VELOCITY_I;
    pidSettingsVelocity.d = PID_VELOCITY_D;
    pidSettingsVelocity.scaleSetPoint = PID_VELOCITY_SCALE;
    pidSettingsVelocity.gain = PID_VELOCITY_GAIN;

    pidSettingsAcceleration.p = PID_ACCELERATION_P;
    pidSettingsAcceleration.i = PID_ACCELERATION_I;
    pidSettingsAcceleration.d = PID_ACCELERATION_D;
    pidSettingsAcceleration.scaleSetPoint = PID_ACCELERATION_SCALE;
    pidSettingsAcceleration.gain = PID_ACCELERATION_GAIN;

    qcSettings = {};

    /* Initialize Hardcoded Settings */
#ifdef DISABLE_RC_SIGNAL_LOST_BUZZER_WARNING
    qcSettings.enableBuzzerWarningRCLost = 0;
#else
    qcSettings.enableBuzzerWarningRCLost = 1;
#endif
#ifdef DISABLE_LOW_VOLTAGE_BUZZER_WARNING
    qcSettings.enableBuzzerWarningLowVoltage = 0;
#else
    qcSettings.enableBuzzerWarningLowVoltage = 1;
#endif
    qcSettings.enableFlightLeds = 1;
    qcSettings.enableMotors = 1;
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

    motorSetpoint = {0,0,0};
    motorValues[0] = 0;
    motorValues[1] = 0;
    motorValues[2] = 0;
    motorValues[3] = 0;

    gpsData = {};

    /* do not reset uptime */
    akkuVoltage = 0;
    globalFlags = 0;
    cpuLoad = 0;

    buzzerQueue1 = {};
    buzzerQueue2 = {};

    /* Do not reset filter coefficients */
}

void Status::restoreConfig() {
    /* restores hardcoded values */

    filterCoefficientXY = FILTER_COEFFICIENT_XY;
    filterCoefficientZ = FILTER_COEFFICIENT_Z;

    pidSettingsAngleXY.p = PID_XY_P;
    pidSettingsAngleXY.i = PID_XY_I;
    pidSettingsAngleXY.d = PID_XY_D;
    pidSettingsAngleXY.scaleSetPoint = PID_XY_SCALE;
    pidSettingsAngleXY.gain = PID_XY_GAIN;

    pidSettingsRotationZ.p = PID_Z_P;
    pidSettingsRotationZ.i = PID_Z_I;
    pidSettingsRotationZ.d = PID_Z_D;
    pidSettingsRotationZ.scaleSetPoint = PID_Z_SCALE;
    pidSettingsRotationZ.gain = PID_Z_GAIN;

    pidSettingsVelocity.p = PID_VELOCITY_P;
    pidSettingsVelocity.i = PID_VELOCITY_I;
    pidSettingsVelocity.d = PID_VELOCITY_D;
    pidSettingsVelocity.scaleSetPoint = PID_VELOCITY_SCALE;
    pidSettingsVelocity.gain = PID_VELOCITY_GAIN;

    pidSettingsAcceleration.p = PID_ACCELERATION_P;
    pidSettingsAcceleration.i = PID_ACCELERATION_I;
    pidSettingsAcceleration.d = PID_ACCELERATION_D;
    pidSettingsAcceleration.scaleSetPoint = PID_ACCELERATION_SCALE;
    pidSettingsAcceleration.gain = PID_ACCELERATION_GAIN;

    /* Restore Hardcoded Settings */
#ifdef DISABLE_RC_SIGNAL_LOST_BUZZER_WARNING
    qcSettings.enableBuzzerWarningRCLost = 0;
#else
    qcSettings.enableBuzzerWarningRCLost = 1;
#endif
#ifdef DISABLE_LOW_VOLTAGE_BUZZER_WARNING
    qcSettings.enableBuzzerWarningLowVoltage = 0;
#else
    qcSettings.enableBuzzerWarningLowVoltage = 1;
#endif
    qcSettings.enableFlightLeds = 1;
    qcSettings.enableMotors = 1;

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
