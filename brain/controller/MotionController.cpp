/*
 * MotionController.cpp
 *
 *  Created on: Jul 8, 2015
 *      Author: bohni
 */

#include "MotionController.h"

MotionController::MotionController(Status* _status, int8_t _defaultPrio)
            : Task(_status, _defaultPrio) {

    horizontalAccelerationFiltered = {0,0,0};

    /* todo set to throttle if automated control is enabled*/
    throttle_out = new float();

    /* setup filters */
    accelerationFilterX = new MAfilterF(status, 0 ,&status->horizontalAcceleration.x,
                &horizontalAccelerationFiltered.x, ACCELERATION_MA_FILTER_SIZE);

    accelerationFilterY = new MAfilterF(status, 0 ,&status->horizontalAcceleration.y,
                &horizontalAccelerationFiltered.y, ACCELERATION_MA_FILTER_SIZE);

    accelerationFilterZ = new MAfilterF(status, 0 ,&status->horizontalAcceleration.z,
                &horizontalAccelerationFiltered.z, ACCELERATION_MA_FILTER_SIZE);

    /* setup pids*/
    veloctityPIDx = new PIDController(_status, 0,
                (float) SCHEDULER_INTERVALL_ms / 1000.0f, &status->velocity.x,
                &horizontalAccelerationFiltered.x, &status->velocitySetpoint.x,
                &status->accelerationSetpoint.x, 1, VELOCITY_PID_LIMIT,
                VELOCITY_PID_SUM_LIMIT, true);

    veloctityPIDy = new PIDController(status, 0,
                (float) SCHEDULER_INTERVALL_ms / 1000.0f, &status->velocity.y,
                &horizontalAccelerationFiltered.y, &status->velocitySetpoint.y,
                &status->accelerationSetpoint.y, 1, VELOCITY_PID_LIMIT,
                VELOCITY_PID_SUM_LIMIT, true);

    veloctityPIDz = new PIDController(status, 0,
                (float) SCHEDULER_INTERVALL_ms / 1000.0f, &status->velocity.z,
                &horizontalAccelerationFiltered.z, &status->velocitySetpoint.z,
                &status->accelerationSetpoint.z, 1, VELOCITY_PID_LIMIT,
                VELOCITY_PID_SUM_LIMIT, true);

    accelerationPIDx = new PIDController(status, 0,
                (float) SCHEDULER_INTERVALL_ms / 1000.0f, &horizontalAccelerationFiltered.x,
                0, &status->accelerationSetpoint.x, &status->angleSetpoint.x, 1,
                ACCELERATION_PID_LIMIT, ACCELERATION_PID_SUM_LIMIT, false);

    accelerationPIDy = new PIDController(status, 0,
                (float) SCHEDULER_INTERVALL_ms / 1000.0f, &horizontalAccelerationFiltered.y,
                0, &status->accelerationSetpoint.y, &status->angleSetpoint.y, 1,
                ACCELERATION_PID_LIMIT, ACCELERATION_PID_SUM_LIMIT, false);

    accelerationPIDz = new PIDController(status, 0,
                (float) SCHEDULER_INTERVALL_ms / 1000.0f, &horizontalAccelerationFiltered.z,
                0, &status->accelerationSetpoint.z, throttle_out, 1, ACCELERATION_PID_LIMIT,
                ACCELERATION_PID_SUM_LIMIT, false);
}
MotionController::~MotionController() {

}

void MotionController::update() {

    /* calc horizontal accelerations */

    status->horizontalAcceleration.x = cosf(status->angle.y) * status->accel.x;
    status->horizontalAcceleration.y = cosf(status->angle.x) * status->accel.y;
    status->horizontalAcceleration.z = cosf(status->angle.x) * cosf(status->angle.y)
                * status->accel.z - G;

    /* filter accelerations */
    accelerationFilterX->update();
    accelerationFilterY->update();
    accelerationFilterZ->update();

    /* calc velocities */

    status->velocity.x += horizontalAccelerationFiltered.x
                * (float) SCHEDULER_INTERVALL_ms / 1000.0f;
    status->velocity.y += horizontalAccelerationFiltered.y
                * (float) SCHEDULER_INTERVALL_ms / 1000.0f;
    status->velocity.z += horizontalAccelerationFiltered.z
                * (float) SCHEDULER_INTERVALL_ms / 1000.0f;

    /* call PID updates */

    veloctityPIDx->update();
    veloctityPIDy->update();
    veloctityPIDz->update();

    accelerationPIDx->update();
    accelerationPIDy->update();
    accelerationPIDz->update();
}

void MotionController::initialize(PID_Settings* _velocityPIDsettings,
            PID_Settings* _accelerationPIDsettings) {

    accelerationFilterX->initialize();
    accelerationFilterY->initialize();
    accelerationFilterZ->initialize();

    veloctityPIDx->initialize(_velocityPIDsettings);
    veloctityPIDy->initialize(_velocityPIDsettings);
    veloctityPIDz->initialize(_velocityPIDsettings);

    accelerationPIDx->initialize(_accelerationPIDsettings);
    accelerationPIDy->initialize(_accelerationPIDsettings);
    accelerationPIDz->initialize(_accelerationPIDsettings);

    SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);

}

void MotionController::reset() {

    veloctityPIDx->reset();
    veloctityPIDy->reset();
    veloctityPIDz->reset();

    accelerationPIDx->reset();
    accelerationPIDy->reset();
    accelerationPIDz->reset();
}
