/*
 * IMU.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: chris
 */

#include <IMU.h>

IMU::IMU(Status* _status, int8_t _defaultPrio) :
		Task(_status, _defaultPrio) {

	/* setup complementary filters */
	compFilterX = new ComplementaryFilter(status, 0, &status->accel.y, &status->accel.z, &status->rate.x,
			&status->angle.x, &status->filterCoefficientXY);
	compFilterY = new ComplementaryFilter(status, 0, &status->accel.x, &status->accel.z, &status->rate.y,
			&status->angle.y, &status->filterCoefficientXY);
	compFilterNorth = new Compass(status, 0, &status->magnetfield.x, &status->magnetfield.y,
			&status->angle.x, &status->angle.y, &status->rate.z,
			&status->angle.z, &status->filterCoefficientZ);

	horizontalAccelerationFiltered = {0,0,0};

	/* setup filters  for horizontal acceleration*/
	accelerationFilterX = new MAfilterF(status, 0,
			&status->horizontalAcceleration.x,
			&horizontalAccelerationFiltered.x, ACCELERATION_MA_FILTER_SIZE);

	accelerationFilterY = new MAfilterF(status, 0,
			&status->horizontalAcceleration.y,
			&horizontalAccelerationFiltered.y, ACCELERATION_MA_FILTER_SIZE);

	accelerationFilterZ = new MAfilterF(status, 0,
			&status->horizontalAcceleration.z,
			&horizontalAccelerationFiltered.z, ACCELERATION_MA_FILTER_SIZE);

}

IMU::~IMU() {

}

void IMU::initialize() {

	compFilterX->initialize();
	compFilterY->initialize();
	compFilterNorth->initialize();

	accelerationFilterX->initialize();
	accelerationFilterY->initialize();
	accelerationFilterZ->initialize();

	SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);

}

void IMU::update() {
	compFilterX->update();
	compFilterY->update();
	compFilterNorth->update();

	/* calc horizontal accelerations */

	status->horizontalAcceleration.x = cosf(status->angle.y) * status->accel.x;
	status->horizontalAcceleration.y = cosf(status->angle.x) * status->accel.y;
	status->horizontalAcceleration.z = cosf(status->angle.x)
			* cosf(status->angle.y) * status->accel.z - G;

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
}

void IMU::reset() {

	/* reset velocities and accelerations and angles */
	compFilterX->kill();
	compFilterY->kill();
	compFilterNorth->kill();

	horizontalAccelerationFiltered = {0,0,0};

	status->velocity.x = 0;
	status->velocity.y = 0;
	status->velocity.z = 0;

}
