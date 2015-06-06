/*
 * ComplementaryFilter.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: bohni
 */

#include "ComplementaryFilter.h"

ComplementaryFilter::ComplementaryFilter(Status* statusPtr, uint8_t defaultPrio,
        float* accel1, float* accel2, float* rotation, float* output,
        float* filter_coefficient)
		: Task(statusPtr, defaultPrio) {

	dt = SCHEDULER_INTERVALL_ms * 0.001f;
	coefficient = filter_coefficient;
	tmp_acc_angle = 0;

	a1 = accel1;
	a2 = accel2;
	rot = rotation;
	out = output;

	initialize();

}

ComplementaryFilter::~ComplementaryFilter() {

}

void ComplementaryFilter::update() {
	/* X + Y:
	 * calculate angle between two accelerations
	 * -> get angle between g(a2) and horizon (a1)
	 *
	 */
	tmp_acc_angle = atan2f(*a1, *a2) * 180.0f / M_PI;

	/* mix old angle with new angle and rotation measurements */
	*out = *coefficient * (*out + *rot * dt)
	        + (1.0f - *coefficient) * tmp_acc_angle;

}

void ComplementaryFilter::initialize() {

	SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}

void ComplementaryFilter::kill() {
}
