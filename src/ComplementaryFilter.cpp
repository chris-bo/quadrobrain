/*
 * ComplementaryFilter.cpp
 *
 *  Created on: Jan 19, 2015
 *      Author: bohni
 */

#include "ComplementaryFilter.h"

ComplementaryFilter::ComplementaryFilter(Status* statusPtr, uint8_t defaultPrio,
		float filter_coefficient) : Task(statusPtr,defaultPrio) {

	dt = SCHEDULER_INTERVALL_ms * 0.001f;
	coefficient = filter_coefficient;
	tmp_acc_angle = 0;

	initialize();

}

ComplementaryFilter::~ComplementaryFilter() {

}

void ComplementaryFilter::update() {
	/* X */
	tmp_acc_angle = atan2f(status->accelY , status->accelZ);
	status->angleX = coefficient * (status->angleX + status->rateX*dt ) + (1 - coefficient) *tmp_acc_angle;

	/* X */
	tmp_acc_angle = atan2f(status->accelX , status->accelZ);
	status->angleY = coefficient * (status->angleY + status->rateY*dt ) + (1 - coefficient) *tmp_acc_angle;


}

void ComplementaryFilter::initialize() {

	SET_FLAG(taskStatusFlags,TASK_FLAG_ACTIVE);
}
