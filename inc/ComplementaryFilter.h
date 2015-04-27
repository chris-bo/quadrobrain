/*
 * ComplementaryFilter.h
 *
 *  Created on: Jan 19, 2015
 *      Author: bohni
 */

#ifndef COMPLEMENTARYFILTER_H_
#define COMPLEMENTARYFILTER_H_

#include "Task.h"
#include "config.h"
#include "math.h"

class ComplementaryFilter: public Task {
public:
	ComplementaryFilter(Status* statusPtr, uint8_t defaultPrio, float* accel1,
			float* accel2, float* rotation, float* output,
			float filter_coefficient);
	virtual ~ComplementaryFilter();
	void update();
	void initialize();

protected:
	float dt;
	float coefficient;
	float tmp_acc_angle;
	float* a1;
	float* a2;
	float* rot;
	float* out;
};

#endif /* COMPLEMENTARYFILTER_H_ */
