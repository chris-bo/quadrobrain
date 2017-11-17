/*
 * Altimeter.cpp
 *
 *  Created on: Nov 17, 2017
 *      Author: chris
 */

#include "Altimeter.h"

Altimeter::Altimeter(Status* _status, int8_t _defaultPrio) :
		Task(_status, _defaultPrio) {
	   height_start = 0;
}

Altimeter::~Altimeter() {

}

void Altimeter::initialize() {

	status->height = 0;
	height_start = 0;
	status->height_rel =0;
	 SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}

void Altimeter::update() {

	/* calc height from pressure*/

	    float x1 = (float) (status->pressure) / 101325.0f;
	    float x2 = (1 - powf(x1, 0.190294957f));
	    x2 = x2 * 44330;

	    if (!GET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE)) {
	        // initializing routine
	        status->height = x2;
	        height_start =  x2;
	        status->height_rel = 0;
	        status->d_h =0;
	    } else {
	        status->d_h = x2  - status->height;
	        status->height = x2;
	        status->height_rel = status->height - height_start;
	    }
}

void Altimeter::reset() {


	   height_start = 0;

	   RESET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}
