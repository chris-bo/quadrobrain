/*
 * Compass.cpp
 *
 *  Created on: Apr 28, 2015
 *      Author: bohni
 */

#include "Compass.h"

Compass::Compass(Status* statusPtr, uint8_t defaultPrio, float* magn1,
        float* magn2, float* ang1, float* ang2, float* rotation, float* output,
        float filter_coefficient)
		: ComplementaryFilter(statusPtr, defaultPriority, magn1, magn2,
		        rotation, output, filter_coefficient) {

	angle1 = ang1;
	angle2 = ang2;

}

Compass::~Compass() {

}

void Compass::update() {

	/*
	 * North:
	 * - calculate horizontal fields using angles
	 * - calculate angle between two magnetic fields
	 * -> get angle between north(x a2) and y (a1)
	 * */

	float x1 = *a1 * cosf(*angle1 * M_PI / 180);
	float x2 = *a2 * cosf(*angle2 * M_PI / 180);
	tmp_acc_angle = atan2f(x1, x2) * 180.0f / M_PI;

	/* mix old angle with new angle and rotation measurements */
	*out = coefficient * (*out + *rot * dt)
	        + (1.0f - coefficient) * tmp_acc_angle;

	/*  handle overruns
	 *  important for north calculation
	 *  */
	if (*out > 180) {
		*out = *out - 180;
	} else if (*out < -180) {
		*out = *out + 180;
	}

}
