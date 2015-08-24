/*
 * Compass.cpp
 *
 *  Created on: Apr 28, 2015
 *      Author: bohni
 */

#include "Compass.h"

Compass::Compass(Status* statusPtr, uint8_t defaultPrio, float* magnX, float* magnY,
            float* angX, float* angY, float* rotationZ, float* outputZ,
            float* filter_coefficient)
            : ComplementaryFilter(statusPtr, defaultPriority, magnX, magnY,
                        rotationZ, outputZ, filter_coefficient) {

    angle1 = angX;
    angle2 = angY;

}

Compass::~Compass() {

}

void Compass::update() {

    /*
     * North:
     * - calculate horizontal fields using angles
     * - calculate angle between two magnetic fields
     * -> get angle between north x (a2) and y (a1)
     * */

    /* todo fix compass */

    float x1 = *a1 * cosf(*angle1 * M_PI / 180);
    float x2 = *a2 * cosf(*angle2 * M_PI / 180);
    tmp_acc_angle = atan2f(x2, x1) * 180.0f / M_PI;

    /* mix old angle with new angle and rotation measurements */
    *out = *coefficient * (*out + *rot * dt) + (1.0f - *coefficient) * tmp_acc_angle;

    /*  handle overruns
     *  important for north calculation
     *  */
    if (*out > 180.0f) {
        *out = *out - 180.0f;
    } else if (*out < -180.0f) {
        *out = *out + 180.0f;
    }

}
