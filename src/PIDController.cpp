/*
 * PIDController.cpp
 *
 *  Created on: 22.03.2015
 *      Author: Jonas
 */

#include "PIDController.h"

PIDController::PIDController(Status* statusPtr, uint8_t defaultPrio,
            float _sampleTime, float* _processVariable,
            float* _derivedProcessVariable, float* _setPoint, float* _controlValue,
            float _controlValueGain, float _limit, float _sumLimit,
            bool _useDerivedInput)
            : Task(statusPtr, defaultPrio) {
    status = statusPtr;
    this->processVariable = _processVariable;
    this->derivedProcessVariable = _derivedProcessVariable;
    this->setPoint = _setPoint;
    this->controlValue = _controlValue;
    this->controlValueGain = _controlValueGain;
    this->p = 0;
    this->i = 0;
    this->d = 0;
    this->scale = 0;
    this->gain = 0;
    this->limit = _limit;
    this->sampleTime = _sampleTime;
    eSum = 0.0f;
    sumLimit = _sumLimit;
    oldValue = 0.0f;

    this->useDerivedInput = _useDerivedInput;
}

PIDController::~PIDController() {

}

void PIDController::update() {
    /*
     esum = esum + e
     y = Kp * e + Ki * Ta * esum + Kd * (e – ealt)/Ta
     ealt = e
     */
    /* enable PID if using throttle > threshold */
    if (status->rcSignalThrottle > PID_THROTTLE_THRESHOLD) {

#ifdef PID_ROUND_PROCESS_VARIABLE
        float pv = ((float) (int32_t) (*processVariable * 1000)) / 1000.0f;
#else
        float pv = *processVariable;
#endif
        float temp;
        float e = (*setPoint * *scale - pv);
        eSum += e;

        if (useDerivedInput) {
            // Falls Ableitung vorhanden wird diese direkt verwendet
            temp = (*p) * e + (*i) * sampleTime * eSum
                        + (*d)
                                    * (((*setPoint) - oldValue) * (*scale)
                                                - (*derivedProcessVariable));
            oldValue = *setPoint;

        } else {
            // falls nicht wird die Ableitung berechnet
            temp = (*p) * e + (*i) * sampleTime * eSum
                        + (*d) * (e - oldValue) / sampleTime;

            oldValue = e;
        }

        // beachte limit
        if (temp > limit) {
            temp = limit;
        } else if (temp < -1 * limit) {
            temp = -1 * limit;

        }

        // Reglerwert ausgeben
        *controlValue = (*gain) * controlValueGain * temp;

        /* limit sum*/
        if (sumLimit != INFINITY) {
            if (eSum > sumLimit) {
                eSum = sumLimit;
            } else if (eSum < -1 * sumLimit) {
                eSum = -1 * sumLimit;
            }
        }
    } else {
        /* reset sum */
        eSum = 0;
        oldValue = 0;
        *controlValue = 0;
    }
}

void PIDController::initialize(float* _p, float* _i, float* _d, float* _gain,
            float* _scale) {
    // Task aktivieren
    SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
    this->p = _p;
    this->i = _i;
    this->d = _d;
    this->scale = _scale;
    this->gain = _gain;
}

void PIDController::kill() {

    reset();
    eSum = 0.0f;
    oldValue = 0.0f;
    RESET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}
