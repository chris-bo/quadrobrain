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
        bool _useDerivedInput) :
        Task(statusPtr, defaultPrio) {
    status = statusPtr;
    this->processVariable = _processVariable;
    this->derivedProcessVariable = _derivedProcessVariable;
    this->setPoint = _setPoint;
    this->controlValue = _controlValue;
    this->controlValueGain = _controlValueGain;
    settings = 0;
    this->limit = _limit;
    this->sampleTime = _sampleTime;
    eSum = 0.0f;
    sumLimit = _sumLimit;
    oldValue = 0.0f;

    this->useDerivedInput = _useDerivedInput;

    /* dummy padding variable */
    pad = 0;
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
        float e = (*setPoint * settings->scaleSetPoint - pv);
        eSum += e;

        if (useDerivedInput) {
            // Falls Ableitung vorhanden wird diese direkt verwendet
            temp = (settings->p) * e + (settings->i) * sampleTime * eSum
                    + (settings->d)
                            * (((*setPoint) - oldValue) / sampleTime
                                    * (settings->scaleSetPoint)
                                    - (*derivedProcessVariable));
            oldValue = *setPoint;

        } else {
            // falls nicht wird die Ableitung berechnet
            temp = (settings->p) * e + (settings->i) * sampleTime * eSum
                    + (settings->d) * (e - oldValue) / sampleTime;

            oldValue = e;
        }

        // beachte limit
        if (temp > limit) {
            temp = limit;
        } else if (temp < -1 * limit) {
            temp = -1 * limit;

        }

        // Reglerwert ausgeben
        *controlValue = (settings->gain) * controlValueGain * temp;

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

void PIDController::initialize(PID_Settings* _settings) {
    // Task aktivieren
    taskActive = true;
    settings = _settings;
}

void PIDController::kill() {

    reset();
    eSum = 0.0f;
    oldValue = 0.0f;
    taskActive = false;
}
