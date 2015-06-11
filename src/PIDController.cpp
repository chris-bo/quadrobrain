/*
 * PIDController.cpp
 *
 *  Created on: 22.03.2015
 *      Author: Jonas
 */

#include "PIDController.h"
PIDController::PIDController(Status* statusPtr, uint8_t defaultPrio,
            float _sampleTime, float* _input, float* _derivedInput,
            float* _reference, float* _output, float _limit, float _sum_limit,
            bool _useDerivedInput)
: Task(statusPtr, defaultPrio) {
    status = statusPtr;
    this->input = _input;
    this->derivedInput = _derivedInput;
    this->reference = _reference;
    this->output = _output;
    this->p = 0;
    this->i = 0;
    this->d = 0;
    this->limit = _limit;
    this->sampleTime = _sampleTime;
    sum = 0.0f;
    sum_limit = _sum_limit;
    oldValue = 0.0f;
    scale = 1;
    this->useDerivedInput = _useDerivedInput;
}

PIDController::~PIDController() {

}

void PIDController::update() {

    float scaled_input = *input * scale;
    float temp;
    float e = (*reference - scaled_input);
    sum += e;

    /* enable PID if using throttle > threshold */
    if (status->rcSignalThrottle > PID_THROTTLE_THRESHOLD) {
        if (useDerivedInput) {
            // Falls Ableitung vorhanden wird diese direkt verwendet
            temp = (*p) * e + (*i) * sampleTime * sum + (*d) * (*derivedInput * scale);
        } else {
            // falls nicht wird die Ableitung berechnet
            temp = (*p) * e + (*i) * sampleTime * sum
                        + (*d) * (scaled_input - oldValue) / sampleTime;

            // alten Wert speichern fuer naechste Differenzierung
            oldValue = scaled_input;
        }

        // beachte limit
        if (temp > limit) {
            temp = limit;
        } else if (temp < -1 * limit) {
            temp = -1 * limit;

        }

        // Reglerwert ausgeben
        *output = temp;

        /* limit sum*/
        if (sum > sum_limit) {
            sum = sum_limit;
        } else if (sum < -1 * sum_limit) {
            sum = -1 * sum_limit;

        }
    } else {
        /* reset sum */
        sum = 0;
        oldValue = 0;
        *output = 0;
    }
}

void PIDController::initialize(float* _p, float* _i, float* _d, float _scale) {
    // Task aktivieren
    SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
    this->p = _p;
    this->i = _i;
    this->d = _d;
    scale = _scale;
}

void PIDController::kill() {

    reset();
    p = 0;
    i = 0;
    d = 0;
    scale = 1;
    sum = 0.0f;
    oldValue = 0.0f;
    RESET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}
