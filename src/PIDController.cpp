/*
 * PIDController.cpp
 *
 *  Created on: 22.03.2015
 *      Author: Jonas
 */

#include "PIDController.h"

PIDController::PIDController(Status* statusPtr, uint8_t defaultPrio,
		float _sampleTime, float* _input, float* _derivedInput, float* _reference,
		float* _output, float _limit, bool _useDerivedInput) :
		Task(statusPtr, defaultPrio) {
	status = statusPtr;
	this->input = _input;
	this->derivedInput = _derivedInput;
	this->reference = _reference;
	this->output = _output;
	this->p = 0.0f;
	this->i = 0.0f;
	this->d = 0.0f;
	this->limit = _limit;
	this->sampleTime = _sampleTime;
	sum = 0.0f;
	oldValue = 0.0f;
	this->useDerivedInput = _useDerivedInput;
}

PIDController::~PIDController() {

}

void PIDController::update() {
	//TODO PIDController: Throttle limitieren

	float temp;

	//Neue Werte mittels PID-Regler ausrechnen
	if (useDerivedInput) {
		// Falls Ableitung vorhanden wird diese direkt verwendet
		temp = p * (*reference - *input) + i * sampleTime * sum
				+ d * *derivedInput;
	} else {
		// falls nicht wird die Ableitung berechnet
		temp = p * (*reference - *input) + i * sampleTime * sum
				+ d * (*input - oldValue) / sampleTime;

		// alten Wert speichern fuer naechste Differenzierung
		oldValue = *input;
	}

	// beachte limit
	if (temp > limit) {
		temp = limit;
	} else if (temp < 0) {
		if (temp > limit) {
			temp = -1 * limit;
		}
	}

	// Reglerwert ausgeben
	*output = temp;

	// Fehler Integrieren
	sum += (*reference - *input);
}

void PIDController::initialize(float _p, float _i, float _d) {
	// Task aktivieren
	SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
	this->p = _p;
	this->i = _i;
	this->d = _d;
}

void PIDController::kill() {

    reset();
    p = 0.0f;
    i = 0.0f;
    d = 0.0f;
    sum = 0.0f;
    oldValue = 0.0f;
    RESET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}
