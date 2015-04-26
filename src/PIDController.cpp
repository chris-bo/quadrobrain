/*
 * PIDController.cpp
 *
 *  Created on: 22.03.2015
 *      Author: Jonas
 */

#include "PIDController.h"

PIDController::PIDController(Status* statusPtr, uint8_t defaultPrio,
		float sampleTime, float* input, float* derivedInput, float* reference,
		float* output, float limit, bool useDerivedInput) :
		Task(statusPtr, defaultPrio) {
	status = statusPtr;
	this->input = input;
	this->derivedInput = derivedInput;
	this->reference = reference;
	this->output = output;
	this->p = 0.0f;
	this->i = 0.0f;
	this->d = 0.0f;
	this->limit = limit;
	this->sampleTime = sampleTime;
	sum = 0.0f;
	oldValue = 0.0f;
	this->useDerivedInput = useDerivedInput;
}

PIDController::~PIDController() {
	// TODO Auto-generated destructor stub
}

void PIDController::update() {
	//TODO: Throttle limitieren

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

void PIDController::initialize(float p, float i, float d) {
	// Task aktivieren
	SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
	this->p = p;
	this->i = i;
	this->d = d;
}
