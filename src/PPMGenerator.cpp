/*
 * PPMGenerator.cpp
 *
 *  Created on: 30.12.2014
 *      Author: Jonas
 */

#include "PPMGenerator.h"

PPMGenerator::PPMGenerator(Status* statusPtr, uint8_t defaultPrio,
		TIM_HandleTypeDef* htim, float* controllerValueX,
		float* controllerValueY) :
		Task(statusPtr, defaultPrio) {
	PPMGenerator_htim = htim;
	this->controllerValueX = controllerValueX;
	this->controllerValueY = controllerValueY;
}

PPMGenerator::~PPMGenerator() {

}

void PPMGenerator::update() {

	// Motoren aktiviert?
	if (status->rcSignalEnable) {
		// Throttle hoch genug, damit Regler aktiv werden darf?
		if (status->rcSignalThrottle > THROTTLE_MIN) {
			status->motorValues[0] = status->rcSignalThrottle
					+ *controllerValueX;
			status->motorValues[1] = status->rcSignalThrottle
					+ *controllerValueY;
			status->motorValues[2] = status->rcSignalThrottle
					- *controllerValueX;
			status->motorValues[3] = status->rcSignalThrottle
					- *controllerValueY;
			// Throttle zu niedrig => Reglerwerte werden nicht beruecksichtig
		} else {
			status->motorValues[0] = status->rcSignalThrottle;
			status->motorValues[1] = status->rcSignalThrottle;
			status->motorValues[2] = status->rcSignalThrottle;
			status->motorValues[3] = status->rcSignalThrottle;
		}
		// Motoren sind deaktiviert => alle Werte auf 0 setzen
	} else {
		status->motorValues[0] = 0;
		status->motorValues[1] = 0;
		status->motorValues[2] = 0;
		status->motorValues[3] = 0;
	}

	// Fuer alle 4 Motoren prozentuale Throttle-Werte in Compare-Werte fuer den Timer wandeln
	// und anschliessend dem Timer uebergeben
	// Motor 1
	uint16_t temp =	status->motorValues[0] * (PPM_TIMER_MAX_PULSE_LENGTH
	            - PPM_TIMER_MIN_PULSE_LENGTH)+ PPM_TIMER_MIN_PULSE_LENGTH;
	__HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_1, temp);
	// Motor 2
	temp = status->motorValues[1] * (PPM_TIMER_MAX_PULSE_LENGTH
	            - PPM_TIMER_MIN_PULSE_LENGTH)+ PPM_TIMER_MIN_PULSE_LENGTH;
	__HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_2, temp);
	// Motor 3
	temp = status->motorValues[2] * (PPM_TIMER_MAX_PULSE_LENGTH
	            - PPM_TIMER_MIN_PULSE_LENGTH)+ PPM_TIMER_MIN_PULSE_LENGTH;
	__HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_3, temp);
	// Motor 4
	temp = status->motorValues[3] * (PPM_TIMER_MAX_PULSE_LENGTH
	            - PPM_TIMER_MIN_PULSE_LENGTH)+ PPM_TIMER_MIN_PULSE_LENGTH;
	__HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_4, temp);

	resetPriority();
}

void PPMGenerator::initialize() {
	// PPM-Timer starten
	HAL_TIM_PWM_Start(PPMGenerator_htim, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(PPMGenerator_htim, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(PPMGenerator_htim, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(PPMGenerator_htim, TIM_CHANNEL_4);

	// Motor-Kalibrierung

	// F�r alle 4 Motoren prozentuale Throttle-Werte in Compare-Werte f�r den Timer wandeln
	// und anschlie�end dem Timer �bergeben
	// Motor 1
	uint16_t temp = status->motorValues[0] * (PPM_TIMER_MAX_PULSE_LENGTH
	            - PPM_TIMER_MIN_PULSE_LENGTH) + PPM_TIMER_MIN_PULSE_LENGTH;
	__HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_1, temp);
	// Motor 2
	temp = status->motorValues[1] * (PPM_TIMER_MAX_PULSE_LENGTH
	            - PPM_TIMER_MIN_PULSE_LENGTH) + PPM_TIMER_MIN_PULSE_LENGTH;
	__HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_2, temp);
	// Motor 3
	temp = status->motorValues[2] * (PPM_TIMER_MAX_PULSE_LENGTH
	            - PPM_TIMER_MIN_PULSE_LENGTH)+ PPM_TIMER_MIN_PULSE_LENGTH;
	__HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_3, temp);
	// Motor 4
	temp = status->motorValues[3] * (PPM_TIMER_MAX_PULSE_LENGTH
	            - PPM_TIMER_MIN_PULSE_LENGTH)+ PPM_TIMER_MIN_PULSE_LENGTH;
	__HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_4, temp);

	// Testen ob sich Motoren korrekt Kalibrieren
	for (uint32_t i = 0; i < PPM_SETUP_TIME; i++) {
		__asm__("nop");
	}
}

void PPMGenerator::disableMotors() {

    // PPM-Timer stoppen
    HAL_TIM_PWM_Stop(PPMGenerator_htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(PPMGenerator_htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(PPMGenerator_htim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Stop(PPMGenerator_htim, TIM_CHANNEL_4);

}

void PPMGenerator::kill() {

    reset();
    disableMotors();
    RESET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}
