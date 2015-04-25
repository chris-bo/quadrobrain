/*
 * PPMGenerator.cpp
 *
 *  Created on: 30.12.2014
 *      Author: Jonas
 */

#include "PPMGenerator.h"

PPMGenerator::PPMGenerator(Status* statusPtr, uint8_t defaultPrio, TIM_HandleTypeDef* htim) :
Task(statusPtr, defaultPrio) {
	PPMGenerator_htim = htim;
}

PPMGenerator::~PPMGenerator() {

}

void PPMGenerator::update() {
	// F�r alle 4 Motoren prozentuale Throttle-Werte in Compare-Werte f�r den Timer wandeln
	// und anschlie�end dem Timer �bergeben
	// Motor 1
	uint16_t temp = status->motorValues[0] * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH) + PPM_TIMER_MIN_PULSE_LENGTH;
	__HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_1, temp);
	// Motor 2
	temp = status->motorValues[1] * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH) + PPM_TIMER_MIN_PULSE_LENGTH;
	__HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_2, temp);
	// Motor 3
	temp = status->motorValues[2] * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH) + PPM_TIMER_MIN_PULSE_LENGTH;
	__HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_3, temp);
	// Motor 4
	temp = status->motorValues[3] * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH) + PPM_TIMER_MIN_PULSE_LENGTH;
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
	uint16_t temp = status->motorValues[0] * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH) + PPM_TIMER_MIN_PULSE_LENGTH;
	__HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_1, temp);
	// Motor 2
	temp = status->motorValues[1] * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH) + PPM_TIMER_MIN_PULSE_LENGTH;
	__HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_2, temp);
	// Motor 3
	temp = status->motorValues[2] * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH) + PPM_TIMER_MIN_PULSE_LENGTH;
	__HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_3, temp);
	// Motor 4
	temp = status->motorValues[3] * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH) + PPM_TIMER_MIN_PULSE_LENGTH;
	__HAL_TIM_SetCompare(PPMGenerator_htim, TIM_CHANNEL_4, temp);

	for(uint32_t i = 0; i < PPM_SETUP_TIME; i++)
	{
		__asm__("nop");
	}
}
