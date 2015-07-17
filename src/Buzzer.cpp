/*
 * Buzzer.cpp
 *
 *  Created on: 26.04.2015
 *      Author: Jonas
 */

#include "Buzzer.h"

Buzzer::Buzzer(Status* statusPtr, uint8_t defaultPrio, TIM_HandleTypeDef* htim) :
		Task(statusPtr, defaultPrio) {
	buzzerHtim = htim;
	elapsedLengthBuzzer = 0;
	toneLengthBuzzer = 0;
}

void Buzzer::update() {
	// check buzzer 1
	if( elapsedLengthBuzzer == 0 && toneLengthBuzzer != 0 ) {
		// start pwm timer
		HAL_TIM_PWM_Start( buzzerHtim, TIM_CHANNEL_1 );
        elapsedLengthBuzzer = 1;
	} else if (elapsedLengthBuzzer == toneLengthBuzzer) {
		// stop pwm timer
		HAL_TIM_PWM_Stop( buzzerHtim, TIM_CHANNEL_1 );
		// reset lengths
		elapsedLengthBuzzer = 0;
		toneLengthBuzzer = 0;
		// reset buzzer status
	    // todo buzzer queue: brauchen wir das dann noch?
		status->buzzer1Busy = false;
	} else {
		// go one timestep forward
		elapsedLengthBuzzer++;
	}
}

void Buzzer::playTone(float frequency, uint16_t length) {
	// set lengts
	elapsedLengthBuzzer = 0;
	toneLengthBuzzer = length;
	// set pwm timer
	uint16_t temp = calculateReloadValue( frequency );
	__HAL_TIM_SetAutoreload( buzzerHtim, temp );
	// TODO: evtl. Lautst�rkenregelung �ber Pulsl�nge
	__HAL_TIM_SetCompare( buzzerHtim, TIM_CHANNEL_1, temp / 2 );

	// set buzzer as busy
	// todo buzzer queue: brauchen wir das dann noch?
	status->buzzer1Busy = true;
}

uint16_t Buzzer::calculateReloadValue( float frequency ) {
	// nreload = 72Mhz / nprescaler / frequency
    uint32_t tmp=((HAL_RCC_GetSysClockFreq() / (BUZZER_PRESCALER +1 )) / frequency);
	return (uint16_t)(tmp);
}

Buzzer::~Buzzer() {

}

