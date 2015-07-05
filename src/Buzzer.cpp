/*
 * Buzzer.cpp
 *
 *  Created on: 26.04.2015
 *      Author: Jonas
 */

#include "Buzzer.h"

Buzzer::Buzzer(Status* statusPtr, uint8_t defaultPrio, TIM_HandleTypeDef* htim) :
		Task(statusPtr, defaultPrio) {
	Buzzer_htim = htim;
	elapsedLengthBuzzer1 = 0;
	toneLengthBuzzer1 = 0;
	elapsedLengthBuzzer2 = 0;
	toneLengthBuzzer2 = 0;
}

void Buzzer::update() {
	// check buzzer 1
	if( elapsedLengthBuzzer1 == 0 && toneLengthBuzzer1 != 0 ) {
		// start pwm timer
		HAL_TIM_PWM_Start( Buzzer_htim, TIM_CHANNEL_1 );
        elapsedLengthBuzzer1 = 1;
	} else if (elapsedLengthBuzzer1 == toneLengthBuzzer1) {
		// stop pwm timer
		HAL_TIM_PWM_Stop( Buzzer_htim, TIM_CHANNEL_1 );
		// reset lengths
		elapsedLengthBuzzer1 = 0;
		toneLengthBuzzer1 = 0;
		// reset buzzer status
		status->buzzer1Busy = false;
	} else {
		// go one timestep forward
		elapsedLengthBuzzer1++;
	}

	// check buzzer 2
		if( elapsedLengthBuzzer2 == 0 && toneLengthBuzzer2 != 0 ) {
			// start pwm timer
			HAL_TIM_PWM_Start( Buzzer_htim, TIM_CHANNEL_2 );
			elapsedLengthBuzzer2 = 1;
		} else if (elapsedLengthBuzzer2 == toneLengthBuzzer2) {
			// stop pwm timer
			HAL_TIM_PWM_Stop( Buzzer_htim, TIM_CHANNEL_2 );
			// reset lengths
			elapsedLengthBuzzer2 = 0;
			toneLengthBuzzer2 = 0;
			// reset buzzer status
			status->buzzer2Busy = false;
		} else {
			// go one timestep forward
			elapsedLengthBuzzer2++;
		}
}

void Buzzer::playToneOnBuzzer1(float frequency, uint16_t length) {
	// set lengts
	elapsedLengthBuzzer1 = 0;
	toneLengthBuzzer1 = length;
	// set pwm timer
	uint16_t temp = calculateReloadValue( frequency );
	__HAL_TIM_SetAutoreload( Buzzer_htim, temp );
	// TODO: evtl. Lautst�rkenregelung �ber Pulsl�nge
	__HAL_TIM_SetCompare( Buzzer_htim, TIM_CHANNEL_1, temp / 2 );

	// set buzzer as busy
	status->buzzer1Busy = true;
}

void Buzzer::playToneOnBuzzer2(float frequency, uint16_t length) {
	// set lengts
	elapsedLengthBuzzer2 = 0;
	toneLengthBuzzer2 = length;
	// set pwm timer
	uint16_t temp = calculateReloadValue( frequency );
	__HAL_TIM_SetAutoreload( Buzzer_htim, temp );
	__HAL_TIM_SetCompare( Buzzer_htim, TIM_CHANNEL_2, temp / 2 );
	// set buzzer as busy
	status->buzzer2Busy = true;
}

uint16_t Buzzer::calculateReloadValue( float frequency ) {
	// nreload = 72Mhz / nprescaler / frequency
    uint32_t tmp=((HAL_RCC_GetSysClockFreq() / (BUZZER_PRESCALER +1 )) / frequency);
	return (uint16_t)(tmp);
}

Buzzer::~Buzzer() {
	// TODO: Geh dich erschiessen
}

