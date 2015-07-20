/*
 * Buzzer.h
 *
 *  Created on: 26.04.2015
 *      Author: Jonas
 */

#ifndef BUZZER_H_
#define BUZZER_H_

#include <Task.h>


class Buzzer: public Task {
public:
	Buzzer(Status* statusPtr, uint8_t defaultPrio, TIM_HandleTypeDef* htim, Buzzer_Queue_t* _queue );
	void update();
	void playTone(float frequency, uint16_t length);
	virtual ~Buzzer();

private:
	TIM_HandleTypeDef* buzzerHtim;
    uint16_t toneLengthBuzzer;
    uint16_t elapsedLengthBuzzer;
    Buzzer_Queue_t* queue;
	uint16_t calculateReloadValue(float frequency);

	void playNextTone();

};

#endif /* BUZZER_H_ */
