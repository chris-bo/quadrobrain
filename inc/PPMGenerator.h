/*
 * PPMGenerator.h
 *
 *  Created on: 30.12.2014
 *      Author: Jonas
 */

#ifndef MOTORTASK_H_
#define MOTORTASK_H_

#include "config.h"
#include "Task.h"

class MotorTask: public Task {
public:
	MotorTask(Status* statusPtr, uint8_t defaultPrio);
	virtual ~MotorTask();
	void update();
	void lowLevelInit();

private:
	uint8_t throttle[];
	GPIO_InitTypeDef GPIO_InitStruct;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef TIM_OCInitStructure;
};

#endif /* MOTORTASK_H_ */
