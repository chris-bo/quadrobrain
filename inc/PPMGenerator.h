/*
 * PPMGenerator.h
 *
 *  Created on: 30.12.2014
 *      Author: Jonas
 */

#ifndef PPMGENERATOR_H_
#define PPMGENERATOR_H_

#include "config.h"
#include "Task.h"

#define THROTTLE_MIN 0.15f

class PPMGenerator: public Task {
public:
	PPMGenerator(Status* statusPtr, uint8_t defaultPrio, TIM_HandleTypeDef* htim, float* controllerValueX, float* controllerValueY);
	virtual ~PPMGenerator();
	void update();
	void initialize();

private:
	TIM_HandleTypeDef* PPMGenerator_htim;
	float* controllerValueX;
	float* controllerValueY;
};

#endif /* PPMGENERATOR_H_ */
