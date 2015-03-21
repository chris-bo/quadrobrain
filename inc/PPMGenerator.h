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

class PPMGenerator: public Task {
public:
	PPMGenerator(Status* statusPtr, uint8_t defaultPrio, TIM_HandleTypeDef* htim);
	virtual ~PPMGenerator();
	void update();
	void initialize();

private:
	TIM_HandleTypeDef* PPMGenerator_htim;
};

#endif /* PPMGENERATOR_H_ */
