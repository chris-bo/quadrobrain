/*
 * AkkuMonitor.h
 *
 *  Created on: Apr 25, 2015
 *      Author: bohni
 */

#ifndef AKKUMONITOR_H_
#define AKKUMONITOR_H_

#include "Task.h"
#include "config.h"

/* ADC in : PA1 */

#define VOLTAGE_DIVIDER_RATIO 		1.0f
#define MEASUREMENT_FREQUENCY		1  // Hz

class AkkuMonitor: public Task {
public:
	AkkuMonitor(Status* statusPtr, uint8_t defaultPrio, ADC_HandleTypeDef* hadc);
	virtual ~AkkuMonitor();

	void initialize();
	void update();

private:
	ADC_HandleTypeDef* akkumonitor_adc;
	uint16_t counter;


};

#endif /* AKKUMONITOR_H_ */
