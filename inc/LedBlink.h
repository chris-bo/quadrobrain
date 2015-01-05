/*
 * LedBlink.h
 *
 *  Created on: Dec 16, 2014
 *      Author: bohni
 */

#ifndef LEDBLINK_H_
#define LEDBLINK_H_


#include "Task.h"
#include "config.h"
#include "stm32f3_discovery.h"

class LedBlink: public Task {
public:
	LedBlink(Status* statusPtr, uint8_t defaultPrio);
	virtual ~LedBlink();
	void update();
	void setLED(Led_TypeDef _led);
	void setFrequency( float freq);
	void setOffset(uint8_t percentage);

private:
	float frequency;
	uint8_t counter;
	uint8_t toggle;
	Led_TypeDef led;

};

#endif /* LEDBLINK_H_ */
