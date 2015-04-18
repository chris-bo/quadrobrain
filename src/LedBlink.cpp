/*
 * LedBlink.cpp
 *
 *  Created on: Dec 16, 2014
 *      Author: bohni
 */

#include "LedBlink.h"

LedBlink::LedBlink(Status* statusPtr, uint8_t defaultPrio) :
		Task(statusPtr, defaultPrio) {
	frequency = 1;
	toggle = (uint8_t) (500 / SCHEDULER_INTERVALL_ms);
	priority = -1;
	counter = 0;
	led = LED3;
}

LedBlink::~LedBlink() {
}

void LedBlink::update() {
	if (frequency > 0.001f) {
		counter++;
		if (counter == toggle) {
			counter = 0;
			BSP_LED_Toggle(led);
		}
	}
	resetPriority();
}

void LedBlink::setLED(Led_TypeDef _led) {
	led = _led;
	BSP_LED_Init(led);
	priority = defaultPriority;
	SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
	setFrequency(1);
	setOffset(0);

}

void LedBlink::setFrequency(float freq) {
	if (freq < 0.001f) {
		frequency = 0.001f;
	} else if (freq > 500 / SCHEDULER_INTERVALL_ms) {
		frequency = (500 / SCHEDULER_INTERVALL_ms);
	} else {
		frequency = freq;
	}
	toggle = (uint8_t) ((500 / SCHEDULER_INTERVALL_ms) / frequency - 1);
}

void LedBlink::setOffset(uint8_t percentage) {
	if (percentage > 200) {
		percentage = 200;
	}
	if (percentage > 100) {
		BSP_LED_Toggle(led);
		percentage = percentage - 100;
	}
	if (percentage > 0) {
		percentage = percentage - 1;
	}
	uint32_t tmp = ((toggle * percentage) / 100);
	counter = (uint8_t) tmp;

}

void LedBlink::on() {
	frequency = 0;
	BSP_LED_On(led);
}

void LedBlink::off() {
	frequency = 0;
	BSP_LED_Off(led);
}
