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
	counter++;
	if (counter == toggle) {
		counter = 0;
		BSP_LED_Toggle(led);
	}
	priority = defaultPriority;
}

void LedBlink::setLED(Led_TypeDef _led) {
	led = _led;
	BSP_LED_Init(led);
	priority = defaultPriority;

}

void LedBlink::setFrequency(float freq) {
	if (freq < 0.001f) {
		freq = 0.001f;
	} else if (freq > 500 / SCHEDULER_INTERVALL_ms) {
		freq = (500 / SCHEDULER_INTERVALL_ms);
	}
	toggle = (uint8_t) ((500 / SCHEDULER_INTERVALL_ms) / freq - 1);
}

void LedBlink::setOffset(uint8_t percentage) {
	uint32_t tmp =((toggle * percentage) / 100);
	counter = (uint8_t) tmp;

}
