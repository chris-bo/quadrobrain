/*
 * DiscoveryLEDs.cpp
 *
 *  Created on: Jun 8, 2015
 *      Author: bohni
 */

#include "DiscoveryLEDs.h"

DiscoveryLEDs::DiscoveryLEDs(Status* statusPtr, uint8_t defaultPrio)
            : Task(statusPtr, defaultPrio) {
    for (uint8_t i = 0; i < 8; i++) {
        frequency[i] = 1;
        toggle[i] = (uint8_t) (500 / SCHEDULER_INTERVALL_ms);
        counter[i] = 0;
    }
}

DiscoveryLEDs::~DiscoveryLEDs() {
}

void DiscoveryLEDs::initialize() {
    BSP_LED_Init(LED3);
    BSP_LED_Init(LED4);
    BSP_LED_Init(LED5);
    BSP_LED_Init(LED6);
    BSP_LED_Init(LED7);
    BSP_LED_Init(LED8);
    BSP_LED_Init(LED9);
    BSP_LED_Init(LED10);

    SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}

void DiscoveryLEDs::update() {
    for (uint8_t i = 0; i < 8; i++) {
        if (frequency[i] > 0.001f) {
            counter[i]++;
            if (counter[i] == toggle[i]) {
                counter[i] = 0;
                BSP_LED_Toggle((Led_TypeDef) i);
            }
        }
    }
    resetPriority();
}

void DiscoveryLEDs::kill() {
    reset();
    off(ALL);
}

void DiscoveryLEDs::setFrequency(Led_TypeDef _led, float freq) {
    if (_led != ALL) {
        if (freq < 0.001f) {
            frequency[_led] = 0.001f;
        } else if (freq > 500 / SCHEDULER_INTERVALL_ms) {
            frequency[_led] = (500 / SCHEDULER_INTERVALL_ms);
        } else {
            frequency[_led] = freq;
        }
        toggle[_led] = (uint8_t) ((500 / SCHEDULER_INTERVALL_ms) / frequency[_led]
                    - 1);
    } else {
        for (uint8_t i = 0; i < 8; i++) {
            if (freq < 0.001f) {
                frequency[i] = 0.001f;
            } else if (freq > 500 / SCHEDULER_INTERVALL_ms) {
                frequency[i] = (500 / SCHEDULER_INTERVALL_ms);
            } else {
                frequency[i] = freq;
            }
            toggle[i] =
                        (uint8_t) ((500 / SCHEDULER_INTERVALL_ms) / frequency[i] - 1);
        }
    }
}

void DiscoveryLEDs::setOffset(Led_TypeDef _led, uint8_t percentage) {
    if (_led != ALL) {
        if (percentage > 200) {
            percentage = 200;
        }
        if (percentage > 100) {
            BSP_LED_Toggle(_led);
            percentage = percentage - 100;
        }
        if (percentage > 0) {
            percentage = percentage - 1;
        }
        uint32_t tmp = ((toggle[_led] * percentage) / 100);
        counter[_led] = (uint8_t) tmp;

    } else {
        for (uint8_t i = 0; i < 8; i++) {
            if (percentage > 200) {
                percentage = 200;
            }
            if (percentage > 100) {
                BSP_LED_Toggle((Led_TypeDef) i);
                percentage = percentage - 100;
            }
            if (percentage > 0) {
                percentage = percentage - 1;
            }
            uint32_t tmp = ((toggle[i] * percentage) / 100);
            counter[i] = (uint8_t) tmp;
        }
    }
}

void DiscoveryLEDs::on(Led_TypeDef _led) {
    if (_led != ALL) {
        frequency[_led] = 0;
        BSP_LED_On(_led);
    } else {
        for (uint8_t i = 0; i < 8; i++) {
            frequency[i] = 0;
            BSP_LED_On((Led_TypeDef) i);
        }
    }
}

void DiscoveryLEDs::off(Led_TypeDef _led) {
    if (_led != ALL) {
        frequency[_led] = 0;
        BSP_LED_Off(_led);
    } else {
        for (uint8_t i = 0; i < 8; i++) {
            frequency[i] = 0;
            BSP_LED_Off((Led_TypeDef) i);
        }
    }
}
