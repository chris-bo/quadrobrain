/*
 * DiscoveryLEDs.h
 *
 *  Created on: Jun 8, 2015
 *      Author: bohni
 */

#ifndef DISCOVERYLEDS_H_
#define DISCOVERYLEDS_H_

#include "Task.h"
#include "config.h"
#include "stm32f3_discovery.h"

class DiscoveryLEDs: public Task {
public:
    DiscoveryLEDs(Status* statusPtr, uint8_t defaultPrio);
    virtual ~DiscoveryLEDs();

    void initialize();
    void update();
    void kill();

    void setFrequency(Led_TypeDef _led, float freq);
    void setOffset(Led_TypeDef _led, uint8_t percentage);
    void on(Led_TypeDef _led);
    void off(Led_TypeDef _led);
private:
    float frequency[8];
    uint8_t counter[8];
    uint8_t toggle[8];
};

#endif /* DISCOVERYLEDS_H_ */
