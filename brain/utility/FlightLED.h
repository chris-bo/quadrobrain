/*
 * FlightLED.h
 *
 *  Created on: Aug 24, 2015
 *      Author: bohni
 */

#ifndef FLIGHTLED_H_
#define FLIGHTLED_H_

#include <core/Task.h>
#include "spi.h"
#include "stm32f3xx_hal_spi.h"

typedef enum {
    off, on, toggle
} switch_option;

class FlightLED: public Task {
public:
    FlightLED(Status* statusPtr, uint8_t defaultPrio, SPI_HandleTypeDef* spi);
    virtual ~FlightLED();

    void update();
    void initialize();
    void kill();

    uint16_t getLEDpattern();
    void switchLED(uint8_t nr, switch_option s);
    void setLEDpattern(uint16_t pattern);
private:
    SPI_HandleTypeDef* ledSpi;
    uint16_t ledPattern;

    void updateShiftRegisters();
};

#endif /* FLIGHTLED_H_ */
