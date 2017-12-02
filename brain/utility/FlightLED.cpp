/*
 * FlightLED.cpp
 *
 *  Created on: Aug 24, 2015
 *      Author: bohni
 */

#include "FlightLED.h"

FlightLED::FlightLED(Status* statusPtr, uint8_t defaultPrio,
        SPI_HandleTypeDef* spi) :
        Task(statusPtr, defaultPrio) {

    ledSpi = spi;
    ledPattern = 0;

}

FlightLED::~FlightLED() {

}
void FlightLED::update() {

}

void FlightLED::initialize() {

    /* switch all on*/
    ledPattern = 0xFFFF;
    updateShiftRegisters();
    HAL_Delay(300);

    /* switch off again*/
    ledPattern = 0x0000;
    updateShiftRegisters();

    SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}

void FlightLED::kill() {

    // switches LEDS of

    ledPattern = 0;
    updateShiftRegisters();
    this->reset();
    RESET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}

/* set led pattern
 * each bit represents one led
 */
void FlightLED::setLEDpattern(uint16_t pattern) {

    ledPattern = pattern;
    updateShiftRegisters();

}

void FlightLED::updateShiftRegisters() {

    uint8_t buf[2];
    buf[1] = (uint8_t) ((ledPattern >> 8) & 0xFF);
    buf[0] = (uint8_t) ((ledPattern) & 0xFF);

    /* start transmission*/
    HAL_SPI_Transmit_IT(ledSpi, buf, 2);

}

uint16_t FlightLED::getLEDpattern() {
    return ledPattern;
}

void FlightLED::switchLED(uint8_t nr, switch_option s) {

    if (s == off) {
        ledPattern &= ~(1 << nr);
    } else if (s == on) {
        ledPattern |= (1 << nr);
    } else if (s == toggle) {
        ledPattern ^= (1 << nr);
    }

}
