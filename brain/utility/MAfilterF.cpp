/*
 * MAfilterF.cpp
 *
 *  Created on: Jun 14, 2015
 *      Author: bohni
 */

#include "MAfilterF.h"

MAfilterF::MAfilterF(Status* statusPtr, int8_t defaultPrio, float* _input,
            float* _output, uint16_t _buffersize)
: Task(statusPtr, defaultPrio) {

    counter = 0;
    input = _input;
    output = _output;
    buffersize = _buffersize;
    buffer = new float[buffersize];

    for (uint16_t i = 0; i < buffersize; i++) {
        buffer[i] = 0;
    }

}

MAfilterF::~MAfilterF() {

}

void MAfilterF::update() {

    buffer[counter] = *input;
    float temp = 0.0f;
    for (uint16_t i = 0; i < buffersize; i++) {
        temp += buffer[i];
    }

    *output = temp / buffersize;

    counter++;
    if (counter == buffersize) {
        counter = 0;
    }
}

void MAfilterF::kill() {
    reset();
    counter = 0;
    for (uint16_t i = 0; i < buffersize; i++) {
        buffer[i] = 0;
    }
}
