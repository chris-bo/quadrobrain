/*
 * MAfilterF.h
 *
 *  Created on: Jun 14, 2015
 *      Author: bohni
 */

#ifndef MAFILTERF_H_
#define MAFILTERF_H_

#include "Task.h"
#include "config.h"

class MAfilterF: public Task {
public:
    MAfilterF(Status* status, int8_t defaultPriority, float* input, float* output,
                uint16_t buffersize);
    virtual ~MAfilterF();
    void update();
    void kill();

private:
    float* input;
    float* output;

    float* buffer;

    uint16_t counter;
    uint16_t buffersize;
};

#endif /* MAFILTERF_H_ */
