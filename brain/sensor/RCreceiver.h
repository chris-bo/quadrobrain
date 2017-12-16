/*
 * RCreceiver.h
 *
 *  Created on: Dec 17, 2014
 *      Author: bohni
 */

#ifndef RCRECEIVER_H_
#define RCRECEIVER_H_

#include <core/Task.h>
#include "stm32f3xx_hal.h"

/*****************
 * Capture Pin: PB8
 * */
/* number of stored rec channels */
#define RECEIVER_CHANNELS						7

/* Channel Configuration:
 *
 * 1: right hor (roll)
 * 2: left vert	(throttle)
 * 3: right vert (nick)
 * 4: left hor (yaw)
 * 5: right switch
 * 6: linear control
 * 7: left switch
 * 8: ---
 *
 */

class RCreceiver: public Task {
public:
    RCreceiver(Status* statusPtr, uint8_t defaultPrio, TIM_HandleTypeDef* htim);
    virtual ~RCreceiver();
    void update();
    void initialize();
    void kill();
    void overrunIRQ();
    void captureIRQ();

private:
    void computeValues();

    TIM_HandleTypeDef* RCreceiver_htim;
    uint16_t rawReceiverValues[8];
    uint16_t rawRCvalues[RECEIVER_CHANNELS];
    uint16_t signalLostBuzzerCounter;
    uint32_t signalLostTime;
    uint8_t currentChannel;


    bool sync :1;
    bool sequenceComplete :1;
    bool noSignal :1;
    bool hadSignal :1;
    bool error :1;

    /* dummy padding variable */
    int pad :19;

};

#endif /* RCRECEIVER_H_ */
