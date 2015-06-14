/*
 * RCreceiver.h
 *
 *  Created on: Dec 17, 2014
 *      Author: bohni
 */

#ifndef RCRECEIVER_H_
#define RCRECEIVER_H_

#include "stm32f3xx_hal.h"
#include "Task.h"
#include "config.h"

/*****************
 * Capture Pin: PB8
 * */

/* receiver Flags */
#define RC_RECEIVER_FLAG_SYNC					0x0100
#define RC_RECEIVER_FLAG_SEQUENCE_COMPLETE 		0x0200

#define RC_RECEIVER_FLAG_NO_SIGNAL		 		0x0400
#define RC_RECEIVER_FLAG_ERROR					0x8000

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
    uint8_t currentChannel;
    uint32_t signalLostTime;

};

#endif /* RCRECEIVER_H_ */
