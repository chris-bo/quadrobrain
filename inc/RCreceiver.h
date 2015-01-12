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

/* receiver Flags */
#define RC_RECEIVER_FLAG_SYNC					0x0100
#define RC_RECEIVER_FLAG_SEQUENCE_COMPLETE 		0x0200

#define RC_RECEIVER_FLAG_SYNC_LOST 		 		0x0400
#define RC_RECEIVER_FLAG_ERROR					0x8000

/* number of stored rec channels */
#define RECEIVER_CHANNELS						7

class RCreceiver: public Task {
public:
	RCreceiver(Status* statusPtr, uint8_t defaultPrio, TIM_HandleTypeDef* htim);
	virtual ~RCreceiver();
	void update();
	void overrunIRQ();
	void captureIRQ();
	void initialize();

private:
	void computeValues();

	TIM_HandleTypeDef* RCreceiver_htim;
	uint16_t rawReceiverValues[8];

	uint8_t currentChannel;
	uint8_t signalLostTime;

};

#endif /* RCRECEIVER_H_ */
