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
#define RC_RECEIVER_FLAG_SYNC					0x01
#define RC_RECEIVER_FLAG_SEQUENCE_COMPLETE 		0x02

#define RC_RECEIVER_FLAG_SYNC_LOST 		 		0x80

/* number of stored rec channels */
#define RECEIVER_CHANNELS						5

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
	void sync();

	uint16_t rawReceiverValues[8];

	uint8_t RCreceiverFlags;
	uint8_t currentChannel;

	TIM_HandleTypeDef* RCreceiver_htim;

};

#endif /* RCRECEIVER_H_ */
