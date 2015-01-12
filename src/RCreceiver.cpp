/*
 * RCreceiver.cpp
 *
 *  Created on: Dec 17, 2014
 *      Author: bohni
 */

#include "RCreceiver.h"

RCreceiver::RCreceiver(Status* statusPtr, uint8_t defaultPrio,
		TIM_HandleTypeDef* htim) :
		Task(statusPtr, defaultPrio) {
	currentChannel = 0;
	rawReceiverValues[0] = 0;
	rawReceiverValues[1] = 0;
	rawReceiverValues[2] = 0;
	rawReceiverValues[3] = 0;
	rawReceiverValues[4] = 0;
	rawReceiverValues[5] = 0;
	rawReceiverValues[6] = 0;
	rawReceiverValues[7] = 0;

	RCreceiver_htim = htim;

}

RCreceiver::~RCreceiver() {
}

void RCreceiver::update() {
	priority = defaultPriority;
	if (GET_FLAG(taskStatusFlags, RC_RECEIVER_FLAG_SYNC_LOST)) {
		/* todo: manage rc signal loss*/
		signalLostTime++;
		if (signalLostTime == 200){
			RESET_FLAG(taskStatusFlags,TASK_FLAG_ACTIVE);
			priority = -1;
		}
	} else {
		computeValues();
		signalLostTime = 0;

	}
}

void RCreceiver::computeValues() {
	/* raw timer output -> channel control values 0-100% */
	int16_t tmp;
	for (uint8_t i = 0; i < RECEIVER_CHANNELS; i++) {

		tmp = (int16_t) (rawReceiverValues[i] - RC_RECEIVER_MinHighTime_us);
		tmp = tmp
				/ ((RC_RECEIVER_MaxHighTime_us - RC_RECEIVER_MinHighTime_us)
						/ 100);
		if (tmp < 0) {
			tmp = 0;
		} else if (tmp > 100) {
			tmp = 100;
		}
		status->RCvalues[i] = (uint8_t) tmp;
	}
}

void RCreceiver::overrunIRQ() {
	/* Overrun interrupt
	 * overrun -> sync -> reset counter
	 */
	if (currentChannel == 8) {

	} else {
		SET_FLAG(taskStatusFlags, RC_RECEIVER_FLAG_SYNC_LOST);
		RESET_FLAG(taskStatusFlags, RC_RECEIVER_FLAG_SYNC);
		currentChannel = 8;
	}
}

void RCreceiver::captureIRQ() {
	/* Input Capture Sequence
	 * input capture -> save value -> reset counter
	 * */
	if (currentChannel == 8) {
		/* after sync sequence */
		__HAL_TIM_SetCounter(RCreceiver_htim, 0x00);
		HAL_TIM_ReadCapturedValue(RCreceiver_htim, RC_RECEIVER_INPUT_CHANNEL);
		currentChannel = 0;
		RESET_FLAG(taskStatusFlags, RC_RECEIVER_FLAG_SEQUENCE_COMPLETE);
	} else {
		rawReceiverValues[currentChannel] =
				(uint16_t) HAL_TIM_ReadCapturedValue(RCreceiver_htim,
				RC_RECEIVER_INPUT_CHANNEL);
		__HAL_TIM_SetCounter(RCreceiver_htim, 0);
		if (currentChannel == 7) {
			/* all pulses detected and saved */
			/* waiting for overrun interrupt to resync */
			SET_FLAG(taskStatusFlags, RC_RECEIVER_FLAG_SEQUENCE_COMPLETE);
			SET_FLAG(taskStatusFlags, RC_RECEIVER_FLAG_SYNC);
			RESET_FLAG(taskStatusFlags, RC_RECEIVER_FLAG_SYNC_LOST);
		}
		currentChannel++;

	}

}

void RCreceiver::initialize() {

	/* initialize timer and interrupts
	 * needs to be called after MX_TIM4_Init();
	 * */
	HAL_TIM_Base_MspInit(RCreceiver_htim);
	__HAL_TIM_ENABLE_IT(RCreceiver_htim, TIM_IT_UPDATE);
	HAL_TIM_IC_Start_IT(RCreceiver_htim, RC_RECEIVER_INPUT_CHANNEL);

	SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);

}
