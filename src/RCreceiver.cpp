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
	RCreceiverFlags = 0;
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

	/* initialize timer and interrupts */
	HAL_TIM_Base_MspInit(RCreceiver_htim);

}

RCreceiver::~RCreceiver() {
}

void RCreceiver::update() {

	if (GET_FLAG(RCreceiverFlags, RC_RECEIVER_FLAG_SEQUENCE_COMPLETE)) {
		computeValues();
	} else if (GET_FLAG(RCreceiverFlags, RC_RECEIVER_FLAG_SYNC_LOST)) {
		sync();
	}
	priority = defaultPriority;
}

//void RCreceiver::lowLevelInit() {
//
//	GPIO_InitTypeDef GPIO_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
//	TIM_ICInitTypeDef TIM_ICInitStructure;
//	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
//
//	/* TIM clock enable */
//	RCC_APB1PeriphClockCmd(RC_RECEIVER_TIMER_CLOCK, ENABLE);
//
//	/* GPIO clock enable */
//	RCC_AHBPeriphClockCmd(RC_RECEIVER_GPIO_PORT_CLOCK, ENABLE);
//
//	/*  pin configuration */
//	GPIO_InitStructure.GPIO_Pin = RC_RECEIVER_GPIO_PIN;
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	GPIO_Init(RC_RECEIVER_GPIO_PORT, &GPIO_InitStructure);
//
//	/* Connect TIM pin to AF */
//	GPIO_PinAFConfig(RC_RECEIVER_GPIO_PORT, RC_RECEIVER_GPIO_PIN_SOURCE,
//	RC_RECEIVER_GPIO_PIN_AF);
//
//	/* Enable the TIM global Interrupt */
//	NVIC_InitStructure.NVIC_IRQChannel = RC_RECEIVERTIMER_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStructure);
//
//	/* Time base configuration
//	 * Prescaler sets timer clock to 1 MHz -> timings in us
//	 * */
//	TIM_TimeBaseStructure.TIM_Period = RC_RECEIVER_OverrunTime_us;
//	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t) (SystemCoreClock / 1000000
//			- 1);
//	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
//	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
//	TIM_TimeBaseInit(SCHEDULER_TIMER, &TIM_TimeBaseStructure);
//
//	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3;
//	TIM_ICInitStructure.TIM_ICPolarity = INPUT_CAPURE_POLARITY;
//	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
//	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
//	TIM_ICInitStructure.TIM_ICFilter = 0x0;
//	TIM_ICInit(RC_RECEIVER_TIMER, &TIM_ICInitStructure);
//
//	/* TIM enable counter */
//	TIM_Cmd(RC_RECEIVER_TIMER, ENABLE);
//
//	/* Enable the CC3 Interrupt Request */
//	TIM_ITConfig(RC_RECEIVER_TIMER, TIM_IT_CC3, ENABLE);
//	/* Enable Overrun Interrupt */
//	TIM_ITConfig(RC_RECEIVER_TIMER, TIM_IT_Update, ENABLE);
//
//}

//void RCreceiver::RCreceiverIRQ() {
//	/* Interrupt request */
//	/*
//	 * if -
//	 * if
//	 *
//	 */
//
//
//}

void RCreceiver::computeValues() {
	/* raw timer output -> channel control values 0-100% */
	uint16_t tmp;
	for (uint8_t i = 0; i < RECEIVER_CHANNELS; i++) {
		tmp = (uint16_t) (rawReceiverValues[i] - RC_RECEIVER_MinHighTime_us
				- RC_RECEIVER_SyncTime_us);
		tmp = tmp / (RC_RECEIVER_MaxHighTime_us - RC_RECEIVER_MinHighTime_us);
		status->RCvalues[i] = (uint8_t) tmp;
	}

}

void RCreceiver::sync() {
	/* TODO: resync ppm to timer */
}

void RCreceiver::overrunIRQ() {
	/* Overrun interrupt
	 * overrun -> sync -> reset counter
	 */
	if (currentChannel == 8) {
		SET_FLAG(RCreceiverFlags, RC_RECEIVER_FLAG_SYNC_LOST);
		RESET_FLAG(RCreceiverFlags, RC_RECEIVER_FLAG_SYNC);
	} else {
		SET_FLAG(RCreceiverFlags, RC_RECEIVER_FLAG_SYNC);
		RESET_FLAG(RCreceiverFlags, RC_RECEIVER_FLAG_SYNC_LOST);
		currentChannel = 8;
	}
}

void RCreceiver::captureIRQ() {
	/* Input Capture Sequence
	 * input capture -> save value -> reset counter
	 * */
	if (currentChannel == 8) {
		/* after sync sequence */
		__HAL_TIM_SetCounter(RCreceiver_htim, 0);
		HAL_TIM_ReadCapturedValue(RCreceiver_htim, RC_RECEIVER_INPUT_CHANNEL);
		currentChannel = 0;
		RESET_FLAG(RCreceiverFlags, RC_RECEIVER_FLAG_SEQUENCE_COMPLETE);
	} else {
		rawReceiverValues[currentChannel] =
				(uint16_t) HAL_TIM_ReadCapturedValue(RCreceiver_htim,
				RC_RECEIVER_INPUT_CHANNEL);
		__HAL_TIM_SetCounter(RCreceiver_htim, 0);
		if (currentChannel == 7) {
			/* all pulses detected and saved */
			/* waiting for overrun interrupt to resync */
			SET_FLAG(RCreceiverFlags, RC_RECEIVER_FLAG_SEQUENCE_COMPLETE);
			RESET_FLAG(RCreceiverFlags, RC_RECEIVER_FLAG_SYNC);
		} else {
			currentChannel++;
		}
	}

}
