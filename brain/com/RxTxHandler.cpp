/*
 * RxTxHandler.cpp
 *
 *  Created on: Nov 22, 2017
 *      Author: chris
 */

#include "RxTxHandler.h"

RxTxHandler::RxTxHandler(Status* statusPtr, uint8_t defaultPrio,
		DiscoveryLEDs* _ledctrl, Led_TypeDef _led) :
		Task(statusPtr, defaultPrio) {

	ledctrl = _ledctrl;
	led = _led;
	numberReceivedData = NULL;
	RxBuffer = NULL;
	TxBuffer = NULL;
}

void RxTxHandler::sendTXBuffer(uint16_t byte_count) {
	/*
	 * send data routine */
}

void RxTxHandler::initializeBuffers(uint8_t* RxBuf, uint8_t* TxBuf,
		uint16_t* numberReceived) {
	/* needed to setup buffers */
	RxBuffer = RxBuf;
	TxBuffer = TxBuf;
	numberReceivedData = numberReceived;

	memset(TxBuffer, 0, RXTX_BUFF_SIZE);
	memset(RxBuffer, 0, RXTX_BUFF_SIZE);
	*numberReceivedData = 0;

}

void RxTxHandler::reset() {
	// todo flush  buffers reset transmit states
	memset(TxBuffer, 0, RXTX_BUFF_SIZE);
	memset(RxBuffer, 0, RXTX_BUFF_SIZE);
}

void RxTxHandler::startRX() {
	/*
	 * start reception data routine
	 * overwrite
	 * */

}

RxTxHandler::~RxTxHandler() {
}

void RxTxHandler::setLED(bool on) {

	if (on) {
		ledctrl->on(led);
	} else {
		ledctrl->off(led);
	}
}
