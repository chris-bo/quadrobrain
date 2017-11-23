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
	newDataReceived = false;
}

void RxTxHandler::sendTXBuffer(uint16_t byte_count) {
	/*
	 * send data routine */
}

void RxTxHandler::initializeBuffers(uint8_t* RxBuf, uint8_t* TxBuf,
		uint8_t* numberReceived) {
	/* needed to setup buffers */
	RxBuffer = RxBuf;
	TxBuffer = TxBuf;
	numberReceivedData = numberReceived;

}

void RxTxHandler::reset() {
	// todo flush  buffers reset transmit states

}

void RxTxHandler::startRX() {
	/*
	 * start reception data routine */
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
