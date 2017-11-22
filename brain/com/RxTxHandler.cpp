/*
 * RxTxHandler.cpp
 *
 *  Created on: Nov 22, 2017
 *      Author: chris
 */

#include "RxTxHandler.h"


RxTxHandler::RxTxHandler(Status* statusPtr, uint8_t defaultPrio)
:Task(statusPtr, defaultPrio) {


}

void RxTxHandler::sendTXBuffer(uint8_t byte_count) {
}

void RxTxHandler::reset() {
	// todo flush  buffers reset transmit states

}

void RxTxHandler::startRX() {
}

RxTxHandler::~RxTxHandler() {
}

void RxTxHandler::loopback() {
}

void RxTxHandler::setLED(bool on) {
}
