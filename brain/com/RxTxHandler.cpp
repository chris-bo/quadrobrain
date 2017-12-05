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
    rxtxBuffersize = 0;
    receptionComplete = true;

    /* padding variable */
    pad = 0;
}

void RxTxHandler::sendTXBuffer(uint16_t byte_count) {
    /*
     * send data routine
     * overwrite
     * */
}

void RxTxHandler::initializeBuffers(uint8_t* RxBuf, uint8_t* TxBuf,
        uint16_t* numberReceived, uint16_t rxtxBufsize) {
    /* needed to setup buffers */
    RxBuffer = RxBuf;
    TxBuffer = TxBuf;
    numberReceivedData = numberReceived;
    rxtxBuffersize = rxtxBufsize;
    memset(TxBuffer, 0, rxtxBuffersize);
    memset(RxBuffer, 0, rxtxBuffersize);
    *numberReceivedData = 0;

}

void RxTxHandler::reset() {
    /* flush  buffers reset transmit states */
    memset(TxBuffer, 0, rxtxBuffersize);
    memset(RxBuffer, 0, rxtxBuffersize);
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
