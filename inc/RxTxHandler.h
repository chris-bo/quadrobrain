/*
 * RxTxHandler.h
 *
 *  Created on: Nov 22, 2017
 *      Author: chris
 */

#ifndef RXTXHANDLER_H_
#define RXTXHANDLER_H_

#include <Task.h>
#include <config.h>

class RxTxHandler: public Task {
public:
	RxTxHandler(Status* statusPtr, uint8_t defaultPrio);
	virtual ~RxTxHandler();

	void startRX();
	void sendTXBuffer(uint8_t byte_count);
	void loopback();
	void setLED(bool on);

	void reset();

	bool newDataReceived;

	uint8_t numberReceivedData;
	uint8_t RxBuffer[RXTX_BUFF_SIZE];
	uint8_t TxBuffer[RXTX_BUFF_SIZE];
};

#endif /* RXTXHANDLER_H_ */
