/*
 * RxTxHandler.h
 *
 *  Created on: Nov 22, 2017
 *      Author: chris
 */

#ifndef RXTXHANDLER_H_
#define RXTXHANDLER_H_

#include <core/Task.h>
#include <utility/DiscoveryLEDs.h>
#include <string.h>

class RxTxHandler: public Task {
public:
	RxTxHandler(Status* statusPtr, uint8_t defaultPrio, DiscoveryLEDs* _ledctrl,
			Led_TypeDef _led);
	virtual ~RxTxHandler();

	void initializeBuffers(uint8_t* RxBuf, uint8_t* TxBuf,
			uint16_t* numberReceived);

	virtual void startRX();
	virtual void sendTXBuffer(uint16_t byte_count);
	virtual void setLED(bool on);

	virtual void reset();

	bool newDataReceived;

	Led_TypeDef led;
	DiscoveryLEDs* ledctrl;

	uint16_t* numberReceivedData;
	uint8_t* RxBuffer;
	uint8_t* TxBuffer;

};

#endif /* RXTXHANDLER_H_ */
