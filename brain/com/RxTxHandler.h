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
            uint16_t* numberReceived, uint16_t rxtxBufsize);

    virtual void startRX();
    virtual void sendTXBuffer(uint16_t byte_count);
    virtual void setLED(bool on);

    virtual void reset();

    DiscoveryLEDs* ledctrl;

    /* pointer to low level driver buffer*/
    uint16_t* numberReceivedData;
    uint8_t* RxBuffer;
    uint8_t* TxBuffer;
    uint16_t rxtxBuffersize;

    bool receptionComplete :1;

    /* dummy padding variable */
    int pad :7;

    Led_TypeDef led;
};

#endif /* RXTXHANDLER_H_ */
