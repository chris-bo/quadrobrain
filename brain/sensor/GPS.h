/*
 * GPS.h
 *
 *  Created on: Jul 10, 2015
 *      Author: bohni
 */

#ifndef GPS_H_
#define GPS_H_

#include <core/Task.h>
#include <sensor/GPS_defines.h>
#include "usart.h"

/**USART1 GPIO Configuration
 PC4     ------> USART1_TX
 PC5     ------> USART1_RX
 */

#define GPS_RX_BUFFER_LENGTH        64
#define GPS_TX_BUFFER_LENGTH        64

class GPS: public Task {
public:
    GPS(Status* _status, int8_t _defaultPriority, UART_HandleTypeDef* uart);
    virtual ~GPS();

    void initialize();
    void update();
    void kill();

    void RXCompleteCallback();
    void TXCompleteCallback();

private:
    UART_HandleTypeDef* gpsUart;

    GPS_Data_t* gpsData;

    void generateUBXHeader(uint8_t msgClass, uint8_t msgID, uint16_t payload);
    void pollUBXMessage(uint8_t msgClass, uint8_t msgID);
    void appendUBXChecksumTX(uint8_t bufferSize);
    void receive(uint8_t msgLength);

    uint8_t updateReceiverConfig(uint8_t configID, uint8_t* buffer,
            uint16_t payloadLength);
//    void getReceiverConfig(uint8_t configID, uint8_t* buffer, uint16_t payloadLength);
    void decodeRX();
    void decodeUBX_NAV();

    void setTransferState();

    uint8_t getUBXMsgLength(uint8_t classid, uint8_t msgid);

    GPS_ACK_t lastACK;

    /* dummy padding variable */
    int pad1 :8;

    bool fixOk :1;
    bool dGPSUsed :1;
    bool weekValid :1;
    bool itowValid :1;

    bool txRunning :1;
    bool rxRunning :1;
    bool decodeComplete :1;
    bool lastUpdateComplete :1;

    bool receptionError :1;
    bool transmissionError :1;
    bool timeoutError :1;
    bool error :1;

    /* transmission states */

    bool get_NAV_SOL :1;
    bool get_LLH :1;
    bool get_VELNED :1;
    bool get_ECEF :1;

    bool get_VELECEF :1;
    bool get_DATE :1;
    bool get_NAV_DOP :1;
    bool get_NAV_STATUS :1;

    bool get_DATA_BITMASK :1;

    bool handlerHalt :1;
    bool continousReception :1;

    /* dummy padding variable */
    int pad2 :9;
};

#endif /* GPS_H_ */
