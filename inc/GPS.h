/*
 * GPS.h
 *
 *  Created on: Jul 10, 2015
 *      Author: bohni
 */

#ifndef GPS_H_
#define GPS_H_

#include "Task.h"
#include "GPS_defines.h"
#include "config.h"
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

    GPS_ACK_t lastACK;

    uint32_t transferState;

    void generateUBXHeader(uint8_t msgClass, uint8_t msgID, uint16_t payload);
    void pollUBXMessage(uint8_t msgClass, uint8_t msgID);
    void appendUBXChecksumTX(uint8_t bufferSize);
    void receive(uint8_t msgLength);

    void decodeRX();
    void decodeUBX_NAV();

    void setTransferState();

    uint8_t getUBXMsgLength(uint8_t classid, uint8_t msgid);

};

#endif /* GPS_H_ */
