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

#define RX_BUFFER_LENGTH        128
#define TX_BUFFER_LENGTH        64


class GPS: public Task {
public:
    GPS(Status* _status, int8_t _defaultPriority, GPS_Data_t* _gpsData);
    virtual ~GPS();

    void initialize();
    void update();
    void reset();
private:
    GPS_Data_t* gpsData;

    GPS_ACK_t lastACK;

    void generateUBXHeader(uint8_t msgClass, uint8_t msgID, uint16_t payload);
    void pollUBXMessage(uint8_t msgClass, uint8_t msgID);
    void appendUBXChecksumTX(uint8_t bufferPosition);
    void decodeRX();

    void decodeUBX_NAV();
    void decodeUBX_NavTimeUtc();

    uint8_t getUBXMsgLength(uint8_t classid, uint8_t msgid);


};

#endif /* GPS_H_ */
