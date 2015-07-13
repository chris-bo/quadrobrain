/*
 * GPS.cpp
 *
 *  Created on: Jul 10, 2015
 *      Author: bohni
 */

#include "GPS.h"

GPS::GPS(Status* _status, int8_t _defaultPriority, GPS_Data_t* _gpsData)
            : Task(_status, _defaultPriority) {

    gpsData = _gpsData;

}

GPS::~GPS() {
}

void GPS::initialize() {

    // todo neo6 initialization routine

    SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);

}

void GPS::update() {
}

void GPS::reset() {
}

void GPS::pollUBXMessage(uint8_t msgClass, uint8_t msgID) {
}

void GPS::appendUBXChecksumTX(uint8_t bufferPosition) {
}

void GPS::decodeRX() {
}

void GPS::decodeUBX_NAV() {
}

void GPS::decodeUBX_NavTimeUtc() {
}

void GPS::generateUBXHeader(uint8_t msgClass, uint8_t msgID,
            uint16_t payloadLength) {

    NEO6_TX_buffer[0] = UBX_HEADER_0;
    NEO6_TX_buffer[1] = UBX_HEADER_1;
    NEO6_TX_buffer[2] = msgClass;
    NEO6_TX_buffer[3] = msgID;
    NEO6_TX_buffer[4] = (uint8_t) (payloadLength & 0x00FF);
    NEO6_TX_buffer[5] = (uint8_t) ((payloadLength & 0xFF00) >> 8);
}

uint8_t GPS::getUBXMsgLength(uint8_t classid, uint8_t msgid) {
}
