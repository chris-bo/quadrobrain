/*
 * GPS.cpp
 *
 *  Created on: Jul 10, 2015
 *      Author: bohni
 */

#include "GPS.h"

uint8_t NEO6_TX_buffer[NEO6_TX_BUFFER_SIZE];
uint8_t NEO6_RX_buffer[NEO6_RX_BUFFER_SIZE];

GPS::GPS(Status* _status, int8_t _defaultPriority, GPS_Data_t* _gpsData)
            : Task(_status, _defaultPriority) {

    gpsData = _gpsData;
    lastACK = {0,0,0};
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
    switch (NEO6_RX_buffer[2]) {
    case UBX_ACK: {
        /*decode ack-msg*/
        if (NEO6_RX_buffer[3] == UBX_ACK_ACK) {
            lastACK.ack = 1;
        } else if (NEO6_RX_buffer[3] == UBX_ACK_NAK) {
            lastACK.ack = 0;
        } else {
            /* Error */
            /* wrong protocol */
        }
        lastACK.classid = NEO6_RX_buffer[6];
        lastACK.msgid = NEO6_RX_buffer[7];
        break;
    }
    case UBX_NAV: {
        /*decode NAV */
        switch (NEO6_RX_buffer[3]) {
        case UBX_NAV_TIMEUTC:
            decodeUBX_NavTimeUtc();
            break;
        case UBX_NAV_SOL:
        case UBX_NAV_POSECEF:
        case UBX_NAV_VELECEF:
        case UBX_NAV_VELNED:
        case UBX_NAV_POSLLH:
        case UBX_NAV_DOP:
        case UBX_NAV_STATUS:
            decodeUBX_NAV();
            break;
        case UBX_NAV_SBAS:
            break;
        default:
            /* Error */
            /* wrong protocol */
            break;
        }

        break;
    }
    case UBX_CFG: {
        /* Decode Configurations */
        break;
    }
    default:{
        /* Error */
        /* wrong protocol */

    }
    }
}

void GPS::decodeUBX_NAV() {
    /*Decode Navigation Data
         * UBX_NAV_POSECEF
         * UBX_NAV_VELECEF
         * UBX_NAV_VELNED
         * UBX_NAV_POSLLH
         * UBX_NAV_SOL
         * UBX_NAV_STATUS
         * UBX_NAV_DOP
         */
        gpsData->iTOW = gps_rx_buffer_32offset(0);

        switch (NEO6_RX_buffer[3]) {

        case UBX_NAV_POSECEF:
            gpsData->ecef_data.x = gps_rx_buffer_32offset(4);
            gpsData->ecef_data.y = gps_rx_buffer_32offset(8);
            gpsData->ecef_data.z = gps_rx_buffer_32offset(12);
            gpsData->ecef_data.pAcc = gps_rx_buffer_32offset(16);
            break;
        case UBX_NAV_VELECEF:
            gpsData->ecef_data.vx = gps_rx_buffer_32offset(4);
            gpsData->ecef_data.vy = gps_rx_buffer_32offset(8);
            gpsData->ecef_data.vz = gps_rx_buffer_32offset(12);
            gpsData->ecef_data.sAcc = gps_rx_buffer_32offset(16);
            break;
        case UBX_NAV_POSLLH:
            gpsData->llh_data.lon = gps_rx_buffer_32offset(4);
            gpsData->llh_data.lat = gps_rx_buffer_32offset(8);
            gpsData->llh_data.h = gps_rx_buffer_32offset(12);
            gpsData->llh_data.hMSL = gps_rx_buffer_32offset(16);
            gpsData->llh_data.hAcc = gps_rx_buffer_32offset(20);
            gpsData->llh_data.vAcc = gps_rx_buffer_32offset(24);
            break;
        case UBX_NAV_VELNED:
            gpsData->ned_data.vN = gps_rx_buffer_32offset(4);
            gpsData->ned_data.vE = gps_rx_buffer_32offset(8);
            gpsData->ned_data.vD = gps_rx_buffer_32offset(12);
            gpsData->ned_data.speed = gps_rx_buffer_32offset(16);
            gpsData->ned_data.gSpeed = gps_rx_buffer_32offset(20);
            gpsData->ned_data.heading = gps_rx_buffer_32offset(24);
            gpsData->ned_data.sAcc = gps_rx_buffer_32offset(28);
            gpsData->ned_data.cAcc = gps_rx_buffer_32offset(32);
            break;
        case UBX_NAV_SOL:
            gpsData->gpsWeek = (int16_t) gps_rx_buffer_16offset(8);
            gpsData->gpsFix = (uint8_t) gps_rx_buffer_offset(10);
            gpsData->flags = (uint8_t) gps_rx_buffer_offset(11);
            gpsData->ecef_data.x = gps_rx_buffer_32offset(12);
            gpsData->ecef_data.y = gps_rx_buffer_32offset(16);
            gpsData->ecef_data.z = gps_rx_buffer_32offset(20);
            gpsData->ecef_data.pAcc = gps_rx_buffer_32offset(24);
            gpsData->ecef_data.vx = gps_rx_buffer_32offset(28);
            gpsData->ecef_data.vy = gps_rx_buffer_32offset(32);
            gpsData->ecef_data.vz = gps_rx_buffer_32offset(36);
            gpsData->ecef_data.sAcc = gps_rx_buffer_32offset(40);

            gpsData->pDOP = (uint16_t) gps_rx_buffer_16offset(44);
            gpsData->numSV = (uint8_t) gps_rx_buffer_offset(47);
            break;
        case UBX_NAV_DOP:
            gpsData->pDOP = (uint16_t) gps_rx_buffer_16offset(4);
            gpsData->gDop = (uint16_t) gps_rx_buffer_16offset(6);
            gpsData->tDOP = (uint16_t) gps_rx_buffer_16offset(8);
            gpsData->vDOP = (uint16_t) gps_rx_buffer_16offset(10);
            gpsData->hDOP = (uint16_t) gps_rx_buffer_16offset(12);
            gpsData->nDOP = (uint16_t) gps_rx_buffer_16offset(14);
            gpsData->eDOP = (uint16_t) gps_rx_buffer_16offset(16);
            break;
        case UBX_NAV_STATUS:
            gpsData->gpsFix = (uint8_t) gps_rx_buffer_offset(4);
            gpsData->flags = (uint8_t) gps_rx_buffer_offset(5);
            gpsData->FixStatus = (uint8_t) gps_rx_buffer_offset(6);
            gpsData->flags2 = (uint8_t) gps_rx_buffer_offset(7);
            gpsData->ttff = gps_rx_buffer_32offset(8);
            gpsData->msss = gps_rx_buffer_32offset(12);
            break;
        default:
            /* Error */
            /* wrong protocol */
            break;
        }
}

void GPS::decodeUBX_NavTimeUtc() {
    gpsData->iTOW = gps_rx_buffer_32offset(0);
    gpsData->date.year = (uint16_t) gps_rx_buffer_16offset(12);
    gpsData->date.month = gps_rx_buffer_offset(14);
    gpsData->date.day = gps_rx_buffer_offset(15);
    gpsData->time.hours = gps_rx_buffer_offset(16);
    gpsData->time.minutes = gps_rx_buffer_offset(17);
    gpsData->time.seconds = gps_rx_buffer_offset(18);
    gpsData->time.validity = gps_rx_buffer_offset(19);
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
    /* returns length with header and checksum*/
    switch (classid) {
    case UBX_NAV:
        switch (msgid) {
        case UBX_NAV_SOL:
            return UBX_MSG_LENGTH_NAV_SOL;
            break;
        case UBX_NAV_POSLLH:
            return UBX_MSG_LENGTH_NAV_POSLLH;
            break;
        case UBX_NAV_POSECEF:
            return UBX_MSG_LENGTH_NAV_POSECEF;
            break;
        case UBX_NAV_VELNED:
            return UBX_MSG_LENGTH_NAV_VELNED;
            break;
        case UBX_NAV_VELECEF:
            return UBX_MSG_LENGTH_NAV_VELECEF;
            break;
        case UBX_NAV_TIMEUTC:
            return UBX_MSG_LENGTH_NAV_TIMEUTC;
            break;
        case UBX_NAV_STATUS:
            return UBX_MSG_LENGTH_NAV_TIMEUTC;
            break;
        case UBX_NAV_DOP:
            return UBX_MSG_LENGTH_NAV_TIMEUTC;
            break;
        default:
            return 0;
        }
        break;
    case UBX_ACK:
        return UBX_MSG_LENGTH_ACK;
        break;
    default:
        return 0;
        break;
    }
    return 0;
}
