/*
 * GPS.cpp
 *
 *  Created on: Jul 10, 2015
 *      Author: bohni
 */

#include "GPS.h"

uint8_t GPS_TX_buffer[GPS_TX_BUFFER_LENGTH];
uint8_t GPS_RX_buffer[GPS_RX_BUFFER_LENGTH];

GPS::GPS(Status* _status, int8_t _defaultPriority, UART_HandleTypeDef* uart)
            : Task(_status, _defaultPriority) {

    gpsData = &status->gpsData;
    gpsUart = uart;
    lastACK = {0,0,0};
    transferState = 0;
}

GPS::~GPS() {
}

void GPS::initialize() {

    // todo neo6 initialization routine

    SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}

void GPS::update() {
    /* start polling and monitor flags */

    if (!GET_FLAG(transferState, GPS_COM_FLAG_RX_RUNNING)) {
        /* previous communication was completed
         * -> poll next msg
         * */

        if (GET_FLAG(transferState, GPS_GET_DATE)) {
            /* POLL NAV_TIMEUTC */
            pollUBXMessage(UBX_NAV, UBX_NAV_TIMEUTC);
        } else if (GET_FLAG(transferState, GPS_GET_ECEF)) {
            /* POLL UBX_NAV_POSECEF */
            pollUBXMessage(UBX_NAV, UBX_NAV_POSECEF);
        } else if (GET_FLAG(transferState, GPS_GET_VELECEF)) {
            /* POLL UBX_NAV_VELECEF */
            pollUBXMessage(UBX_NAV, UBX_NAV_VELECEF);
        } else if (GET_FLAG(transferState, GPS_GET_LLH)) {
            /* POLL UBX_NAV_POSLLH */
            pollUBXMessage(UBX_NAV, UBX_NAV_POSLLH);
        } else if (GET_FLAG(transferState, GPS_GET_VELNED)) {
            /* POLL UBX_NAV_VELNED */
            pollUBXMessage(UBX_NAV, UBX_NAV_VELNED);
        } else if (GET_FLAG(transferState, GPS_GET_NAV_SOL)) {
            /* POLL UBX_NAV_SOL */
            pollUBXMessage(UBX_NAV, UBX_NAV_SOL);
        } else if (GET_FLAG(transferState, GPS_GET_NAV_DOP)) {
            /* POLL UBX_NAV_DOP */
            pollUBXMessage(UBX_NAV, UBX_NAV_DOP);
        } else if (GET_FLAG(transferState, GPS_GET_NAV_STATUS)) {
            /* POLL UBX_NAV_STATUS */
            pollUBXMessage(UBX_NAV, UBX_NAV_STATUS);
        } else {
            /* current data retrieving cycle finished
             * restart
             */
            if (GET_FLAG(transferState, GPS_CONTINUOUS_REC)) {
                setTransferState();
            }
        }
    }
    // TODO GPS::update() check errors

    resetPriority();
}

void GPS::pollUBXMessage(uint8_t msgClass, uint8_t msgID) {

    /* Generate Header  */
    generateUBXHeader(msgClass, msgID, 0);

    /* Generate Checksum*/
    appendUBXChecksumTX(6);

    /* set tx flag and start dma */
    SET_FLAG(transferState, GPS_COM_FLAG_TX_RUNNING);
    HAL_UART_Transmit_DMA(gpsUart, GPS_TX_buffer, 8);

    /*get msg length with header and checksum*/
    uint8_t msg_length = getUBXMsgLength(msgClass, msgID);
    /* Start Receiver */
    receive(msg_length);

}

void GPS::appendUBXChecksumTX(uint8_t bufferSize) {
    /*
     * CK_A = 0, CK_B = 0
     For(I=0;I<N;I++)
     {
     CK_A = CK_A + Buffer[I]
     CK_B = CK_B + CK_A
     }
     */
    uint32_t tmp_a = 0;
    uint32_t tmp_b = 0;

    for (uint8_t i = 2; i < bufferSize; i++) {
        tmp_a = (tmp_a + GPS_TX_buffer[i]);
        tmp_b = (tmp_b + tmp_a);

    }
    GPS_TX_buffer[bufferSize] = (tmp_a & 0xFF);
    GPS_TX_buffer[bufferSize + 1] = (tmp_b & 0xFF);
}

void GPS::decodeRX() {

    switch (GPS_RX_buffer[2]) {
        case UBX_ACK: {
            /*decode ack-msg*/
            if (GPS_RX_buffer[3] == UBX_ACK_ACK) {
                lastACK.ack = 1;
            } else if (GPS_RX_buffer[3] == UBX_ACK_NAK) {
                lastACK.ack = 0;
            } else {
                /* Error */
                /* wrong protocol */
            }
            lastACK.classid = GPS_RX_buffer[6];
            lastACK.msgid = GPS_RX_buffer[7];
            break;
        }
        case UBX_NAV: {
            /*decode NAV */
            switch (GPS_RX_buffer[3]) {
                case UBX_NAV_TIMEUTC:
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
        default: {
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
     * UBS_NAV_TIMEUTC
     *
     * resets transferState Flags
     */
    gpsData->iTOW = gps_rx_buffer_32offset(0);

    switch (GPS_RX_buffer[3]) {

        case UBX_NAV_POSECEF:
            gpsData->ecef_data.x = gps_rx_buffer_32offset(4);
            gpsData->ecef_data.y = gps_rx_buffer_32offset(8);
            gpsData->ecef_data.z = gps_rx_buffer_32offset(12);
            gpsData->ecef_data.pAcc = gps_rx_buffer_32offset(16);
            RESET_FLAG(transferState, GPS_GET_ECEF);
            break;
        case UBX_NAV_VELECEF:
            gpsData->ecef_data.vx = gps_rx_buffer_32offset(4);
            gpsData->ecef_data.vy = gps_rx_buffer_32offset(8);
            gpsData->ecef_data.vz = gps_rx_buffer_32offset(12);
            gpsData->ecef_data.sAcc = gps_rx_buffer_32offset(16);
            RESET_FLAG(transferState, GPS_GET_VELECEF);
            break;
        case UBX_NAV_POSLLH:
            gpsData->llh_data.lon = gps_rx_buffer_32offset(4);
            gpsData->llh_data.lat = gps_rx_buffer_32offset(8);
            gpsData->llh_data.h = gps_rx_buffer_32offset(12);
            gpsData->llh_data.hMSL = gps_rx_buffer_32offset(16);
            gpsData->llh_data.hAcc = gps_rx_buffer_32offset(20);
            gpsData->llh_data.vAcc = gps_rx_buffer_32offset(24);
            RESET_FLAG(transferState, GPS_GET_LLH);
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
            RESET_FLAG(transferState, GPS_GET_VELNED);
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
            RESET_FLAG(transferState, GPS_GET_NAV_SOL);
            break;
        case UBX_NAV_DOP:
            gpsData->pDOP = (uint16_t) gps_rx_buffer_16offset(4);
            gpsData->gDop = (uint16_t) gps_rx_buffer_16offset(6);
            gpsData->tDOP = (uint16_t) gps_rx_buffer_16offset(8);
            gpsData->vDOP = (uint16_t) gps_rx_buffer_16offset(10);
            gpsData->hDOP = (uint16_t) gps_rx_buffer_16offset(12);
            gpsData->nDOP = (uint16_t) gps_rx_buffer_16offset(14);
            gpsData->eDOP = (uint16_t) gps_rx_buffer_16offset(16);
            RESET_FLAG(transferState, GPS_GET_NAV_DOP);
            break;
        case UBX_NAV_STATUS:
            gpsData->gpsFix = (uint8_t) gps_rx_buffer_offset(4);
            gpsData->flags = (uint8_t) gps_rx_buffer_offset(5);
            gpsData->FixStatus = (uint8_t) gps_rx_buffer_offset(6);
            gpsData->flags2 = (uint8_t) gps_rx_buffer_offset(7);
            gpsData->ttff = gps_rx_buffer_32offset(8);
            gpsData->msss = gps_rx_buffer_32offset(12);
            RESET_FLAG(transferState, GPS_GET_NAV_STATUS);
            break;
        case UBX_NAV_TIMEUTC:
            gpsData->date.year = (uint16_t) gps_rx_buffer_16offset(12);
            gpsData->date.month = gps_rx_buffer_offset(14);
            gpsData->date.day = gps_rx_buffer_offset(15);
            gpsData->time.hours = gps_rx_buffer_offset(16);
            gpsData->time.minutes = gps_rx_buffer_offset(17);
            gpsData->time.seconds = gps_rx_buffer_offset(18);
            gpsData->time.validity = gps_rx_buffer_offset(19);
            RESET_FLAG(transferState, GPS_GET_DATE);
            break;
        default:
            break;
    }
}

void GPS::generateUBXHeader(uint8_t msgClass, uint8_t msgID,
            uint16_t payloadLength) {

    GPS_TX_buffer[0] = UBX_HEADER_0;
    GPS_TX_buffer[1] = UBX_HEADER_1;
    GPS_TX_buffer[2] = msgClass;
    GPS_TX_buffer[3] = msgID;
    GPS_TX_buffer[4] = (uint8_t) (payloadLength & 0x00FF);
    GPS_TX_buffer[5] = (uint8_t) ((payloadLength & 0xFF00) >> 8);
}

void GPS::receive(uint8_t msgLenght) {

    SET_FLAG(transferState, GPS_COM_FLAG_RX_RUNNING);
    HAL_UART_Receive_DMA(gpsUart, GPS_RX_buffer, msgLenght);

}

void GPS::RXCompleteCallback() {
    /* RX finished
     * Clear RX running Flag to allow polling of next data frame
     * and decode buffer
     */
    RESET_FLAG(transferState, GPS_COM_FLAG_RX_RUNNING);
    decodeRX();
}

void GPS::TXCompleteCallback() {
    /* TX finished:
     *
     * Release TX Channel for next transmission
     * done by clearing tx running flag
     */
    RESET_FLAG(transferState, GPS_COM_FLAG_TX_RUNNING);
}

void GPS::setTransferState() {
    /* sets transferstate with config values*/

#ifdef POLL_NAV_TIMEUTC
    transferState |= GPS_GET_DATE;
#endif
#ifdef POLL_NAV_SOL
    transferState |= GPS_GET_NAV_SOL;
#endif
#ifdef POLL_NAV_POSECEV
    transferState |= GPS_GET_ECEV;
#endif
#ifdef POLL_NAV_POSLLH
    transferState |= GPS_GET_LLH;
#endif
#ifdef POLL_NAV_VELECEF
    transferState |= GPS_GET_VELECEF;
#endif
#ifdef POLL_NAV_VELNED
    transferState |= GPS_GET_VELNED;
#endif
#ifdef POLL_NAV_DOP
    transferState |= GPS_GET_NAV_DOP;
#endif
#ifdef POLL_NAV_STATUS
    transferState |= GPS_GET_NAV_STATUS;
#endif

#ifdef CONTINUOUS_RECEPTION
    transferState |= GPS_CONTINUOUS_REC;
#endif

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

void GPS::kill() {
    reset();
    lastACK = {0,0,0};
    transferState = 0;

    *gpsData = {};

    RESET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);

}
