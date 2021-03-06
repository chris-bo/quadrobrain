/*
 * GPS.cpp
 *
 *  Created on: Jul 10, 2015
 *      Author: bohni
 */

#include "GPS.h"

uint8_t GPS_TX_buffer[GPS_TX_BUFFER_LENGTH];
uint8_t GPS_RX_buffer[GPS_RX_BUFFER_LENGTH];

GPS::GPS(Status* _status, int8_t _defaultPriority, UART_HandleTypeDef* uart) :
        Task(_status, _defaultPriority) {

    gpsData = &status->gpsData;
    gpsUart = uart;
    lastACK = {0,0,0};

    fixOk = false;
    dGPSUsed = false;
    weekValid = false;
    itowValid = false;

    txRunning = false;
    rxRunning = false;
    decodeComplete = false;
    lastUpdateComplete = false;
    receptionError = false;
    transmissionError = false;
    timeoutError = false;
    error = false;

    get_NAV_SOL = false;
    get_LLH = false;
    get_VELNED = false;
    get_ECEF = false;

    get_VELECEF = false;
    get_DATE = false;
    get_NAV_DOP = false;
    get_NAV_STATUS = false;

    get_DATA_BITMASK = false;

    handlerHalt = true;
    continousReception = false;

    /* dummy padding variable */
    pad1 = 0;
    pad2 = 0;
}

GPS::~GPS() {
}

void GPS::initialize() {
// TODO GPS::initialize() : check if initializer works with standard config
    gpsUart->Init.BaudRate = 115200;
    HAL_UART_Init(gpsUart);

    /* UBX port settings */
    /* USART1
     * Protocol in : UBX NEMA RTCM
     * Protocol out: UBX
     * Baudrate : 115200
     */
    uint8_t PORT_Config_115200[] = { 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00,
            0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00,
            0x00, 0x00 };
#ifdef CONFIG_BAUD_9600_NO_NMEA
    /* UBX port settings */
    /* USART1
     * Protocol in : UBX NEMA RTCM
     * Protocol out: UBX
     * Baudrate : 9600
     */
    uint8_t PORT_Config_9600[] = {0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00,
        0x00, 0x80, 0x25, 0x00, 0x00, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00,
        0x00, 0x00};
#endif

    /* Check Baudrate by Port config -> ack?*/
    if (updateReceiverConfig(UBX_CFG_PRT, PORT_Config_115200, 20) == 0) {
        /* no ack -> needs fixing*/
        /* solution 1: reset uart and check again */

        /* wait if receiver is still transmitting */
        HAL_Delay(100);
        /* reset */
        gpsUart->Init.BaudRate = 115200;
        HAL_UART_Init(gpsUart);

        if (updateReceiverConfig(UBX_CFG_PRT, PORT_Config_115200, 20) == 0) {
            /* failed again */
            /* solution 2
             * try 9600 baud
             */
            /* wait if receiver is still transmitting */
            HAL_Delay(100);
            /* reset */
            gpsUart->Init.BaudRate = 9600;
            HAL_UART_Init(gpsUart);
            updateReceiverConfig(UBX_CFG_PRT, PORT_Config_115200, 20);

            if (!transmissionError) {
                transmissionError = true;
                /* recall initialization */
                this->initialize();
            } else {
                /* 2nd try failed */
                /* no gps */
                /* task will not be activated */
                return;
            }

        }
    }

    /* now port should be accepted */
#ifdef SAVE_PORT_SETTINGS_TO_BBR
    /* Save Settings for next startup*/
    /* Payload to save Portconfig to BBR(battery backed ram*/
    static uint8_t save_port_config[] = {0x00, 0x00, 0x00, 0x00, 0x01,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01};

    updateReceiverConfig(UBX_CFG_CFG, save_port_config, 13);
#endif

    /* Port Config accepted
     *  -> config receiver
     */
#ifdef USE_SBAS
    /* SBAS_config
     * enabled
     * test mode
     *
     * use SBAS in ranging correction and integrity
     * 3 channels
     * autoscan
     *
     */
    static uint8_t use_sbas_config[] = { 0x03, 0x07, 0x03, 0x00, 0x00, 0x00,
            0x00, 0x00 };
    updateReceiverConfig(UBX_CFG_SBAS, use_sbas_config, 8);
#endif

#ifdef INCREASE_MEASUREMENT_RATE
    static uint8_t inrease_measurement_rate[] = { 0xC8, 0x00, 0x01, 0x00, 0x01,
            0x00, };
    updateReceiverConfig(UBX_CFG_RATE, inrease_measurement_rate, 6);
#endif

#ifdef SAVE_ALL_SETTINGS_TO_BBR

    static uint8_t save_all_settings[] = { 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0D };
    updateReceiverConfig(UBX_CFG_RATE, save_all_settings, 13);
#endif
    setTransferState();
    taskActive = true;
}

void GPS::update() {
    /* start polling and monitor flags */

    if (!rxRunning) {
        /* previous communication was completed
         * -> poll next msg
         * */

        if (get_DATE) {
            /* POLL NAV_TIMEUTC */
            pollUBXMessage(UBX_NAV, UBX_NAV_TIMEUTC);
        } else if (get_ECEF) {
            /* POLL UBX_NAV_POSECEF */
            pollUBXMessage(UBX_NAV, UBX_NAV_POSECEF);
        } else if (get_VELECEF) {
            /* POLL UBX_NAV_VELECEF */
            pollUBXMessage(UBX_NAV, UBX_NAV_VELECEF);
        } else if (get_LLH) {
            /* POLL UBX_NAV_POSLLH */
            pollUBXMessage(UBX_NAV, UBX_NAV_POSLLH);
        } else if (get_VELNED) {
            /* POLL UBX_NAV_VELNED */
            pollUBXMessage(UBX_NAV, UBX_NAV_VELNED);
        } else if (get_NAV_SOL) {
            /* POLL UBX_NAV_SOL */
            pollUBXMessage(UBX_NAV, UBX_NAV_SOL);
        } else if (get_NAV_DOP) {
            /* POLL UBX_NAV_DOP */
            pollUBXMessage(UBX_NAV, UBX_NAV_DOP);
        } else if (get_NAV_STATUS) {
            /* POLL UBX_NAV_STATUS */
            pollUBXMessage(UBX_NAV, UBX_NAV_STATUS);
        } else {
            /* current data retrieving cycle finished
             * restart
             */
            if (continousReception) {
                setTransferState();
            }
        }
    }
    // TODO GPS::update() check errors

}

void GPS::pollUBXMessage(uint8_t msgClass, uint8_t msgID) {

    /* Generate Header  */
    generateUBXHeader(msgClass, msgID, 0);

    /* Generate Checksum*/
    appendUBXChecksumTX(6);

    /* set tx flag and start dma */
    txRunning = true;
    HAL_UART_Transmit_DMA(gpsUart, GPS_TX_buffer, 8);

    /* Start Receiver */
    receive(getUBXMsgLength(msgClass, msgID));

}
/* calculates ubx checksum
 * appends two bytes to the buffer
 * beginning with position "bufferSize"
 */
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
     * UBX_NAV_TIMEUTC
     * UBX_NAV_SBAS
     * resets transferState Flags
     */
    gpsData->iTOW = gps_rx_buffer_32offset(0);

    switch (GPS_RX_buffer[3]) {

    case UBX_NAV_POSECEF:
        gpsData->ecef_data.x = gps_rx_buffer_32offset(4);
        gpsData->ecef_data.y = gps_rx_buffer_32offset(8);
        gpsData->ecef_data.z = gps_rx_buffer_32offset(12);
        gpsData->ecef_data.pAcc = gps_rx_buffer_32offset(16);
        get_ECEF = false;
        break;
    case UBX_NAV_VELECEF:
        gpsData->ecef_data.vx = gps_rx_buffer_32offset(4);
        gpsData->ecef_data.vy = gps_rx_buffer_32offset(8);
        gpsData->ecef_data.vz = gps_rx_buffer_32offset(12);
        gpsData->ecef_data.sAcc = gps_rx_buffer_32offset(16);
        get_VELECEF = false;
        break;
    case UBX_NAV_POSLLH:
        gpsData->llh_data.lon = gps_rx_buffer_32offset(4);
        gpsData->llh_data.lat = gps_rx_buffer_32offset(8);
        gpsData->llh_data.h = gps_rx_buffer_32offset(12);
        gpsData->llh_data.hMSL = gps_rx_buffer_32offset(16);
        gpsData->llh_data.hAcc = gps_rx_buffer_32offset(20);
        gpsData->llh_data.vAcc = gps_rx_buffer_32offset(24);
        get_LLH = false;
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
        get_VELNED = false;
        break;
    case UBX_NAV_SOL:
        gpsData->gpsWeek = (int16_t) gps_rx_buffer_16offset(8);
        gpsData->gpsFix = (GPS_Fix_t) gps_rx_buffer_offset(10);
        gpsData->navStatusFlags = gps_rx_buffer_offset(11);
        gpsData->ecef_data.x = gps_rx_buffer_32offset(12);
        gpsData->ecef_data.y = gps_rx_buffer_32offset(16);
        gpsData->ecef_data.z = gps_rx_buffer_32offset(20);
        gpsData->ecef_data.pAcc = gps_rx_buffer_32offset(24);
        gpsData->ecef_data.vx = gps_rx_buffer_32offset(28);
        gpsData->ecef_data.vy = gps_rx_buffer_32offset(32);
        gpsData->ecef_data.vz = gps_rx_buffer_32offset(36);
        gpsData->ecef_data.sAcc = gps_rx_buffer_32offset(40);

        gpsData->dop.pDOP = (uint16_t) gps_rx_buffer_16offset(44);
        gpsData->numSV = (uint8_t) gps_rx_buffer_offset(47);
        get_NAV_SOL = false;
        break;
    case UBX_NAV_DOP:
        gpsData->dop.pDOP = (uint16_t) gps_rx_buffer_16offset(4);
        gpsData->dop.gDOP = (uint16_t) gps_rx_buffer_16offset(6);
        gpsData->dop.tDOP = (uint16_t) gps_rx_buffer_16offset(8);
        gpsData->dop.vDOP = (uint16_t) gps_rx_buffer_16offset(10);
        gpsData->dop.hDOP = (uint16_t) gps_rx_buffer_16offset(12);
        gpsData->dop.nDOP = (uint16_t) gps_rx_buffer_16offset(14);
        gpsData->dop.eDOP = (uint16_t) gps_rx_buffer_16offset(16);
        get_NAV_DOP = false;
        break;
    case UBX_NAV_STATUS:
        gpsData->gpsFix = (GPS_Fix_t) gps_rx_buffer_offset(4);
        gpsData->navStatusFlags = gps_rx_buffer_offset(5);
        gpsData->FixStatus = gps_rx_buffer_offset(6);
        gpsData->psmState = (PSM_State_t) gps_rx_buffer_offset(7);
        gpsData->ttff = gps_rx_buffer_32offset(8);
        gpsData->msss = gps_rx_buffer_32offset(12);
        get_NAV_STATUS = false;
        break;
    case UBX_NAV_TIMEUTC:
        gpsData->date.year = (uint16_t) gps_rx_buffer_16offset(12);
        gpsData->date.month = gps_rx_buffer_offset(14);
        gpsData->date.day = gps_rx_buffer_offset(15);
        gpsData->time.hours = gps_rx_buffer_offset(16);
        gpsData->time.minutes = gps_rx_buffer_offset(17);
        gpsData->time.seconds = gps_rx_buffer_offset(18);
        gpsData->time.validity = gps_rx_buffer_offset(19);
        get_DATE = false;
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

    HAL_UART_Receive_DMA(gpsUart, GPS_RX_buffer, msgLenght);
    rxRunning = true;
}

void GPS::RXCompleteCallback() {
    /* RX finished
     * Clear RX running Flag to allow polling of next data frame
     * and decode buffer
     */
    rxRunning = false;
    decodeRX();
}

void GPS::TXCompleteCallback() {
    /* TX finished:
     *
     * Release TX Channel for next transmission
     * done by clearing tx running flag
     */
    txRunning = false;
}

void GPS::setTransferState() {
    /* sets transferstate with config values*/

#ifdef POLL_NAV_TIMEUTC
    get_DATE = true;
#endif
#ifdef POLL_NAV_SOL
    get_NAV_SOL = true;
#endif
#ifdef POLL_NAV_POSECEF
    get_ECEF;
#endif
#ifdef POLL_NAV_POSLLH
    get_LLH = true;
#endif
#ifdef POLL_NAV_VELECEF
    get_VELECEF;
#endif
#ifdef POLL_NAV_VELNED
    get_VELNED = true;
#endif
#ifdef POLL_NAV_DOP
    get_NAV_DOP = true;
#endif
#ifdef POLL_NAV_STATUS
    get_NAV_STATUS = true;
#endif

#ifdef CONTINUOUS_RECEPTION
    continousReception = true;
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
            return UBX_MSG_LENGTH_NAV_STATUS;
            break;
        case UBX_NAV_DOP:
            return UBX_MSG_LENGTH_NAV_DOP;
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

    *gpsData = {};

    fixOk = false;
    dGPSUsed = false;
    weekValid = false;
    itowValid = false;

    txRunning = false;
    rxRunning = false;
    decodeComplete = false;
    lastUpdateComplete = false;
    receptionError = false;
    transmissionError = false;
    timeoutError = false;
    error = false;

    get_NAV_SOL = false;
    get_LLH = false;
    get_VELNED = false;
    get_ECEF = false;

    get_VELECEF = false;
    get_DATE = false;
    get_NAV_DOP = false;
    get_NAV_STATUS = false;

    get_DATA_BITMASK = false;

    handlerHalt = true;
    continousReception = false;

    taskActive = false;

}

uint8_t GPS::updateReceiverConfig(uint8_t configID, uint8_t* buffer,
        uint16_t payloadLength) {

    /* Generate Header and copy buffer to TX_buffer*/
    generateUBXHeader(UBX_CFG, configID, payloadLength);
    for (uint16_t i = 0; i < payloadLength; i++) {
        GPS_TX_buffer[i + 6] = buffer[i];
    }
    appendUBXChecksumTX((uint8_t) (payloadLength + 6));

    /* start transmission */
    HAL_UART_Transmit(gpsUart, GPS_TX_buffer, (uint8_t) (payloadLength + 8),
    GPS_UART_TIMEOUT);

    /* receive ack*/
    receive(UBX_MSG_LENGTH_ACK);
    uint32_t timeout = GPS_UART_TIMEOUT * 100;
    while (rxRunning && (timeout > 0)) {

        timeout--;
    }

    if (lastACK.ack == 1) {
        if (lastACK.classid == UBX_CFG) {
            if (lastACK.msgid == configID) {
                /* ACK SUCCESS*/
                /* clear last ack */
                lastACK.ack = 0;
                lastACK.classid = 0;
                lastACK.msgid = 0;
                return 1;
            }
        }
    } else {
        /* NACK ... */
        return 0;
    }
    return 0;
}
