/*
 * usbhandler.cpp
 *
 *  Created on: Mar 29, 2015
 *      Author: bohni
 */

#include "usbhandler.h"

usb_handler::usb_handler(Status* statusPtr, uint8_t defaultPrio,
            USBD_HandleTypeDef* husb)
            : Task(statusPtr, defaultPrio) {

    usb_state = USBD_BUSY;
    usb = husb;
    usb_mode_request = 0;
    confReader = NULL;
}

usb_handler::~usb_handler() {

}

void usb_handler::update() {
    if (number_received_data > 0) {

        switch (UserRxBufferFS[0]) {
            case USB_CMD_LOOP:
                usb_state = CDC_Transmit_FS(UserRxBufferFS, number_received_data);
                break;
            case USB_CMD_SEND_STATUS_8BIT:
                sendStatus8Bit();
                break;
            case USB_CMD_SEND_STATUS_FLOAT:
                sendStatusFloat(0);
                break;
            case USB_CMD_GLOBAL_FLAGS:
                /* HIGH -> LOW */
                UserTxBufferFS[0] = (uint8_t) ((status->globalFlags >> 24) & 0xFF);
                UserTxBufferFS[1] = (uint8_t) ((status->globalFlags >> 16) & 0xFF);
                UserTxBufferFS[2] = (uint8_t) ((status->globalFlags >> 8) & 0xFF);
                UserTxBufferFS[3] = (uint8_t) ((status->globalFlags) & 0xFF);
                usb_state = CDC_Transmit_FS(UserTxBufferFS, 4);
                break;

            case USB_CMD_CONFIG_MODE:
                /* entering config mode */
                if (usb_mode_request == 0) {
                    uint8_t msg[] = { "Entering config mode." };
                    usb_state = CDC_Transmit_FS(msg, sizeof(msg));
                    usb_mode_request = 1;
                } else if (usb_mode_request == 1) {
                    /* leaving config mode*/
                    uint8_t msg[] = { "Leaving config mode. Reset" };
                    usb_state = CDC_Transmit_FS(msg, sizeof(msg));
                    usb_mode_request = 2;
                }
                break;

                /* Config commands */
            case USB_CMD_GET_CONFIG:
                sendConfig();
                break;
            case USB_CMD_UPDATE_CONFIG:
                updateConfig();
                break;
            case USB_CMD_READ_BYTE:
                readEEPROM(1);
                break;
            case USB_CMD_READ_2BYTES:
                readEEPROM(2);
                break;
            case USB_CMD_READ_4BYTES:
                readEEPROM(4);
                break;

            case USB_CMD_WRITE_BYTE:
                writeEEPROM(1);
                break;
            case USB_CMD_WRITE_2BYTES:
                writeEEPROM(2);
                break;
            case USB_CMD_WRITE_4BYTES:
                writeEEPROM(4);
                break;

            case USB_CMD_RESTORE_CONFIG:
                if (usb_mode_request == 1) {
                    uint8_t msg[] = { "Restore hard-coded config" };
                    usb_state = CDC_Transmit_FS(msg, sizeof(msg));
                    status->restoreConfig();
                }
                break;

            case USB_CMD_RESET: {
                uint8_t msg[] = { "Reset" };
                usb_state = CDC_Transmit_FS(msg, sizeof(msg));
                number_received_data = 0;
                usb_mode_request = 0xFF;
            }
                break;
            default:
                uint8_t error_msg[] = { "ERROR:unknown cmd!" };
                usb_state = CDC_Transmit_FS(error_msg, sizeof(error_msg));
                break;
        }
        number_received_data = 0;
        USBD_CDC_ReceivePacket(usb);
    }
    resetPriority();
}

void usb_handler::initialize(ConfigReader* _confReader) {

    SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);

    confReader = _confReader;
    USBD_CDC_ReceivePacket(usb);
}

void usb_handler::sendStatus8Bit() {
    /*
     * old Debug_Data
     *
     0. Byte: Kanal1	= roll
     1. Byte: Kanal2	= throttle
     2. Byte: Kanal3	= nick
     3. Byte: Kanal4	= yaw
     4. Byte: Kanal5	= enable
     5. Byte: Kanal6	= lin control
     6. Byte: Kanal7	= left switch
     7. Byte: Kanal8	= --
     8. Byte: Motor1
     9. Byte: Motor2
     10. Byte: Motor3
     11. Byte: Motor4
     12. Byte: AccX
     13. Byte: AccY
     14. Byte: AccZ
     15. Byte: GyroX
     16. Byte: GyroY
     17. Byte: GyroZ
     18. Byte: Druck
     19. Byte: Temp
     20. Byte: H�he
     21. Byte: Winkel x
     22. Byte: Winkel y
     23. Byte: Winkel z
     */

    /* RC signals*/

    UserTxBufferFS[0] = (uint8_t) (status->rcSignalRoll * 100 + 50);
    UserTxBufferFS[1] = (uint8_t) (status->rcSignalThrottle * 100);
    UserTxBufferFS[2] = (uint8_t) (status->rcSignalNick * 100 + 50);
    UserTxBufferFS[3] = (uint8_t) (status->rcSignalYaw * 100 + 50);
    UserTxBufferFS[4] = (uint8_t) (status->rcSignalEnable * 100);
    UserTxBufferFS[5] = (uint8_t) (status->rcSignalLinPoti * 100);
    UserTxBufferFS[6] = (uint8_t) (status->rcSignalSwitch * 100);

    /* Motor controls*/
    UserTxBufferFS[8] = (uint8_t) (status->motorValues[0] * 100);
    UserTxBufferFS[9] = (uint8_t) (status->motorValues[1] * 100);
    UserTxBufferFS[10] = (uint8_t) (status->motorValues[2] * 100);
    UserTxBufferFS[11] = (uint8_t) (status->motorValues[3] * 100);

    /* Accel XYZ in 0,01 G */
    UserTxBufferFS[12] = (int8_t) (status->accelX / G * 100);
    UserTxBufferFS[13] = (int8_t) (status->accelY / G * 100);
    UserTxBufferFS[14] = (int8_t) (status->accelZ / G * 100);

    /* Gyro XYZ in 10 deg/s */
    UserTxBufferFS[15] = (int8_t) (status->rateX / 10);
    UserTxBufferFS[16] = (int8_t) (status->rateX / 10);
    UserTxBufferFS[17] = (int8_t) (status->rateX / 10);

    /* Temp */
    UserTxBufferFS[19] = (int8_t) (status->temp);
    /* akku voltage */
    UserTxBufferFS[20] = (uint8_t) (status->akkuVoltage);

    /*angles in deg*/
    UserTxBufferFS[21] = (int8_t) (status->angleX);
    UserTxBufferFS[22] = (int8_t) (status->angleY);
    /* north in 10 deg*/
    UserTxBufferFS[23] = (int8_t) (status->angleNorth / 10);

    // TODO usb: send 8bit Magn XYZ */

    usb_state = CDC_Transmit_FS(UserTxBufferFS, 24);
}

void usb_handler::sendStatusFloat(uint8_t part) {
    /* needs to be updated, if new variables are needed to be sent */

    if (part == 0) {
        /* accelerometer XYZ in m/s^2 */
        fillBuffer(UserTxBufferFS, 0, status->accelX);
        fillBuffer(UserTxBufferFS, 4, status->accelY);
        fillBuffer(UserTxBufferFS, 8, status->accelZ);

        /* rate in deg/s */
        fillBuffer(UserTxBufferFS, 12, status->rateX);
        fillBuffer(UserTxBufferFS, 16, status->rateY);
        fillBuffer(UserTxBufferFS, 20, status->rateZ);

        /* magn in gauss ??? */
        fillBuffer(UserTxBufferFS, 24, status->magnetX);
        fillBuffer(UserTxBufferFS, 28, status->magnetY);
        fillBuffer(UserTxBufferFS, 32, status->magnetZ);

        /* angles in deg */
        fillBuffer(UserTxBufferFS, 36, status->angleX);
        fillBuffer(UserTxBufferFS, 40, status->angleY);
        fillBuffer(UserTxBufferFS, 44, status->angleNorth);

        /* rc signals */
        fillBuffer(UserTxBufferFS, 48, status->rcSignalRoll);
        fillBuffer(UserTxBufferFS, 52, status->rcSignalNick);
        fillBuffer(UserTxBufferFS, 56, status->rcSignalYaw);
        fillBuffer(UserTxBufferFS, 60, status->rcSignalThrottle);
        fillBuffer(UserTxBufferFS, 64, status->rcSignalEnable);

        /* motor control values */
        fillBuffer(UserTxBufferFS, 68, status->motorValues[0]);
        fillBuffer(UserTxBufferFS, 72, status->motorValues[1]);
        fillBuffer(UserTxBufferFS, 76, status->motorValues[2]);
        fillBuffer(UserTxBufferFS, 80, status->motorValues[3]);

        usb_state = CDC_Transmit_FS(UserTxBufferFS, 84);
        USBD_CDC_ReceivePacket(usb);
    }
}

void usb_handler::fillBuffer(uint8_t* buffer, uint8_t pos, float var) {

    uint8_t* tmp = (uint8_t*) &var;
    buffer[pos] = *tmp++;
    buffer[pos + 1] = *(tmp++);
    buffer[pos + 2] = *(tmp++);
    buffer[pos + 3] = *(tmp);

}

void usb_handler::sendMSGstring(uint8_t* buffer, uint8_t length) {

    uint32_t timeout = USB_TIMEOUT;
    while ((usb_state = CDC_Transmit_FS(buffer, length)) != USBD_OK) {
        if ((timeout--) == 0) {
            return;
        }
    }
}

void usb_handler::readEEPROM(uint8_t byteCount) {

    switch (byteCount) {
        case 1: {
            uint8_t tmp;
            confReader->loadVariable(&tmp, (uint16_t)
                        (UserRxBufferFS[1] << 8 | UserRxBufferFS[2]));
            UserTxBufferFS[0] = tmp;
            break;
        }
        case 2: {
            uint16_t tmp;
            confReader->loadVariable(&tmp, (uint16_t)
                        (UserRxBufferFS[1] << 8 | UserRxBufferFS[2]));
            UserTxBufferFS[0] = (uint8_t) ((tmp >> 8) & 0xff);
            UserTxBufferFS[1] = (uint8_t) (tmp & 0xff);
            break;
        }
        case 4: {
            uint32_t tmp;
            confReader->loadVariable(&tmp,(uint16_t)
                        (UserRxBufferFS[1] << 8 | UserRxBufferFS[2]));
            UserTxBufferFS[0] = (uint8_t) ((tmp >> 24) & 0xff);
            UserTxBufferFS[1] = (uint8_t) ((tmp >> 16) & 0xff);
            UserTxBufferFS[2] = (uint8_t) ((tmp >> 8) & 0xff);
            UserTxBufferFS[3] = (uint8_t) (tmp & 0xff);
            break;
        }

    }
    /* send eeprom content*/
    usb_state = CDC_Transmit_FS(UserTxBufferFS, byteCount);
}

void usb_handler::writeEEPROM(uint8_t byteCount) {

    switch (byteCount) {
        case 1: {
            uint8_t tmp = UserRxBufferFS[3];
            confReader->saveVariable(&tmp, (uint16_t)
                        (UserRxBufferFS[1] << 8 | UserRxBufferFS[2]), 0);
            break;
        }
        case 2: {
            uint16_t tmp =  (uint16_t)( (UserRxBufferFS[3] << 8) | UserRxBufferFS[4]);
            confReader->saveVariable(&tmp,  (uint16_t)
                        (UserRxBufferFS[1] << 8 | UserRxBufferFS[2]), 0);
            break;
        }

        case 4: {
            uint32_t tmp = ((UserRxBufferFS[3] << 24) | (UserRxBufferFS[4] << 16)
                        | (UserRxBufferFS[5] << 8) | UserRxBufferFS[6]);
            confReader->saveVariable(&tmp,  (uint16_t)
                        (UserRxBufferFS[1] << 8 | UserRxBufferFS[2]), 0);
            break;
        }
    }
    /* send confirmation */
    usb_state = CDC_Transmit_FS(UserRxBufferFS, 1);
}

void usb_handler::sendConfig() {

    /* send config
     *
     */
    /* pid xy */
    fillBuffer(UserTxBufferFS, 0, status->pXY);
    fillBuffer(UserTxBufferFS, 4, status->iXY);
    fillBuffer(UserTxBufferFS, 8, status->dXY);

    /* pid z  */
    fillBuffer(UserTxBufferFS, 12, status->pZ);
    fillBuffer(UserTxBufferFS, 16, status->iZ);
    fillBuffer(UserTxBufferFS, 20, status->dZ);

    /* comp filter */
    fillBuffer(UserTxBufferFS, 24, status->filterCoefficientXY);
    fillBuffer(UserTxBufferFS, 28, status->filterCoefficientZ);

    usb_state = CDC_Transmit_FS(UserTxBufferFS, 32);
    USBD_CDC_ReceivePacket(usb);
}

void usb_handler::updateConfig() {

    status->pXY = BUFFER_TO_FLOAT(UserRxBufferFS,1);
    status->iXY = BUFFER_TO_FLOAT(UserRxBufferFS,5);
    status->dXY = BUFFER_TO_FLOAT(UserRxBufferFS,9);

    status->pZ = BUFFER_TO_FLOAT(UserRxBufferFS,13);
    status->iZ = BUFFER_TO_FLOAT(UserRxBufferFS,17);
    status->dZ = BUFFER_TO_FLOAT(UserRxBufferFS,21);

    status->filterCoefficientXY = BUFFER_TO_FLOAT(UserRxBufferFS,25);
    status->filterCoefficientZ = BUFFER_TO_FLOAT(UserRxBufferFS,29);

}
