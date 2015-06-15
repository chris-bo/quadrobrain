/*
 * usbhandler.cpp
 *
 *  Created on: Mar 29, 2015
 *      Author: bohni
 */

#include "usbhandler.h"

float byteToFloat(uint8_t* array, uint8_t offset)
{
float val;

memcpy(&val,array + offset,4l);

return val;
}

usb_handler::usb_handler(Status* statusPtr, uint8_t defaultPrio,
            USBD_HandleTypeDef* husb)
            : Task(statusPtr, defaultPrio) {

    usb_state = USBD_BUSY;
    usb = husb;
    usb_mode_request = 0;
    confReader = NULL;
    usbTransmitBusyCounter = 0;
}

usb_handler::~usb_handler() {

}

void usb_handler::update() {
    /* reset leds*/
    leds.off(USB_RECEIVE_LED);
    leds.off(USB_TRANSMIT_LED);

    if (number_received_data > 0) {
        leds.on(USB_RECEIVE_LED);
        switch (UserRxBufferFS[0]) {
            case USB_CMD_LOOP:
                usbTransmit(UserRxBufferFS, number_received_data);
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
                usbTransmit(UserTxBufferFS, 4);
                break;

            case USB_CMD_CONFIG_MODE:
                /* entering config mode */
                if (usb_mode_request == USB_MODE_NORMAL) {
                    /* send confirmation */
                    usbTransmit(UserRxBufferFS, 1);
                    usb_mode_request = USB_MODE_CONFIG;
                } else if (usb_mode_request == USB_MODE_CONFIG) {
                    /* leaving config mode*/
                    /* send confirmation */
                    usbTransmit(UserRxBufferFS, 1);
                    usb_mode_request = USB_MODE_LEAVE_CONFIG;
                }
                /* reset puffer to prevent executing twice */
                UserRxBufferFS[0] = 0;
                break;
                /* Config commands */
            case USB_CMD_GET_CONFIG:
                sendConfig();
                break;
            case USB_CMD_UPDATE_CONFIG:
                updateConfig();
                break;
            case USB_CMD_READ_BYTE:
                if (usb_mode_request == USB_MODE_CONFIG) {
                    readEEPROM(1);
                }
                break;
            case USB_CMD_READ_2BYTES:
                if (usb_mode_request == USB_MODE_CONFIG) {
                    readEEPROM(2);
                }
                break;
            case USB_CMD_READ_4BYTES:
                if (usb_mode_request == USB_MODE_CONFIG) {
                    readEEPROM(4);
                }
                break;

            case USB_CMD_WRITE_BYTE:
                if (usb_mode_request == USB_MODE_CONFIG) {
                    writeEEPROM(1);
                }
                break;
            case USB_CMD_WRITE_2BYTES:
                if (usb_mode_request == USB_MODE_CONFIG) {
                    writeEEPROM(2);
                }
                break;
            case USB_CMD_WRITE_4BYTES:
                if (usb_mode_request == USB_MODE_CONFIG) {
                    writeEEPROM(4);
                }
                break;
            case USB_CMD_RELOAD_EEPROM:
                if (usb_mode_request == USB_MODE_CONFIG){
                    usb_mode_request = USB_MODE_RELOAD_EEPROM;
                    /* send confirmation */
                    usbTransmit(UserRxBufferFS, 1);
                }
                break;
            case USB_CMD_SAVE_CONFIG:
                usb_mode_request = USB_MODE_SAVE_CONFIG;
                /* send confirmation */
                usbTransmit(UserRxBufferFS, 1);
                break;

            case USB_CMD_RESTORE_CONFIG:
                status->restoreConfig();
                /* send confirmation */
                usbTransmit(UserRxBufferFS, 1);
                /* reset puffer to prevent executing twice */
                UserRxBufferFS[0] = 0;
                break;

            case USB_CMD_RESET: {
                /* Reset */
                /* send confirmation */
                usbTransmit(UserRxBufferFS, 1);
                /* reset puffer to prevent executing twice */
                UserRxBufferFS[0] = 0;
                number_received_data = 0;
                usb_mode_request = USB_MODE_RESET;
            }
                break;
            default:
                break;
        }

        if (usb_state == USBD_OK) {
            number_received_data = 0;
        }
        USBD_CDC_ReceivePacket(usb);
    }
    resetPriority();
}

void usb_handler::initialize(ConfigReader* _confReader) {

    SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);

    confReader = _confReader;
    USBD_CDC_ReceivePacket(usb);
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
        fillBuffer(UserTxBufferFS, 64, (float) status->rcSignalEnable);
        fillBuffer(UserTxBufferFS, 68, (float) status->rcSignalSwitch);
        fillBuffer(UserTxBufferFS, 72, status->rcSignalLinPoti);

        /* motor control values */
        fillBuffer(UserTxBufferFS, 76, status->motorValues[0]);
        fillBuffer(UserTxBufferFS, 80, status->motorValues[1]);
        fillBuffer(UserTxBufferFS, 84, status->motorValues[2]);
        fillBuffer(UserTxBufferFS, 88, status->motorValues[3]);

        /* Temperature */
        fillBuffer(UserTxBufferFS, 92, status->temp);

        /* Height */
        fillBuffer(UserTxBufferFS, 96, status->height);
        fillBuffer(UserTxBufferFS, 100, status->height_rel);
        fillBuffer(UserTxBufferFS, 104, status->d_h);

        /* AkkuVoltage */
        fillBuffer(UserTxBufferFS, 108, status->akkuVoltage);

        /* PID Outputs */
        fillBuffer(UserTxBufferFS, 112, status->pidXOut);
        fillBuffer(UserTxBufferFS, 116, status->pidYOut);
        fillBuffer(UserTxBufferFS, 120, status->pidZOut);

        /* cpu load */
        fillBuffer(UserTxBufferFS, 124, status->cpuLoad);

        usbTransmit(UserTxBufferFS, 128);
    }
}

void usb_handler::fillBuffer(uint8_t* buffer, uint8_t pos, float var) {

    uint8_t* tmp = (uint8_t*) &var;
    buffer[pos] = *tmp++;
    buffer[pos + 1] = *(tmp++);
    buffer[pos + 2] = *(tmp++);
    buffer[pos + 3] = *(tmp);

}

void usb_handler::readEEPROM(uint8_t byteCount) {

    switch (byteCount) {
        case 1: {
            uint8_t tmp;
            confReader->loadVariable(&tmp,
                        (uint16_t) (UserRxBufferFS[1] << 8 | UserRxBufferFS[2]));
            UserTxBufferFS[0] = tmp;
            break;
        }
        case 2: {
            uint16_t tmp;
            confReader->loadVariable(&tmp,
                        (uint16_t) (UserRxBufferFS[1] << 8 | UserRxBufferFS[2]));
            UserTxBufferFS[0] = (uint8_t) ((tmp >> 8) & 0xff);
            UserTxBufferFS[1] = (uint8_t) (tmp & 0xff);
            break;
        }
        case 4: {
            uint32_t tmp;
            confReader->loadVariable(&tmp,
                        (uint16_t) (UserRxBufferFS[1] << 8 | UserRxBufferFS[2]));
            UserTxBufferFS[0] = (uint8_t) ((tmp >> 24) & 0xff);
            UserTxBufferFS[1] = (uint8_t) ((tmp >> 16) & 0xff);
            UserTxBufferFS[2] = (uint8_t) ((tmp >> 8) & 0xff);
            UserTxBufferFS[3] = (uint8_t) (tmp & 0xff);
            break;
        }

    }
    /* send eeprom content*/
    usbTransmit(UserTxBufferFS, byteCount);
}

void usb_handler::writeEEPROM(uint8_t byteCount) {

    switch (byteCount) {
        case 1: {
            uint8_t tmp = UserRxBufferFS[3];
            confReader->saveVariable(&tmp,
                        (uint16_t) (UserRxBufferFS[1] << 8 | UserRxBufferFS[2]), 0);
            break;
        }
        case 2: {
            uint16_t tmp = (uint16_t) ((UserRxBufferFS[3] << 8) | UserRxBufferFS[4]);
            confReader->saveVariable(&tmp,
                        (uint16_t) (UserRxBufferFS[1] << 8 | UserRxBufferFS[2]), 0);
            break;
        }

        case 4: {
            uint32_t tmp = ((UserRxBufferFS[3] << 24) | (UserRxBufferFS[4] << 16)
                        | (UserRxBufferFS[5] << 8) | UserRxBufferFS[6]);
            confReader->saveVariable(&tmp,
                        (uint16_t) (UserRxBufferFS[1] << 8 | UserRxBufferFS[2]), 0);
            break;
        }
    }
    /* send confirmation */
    usbTransmit(UserRxBufferFS, 1);
}

void usb_handler::sendConfig() {

    /* send config
     *
     */
    /* pid xy */
    fillBuffer(UserTxBufferFS, 0, status->pXY);
    fillBuffer(UserTxBufferFS, 4, status->iXY);
    fillBuffer(UserTxBufferFS, 8, status->dXY);
    fillBuffer(UserTxBufferFS, 12, status->gainXY);
    fillBuffer(UserTxBufferFS, 16, status->scaleXY);

    /* pid z  */
    fillBuffer(UserTxBufferFS, 20, status->pZ);
    fillBuffer(UserTxBufferFS, 24, status->iZ);
    fillBuffer(UserTxBufferFS, 28, status->dZ);
    fillBuffer(UserTxBufferFS, 32, status->gainZ);
    fillBuffer(UserTxBufferFS, 36, status->scaleZ);

    /* comp filter */
    fillBuffer(UserTxBufferFS, 40, status->filterCoefficientXY);
    fillBuffer(UserTxBufferFS, 44, status->filterCoefficientZ);

    usbTransmit(UserTxBufferFS, 48);

}

void usb_handler::updateConfig() {

    status->pXY = byteToFloat(UserRxBufferFS, 1);
    status->iXY = byteToFloat(UserRxBufferFS, 5);
    status->dXY = byteToFloat(UserRxBufferFS, 9);
    status->gainXY = byteToFloat(UserRxBufferFS, 13);
    status->scaleXY = byteToFloat(UserRxBufferFS, 17);

    status->pZ = byteToFloat(UserRxBufferFS, 21);
    status->iZ = byteToFloat(UserRxBufferFS, 25);
    status->dZ = byteToFloat(UserRxBufferFS, 29);
    status->gainZ = byteToFloat(UserRxBufferFS, 33);
    status->scaleZ = byteToFloat(UserRxBufferFS, 37);

    status->filterCoefficientXY = byteToFloat(UserRxBufferFS, 41);
    status->filterCoefficientZ = byteToFloat(UserRxBufferFS, 45);

    usbTransmit(UserRxBufferFS, 1);

}

void usb_handler::usbTransmit(uint8_t* buffer, uint16_t len) {
    usb_state = CDC_Transmit_FS(buffer, len);

    if (usb_state == USBD_BUSY) {
        usbTransmitBusyCounter++;
        if (usbTransmitBusyCounter == USB_TRANSMIT_BUSY_MAX) {
            resetTransmissionState();
            SET_FLAG(status->globalFlags, USB_ERROR_FLAG | ERROR_FLAG);
        }
    } else {
        leds.on(USB_TRANSMIT_LED);
        usbTransmitBusyCounter = 0;
        RESET_FLAG(status->globalFlags, USB_ERROR_FLAG);
    }

}

void usb_handler::kill() {
}

void usb_handler::resetTransmissionState() {
    /* clear endpoint*/
    usb->pClass->DataIn(usb, 1);
    PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *) usb->pData;
    PCD_EPTypeDef *ep = (PCD_EPTypeDef*) &hpcd->IN_ep[1];
    ep->xfer_count = 0;
    usbTransmitBusyCounter = 0;
}

