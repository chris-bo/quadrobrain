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

            case USB_CMD_SEND_SENSOR_DATA:
            case USB_CMD_SEND_FLIGHT_DATA:
            case USB_CMD_SEND_SYSTEM_STATE:
            case USB_CMD_SEND_GPS_DATA_1:
            case USB_CMD_SEND_GPS_DATA_2:
            case USB_CMD_SEND_STATUS_FLOAT:
                sendStatusFloat(UserRxBufferFS[0]);
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

    if (part == USB_CMD_SEND_STATUS_FLOAT) {
        /* accelerometer XYZ in m/s^2 */
        fillBuffer(UserTxBufferFS, 0, status->accel.x);
        fillBuffer(UserTxBufferFS, 4, status->accel.y);
        fillBuffer(UserTxBufferFS, 8, status->accel.z);

        /* rate in deg/s */
        fillBuffer(UserTxBufferFS, 12, status->rate.x);
        fillBuffer(UserTxBufferFS, 16, status->rate.y);
        fillBuffer(UserTxBufferFS, 20, status->rate.z);

        /* magn in gauss ??? */
        fillBuffer(UserTxBufferFS, 24, status->magnetfield.x);
        fillBuffer(UserTxBufferFS, 28, status->magnetfield.y);
        fillBuffer(UserTxBufferFS, 32, status->magnetfield.z);

        /* angles in deg */
        fillBuffer(UserTxBufferFS, 36, status->angle.x);
        fillBuffer(UserTxBufferFS, 40, status->angle.y);
        fillBuffer(UserTxBufferFS, 44, status->angle.z);

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
        fillBuffer(UserTxBufferFS, 112, status->motorSetpoint.x);
        fillBuffer(UserTxBufferFS, 116, status->motorSetpoint.y);
        fillBuffer(UserTxBufferFS, 120, status->motorSetpoint.z);

        /* cpu load */
        fillBuffer(UserTxBufferFS, 124, status->cpuLoad);

        usbTransmit(UserTxBufferFS, 128);
    } else if(part == USB_CMD_SEND_SENSOR_DATA){
        /* accelerometer XYZ in m/s^2 */
        fillBuffer(UserTxBufferFS, 0, status->accel.x);
        fillBuffer(UserTxBufferFS, 4, status->accel.y);
        fillBuffer(UserTxBufferFS, 8, status->accel.z);

        /* rate in deg/s */
        fillBuffer(UserTxBufferFS, 12, status->rate.x);
        fillBuffer(UserTxBufferFS, 16, status->rate.y);
        fillBuffer(UserTxBufferFS, 20, status->rate.z);

        /* magn in gauss ??? */
        fillBuffer(UserTxBufferFS, 24, status->magnetfield.x);
        fillBuffer(UserTxBufferFS, 28, status->magnetfield.y);
        fillBuffer(UserTxBufferFS, 32, status->magnetfield.z);

        /* Temperature */
        fillBuffer(UserTxBufferFS, 36, status->temp);

        /* Height */
        fillBuffer(UserTxBufferFS, 40, status->height);
        fillBuffer(UserTxBufferFS, 44, status->height_rel);
        fillBuffer(UserTxBufferFS, 48, status->d_h);

        /* rc signals */
        fillBuffer(UserTxBufferFS, 52, status->rcSignalRoll);
        fillBuffer(UserTxBufferFS, 56, status->rcSignalNick);
        fillBuffer(UserTxBufferFS, 60, status->rcSignalYaw);
        fillBuffer(UserTxBufferFS, 64, status->rcSignalThrottle);
        fillBuffer(UserTxBufferFS, 68, (float) status->rcSignalEnable);
        fillBuffer(UserTxBufferFS, 72, (float) status->rcSignalSwitch);
        fillBuffer(UserTxBufferFS, 74, status->rcSignalLinPoti);

        usbTransmit(UserTxBufferFS, 78);
    } else if(part == USB_CMD_SEND_FLIGHT_DATA){

        /* angles in deg */
        fillBuffer(UserTxBufferFS, 0, status->angle.x);
        fillBuffer(UserTxBufferFS, 4, status->angle.y);
        fillBuffer(UserTxBufferFS, 8, status->angle.z);

        fillBuffer(UserTxBufferFS, 12, status->angleSetpoint.x);
        fillBuffer(UserTxBufferFS, 16, status->angleSetpoint.y);
        fillBuffer(UserTxBufferFS, 20, status->angleSetpoint.z);

        /* velocities in m/s */
        fillBuffer(UserTxBufferFS, 24, status->velocity.x);
        fillBuffer(UserTxBufferFS, 28, status->velocity.y);
        fillBuffer(UserTxBufferFS, 32, status->velocity.z);

        fillBuffer(UserTxBufferFS, 36, status->velocitySetpoint.x);
        fillBuffer(UserTxBufferFS, 40, status->velocitySetpoint.y);
        fillBuffer(UserTxBufferFS, 44, status->velocitySetpoint.z);

        /* Height */
        fillBuffer(UserTxBufferFS, 48, status->height);
        fillBuffer(UserTxBufferFS, 52, status->height_rel);
        fillBuffer(UserTxBufferFS, 56, status->d_h);

        /* motor control values */
        fillBuffer(UserTxBufferFS, 60, status->motorValues[0]);
        fillBuffer(UserTxBufferFS, 64, status->motorValues[1]);
        fillBuffer(UserTxBufferFS, 68, status->motorValues[2]);
        fillBuffer(UserTxBufferFS, 72, status->motorValues[3]);

        /* PID Outputs */
        fillBuffer(UserTxBufferFS, 76, status->motorSetpoint.x);
        fillBuffer(UserTxBufferFS, 80, status->motorSetpoint.y);
        fillBuffer(UserTxBufferFS, 84, status->motorSetpoint.z);

        usbTransmit(UserTxBufferFS, 88);
    } else if(part == USB_CMD_SEND_SYSTEM_STATE){

        /* AkkuVoltage */
        fillBuffer(UserTxBufferFS, 0, status->akkuVoltage);
        /* cpu load */
        fillBuffer(UserTxBufferFS, 4, status->cpuLoad);

        /* uptime */
        UserTxBufferFS[8] = (uint8_t) ((status->uptime & 0xFF) >> 0);
        UserTxBufferFS[9] = (uint8_t) ((status->uptime & 0xFF00) >> 8);
        UserTxBufferFS[10] = (uint8_t) ((status->uptime & 0xFF0000) >> 16);
        UserTxBufferFS[11] = (uint8_t) ((status->uptime & 0xFF000000) >> 24);

        usbTransmit(UserTxBufferFS, 12);
    } else if(part == USB_CMD_SEND_GPS_DATA_1){
        //TODO: USBHandler GPS data transmission
    } else if(part == USB_CMD_SEND_GPS_DATA_2){

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
    fillBuffer(UserTxBufferFS, 0, status->pidSettigsAngleXY.p);
    fillBuffer(UserTxBufferFS, 4, status->pidSettigsAngleXY.i);
    fillBuffer(UserTxBufferFS, 8, status->pidSettigsAngleXY.d);
    fillBuffer(UserTxBufferFS, 12, status->pidSettigsAngleXY.gain);
    fillBuffer(UserTxBufferFS, 16, status->pidSettigsAngleXY.scaleSetPoint);

    /* pid z  */
    fillBuffer(UserTxBufferFS, 20, status->pidSettigsRotationZ.p);
    fillBuffer(UserTxBufferFS, 24, status->pidSettigsRotationZ.i);
    fillBuffer(UserTxBufferFS, 28, status->pidSettigsRotationZ.d);
    fillBuffer(UserTxBufferFS, 32, status->pidSettigsRotationZ.gain);
    fillBuffer(UserTxBufferFS, 36, status->pidSettigsRotationZ.scaleSetPoint);

    /* comp filter */
    fillBuffer(UserTxBufferFS, 40, status->filterCoefficientXY);
    fillBuffer(UserTxBufferFS, 44, status->filterCoefficientZ);

    usbTransmit(UserTxBufferFS, 48);

}

void usb_handler::updateConfig() {

    status->pidSettigsAngleXY.p = byteToFloat(UserRxBufferFS, 1);
    status->pidSettigsAngleXY.i = byteToFloat(UserRxBufferFS, 5);
    status->pidSettigsAngleXY.d = byteToFloat(UserRxBufferFS, 9);
    status->pidSettigsAngleXY.gain = byteToFloat(UserRxBufferFS, 13);
    status->pidSettigsAngleXY.scaleSetPoint = byteToFloat(UserRxBufferFS, 17);

    status->pidSettigsRotationZ.p = byteToFloat(UserRxBufferFS, 21);
    status->pidSettigsRotationZ.i = byteToFloat(UserRxBufferFS, 25);
    status->pidSettigsRotationZ.d = byteToFloat(UserRxBufferFS, 29);
    status->pidSettigsRotationZ.gain = byteToFloat(UserRxBufferFS, 33);
    status->pidSettigsRotationZ.scaleSetPoint = byteToFloat(UserRxBufferFS, 37);

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

