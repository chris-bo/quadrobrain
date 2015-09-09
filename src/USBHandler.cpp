/*
 * usbhandler.cpp
 *
 *  Created on: Mar 29, 2015
 *      Author: bohni
 */

#include <USBHandler.h>

static float byteToFloat(uint8_t* array, uint8_t offset) {
    float val;
    memcpy(&val, array + offset, 4l);
    return val;
}

USBHandler::USBHandler(Status* statusPtr, uint8_t defaultPrio,
            USBD_HandleTypeDef* husb)
            : Task(statusPtr, defaultPrio) {

    usb_state = USBD_BUSY;
    usb = husb;
    usb_mode_request = 0;
    confReader = NULL;
    usbTransmitBusyCounter = 0;
}

USBHandler::~USBHandler() {

}

void USBHandler::update() {
    /* reset leds*/
    leds.off(USB_RECEIVE_LED);
    leds.off(USB_TRANSMIT_LED);

    if (number_received_data > 0) {
        leds.on(USB_RECEIVE_LED);
        switch (UserRxBufferFS[0]) {
            case USB_CMD_LOOP:
                usbTransmit(UserRxBufferFS, number_received_data);
                break;
            case USB_CMD_SEND_CUSTOM_FRAME:
                sendCustomFrame();
                break;
            case USB_CMD_GLOBAL_FLAGS:
                createCustomFrame(USB_CMD_GLOBAL_FLAGS);
                break;
            case USB_CMD_SET_FLIGHT_LED_PATTERN:
                flightLEDs.setLEDpattern(
                            (uint16_t) ((UserRxBufferFS[1] << 8) | UserRxBufferFS[2]));
                /* send confirmation */
                usbTransmit(UserRxBufferFS, 1);
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
                /* Config commands */
                if ((UserRxBufferFS[0] & 0xC0) == 0xC0) {
                    decodeConfigMSG();
                }
                break;
        }

        if (usb_state == USBD_OK) {
            number_received_data = 0;
        }
        USBD_CDC_ReceivePacket(usb);
    }
    resetPriority();
}

void USBHandler::initialize(ConfigReader* _confReader) {

    SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);

    confReader = _confReader;
    USBD_CDC_ReceivePacket(usb);
}

void USBHandler::readEEPROM(uint8_t byteCount) {

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

void USBHandler::writeEEPROM(uint8_t byteCount) {

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

void USBHandler::updateConfig() {

    status->pidSettingsAngleXY.p = byteToFloat(UserRxBufferFS, 1);
    status->pidSettingsAngleXY.i = byteToFloat(UserRxBufferFS, 5);
    status->pidSettingsAngleXY.d = byteToFloat(UserRxBufferFS, 9);
    status->pidSettingsAngleXY.gain = byteToFloat(UserRxBufferFS, 13);
    status->pidSettingsAngleXY.scaleSetPoint = byteToFloat(UserRxBufferFS, 17);

    status->pidSettingsRotationZ.p = byteToFloat(UserRxBufferFS, 21);
    status->pidSettingsRotationZ.i = byteToFloat(UserRxBufferFS, 25);
    status->pidSettingsRotationZ.d = byteToFloat(UserRxBufferFS, 29);
    status->pidSettingsRotationZ.gain = byteToFloat(UserRxBufferFS, 33);
    status->pidSettingsRotationZ.scaleSetPoint = byteToFloat(UserRxBufferFS, 37);

    status->filterCoefficientXY = byteToFloat(UserRxBufferFS, 41);
    status->filterCoefficientZ = byteToFloat(UserRxBufferFS, 45);

    status->pidSettingsRotationZ.p = byteToFloat(UserRxBufferFS, 49);
    status->pidSettingsRotationZ.i = byteToFloat(UserRxBufferFS, 53);
    status->pidSettingsRotationZ.d = byteToFloat(UserRxBufferFS, 57);
    status->pidSettingsRotationZ.gain = byteToFloat(UserRxBufferFS, 61);
    status->pidSettingsRotationZ.scaleSetPoint = byteToFloat(UserRxBufferFS, 65);

    status->pidSettingsRotationZ.p = byteToFloat(UserRxBufferFS, 69);
    status->pidSettingsRotationZ.i = byteToFloat(UserRxBufferFS, 73);
    status->pidSettingsRotationZ.d = byteToFloat(UserRxBufferFS, 77);
    status->pidSettingsRotationZ.gain = byteToFloat(UserRxBufferFS, 81);
    status->pidSettingsRotationZ.scaleSetPoint = byteToFloat(UserRxBufferFS, 85);

    status->qcSettings.enableBuzzerWarningRCLost = UserRxBufferFS[89];
    status->qcSettings.enableBuzzerWarningLowVoltage = UserRxBufferFS[90];
    status->qcSettings.enableFlightLeds = UserRxBufferFS[91];
    status->qcSettings.enableMotors = UserRxBufferFS[92];

    usbTransmit(UserRxBufferFS, 1);

}

void USBHandler::usbTransmit(uint8_t* buffer, uint16_t len) {
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

void USBHandler::kill() {
    /* override default kill function
     * usb must not be killed
     */
}

void USBHandler::sendCustomFrame() {
    uint8_t bufferPos = 0;
    uint8_t bufferOverrun = 0;

    /* start loop at beginning of data ids */
    for (uint8_t i = 1; i < number_received_data; i++) {
        /* add requested values
         * but check fist if tx buffer has enough space
         */
        switch (UserRxBufferFS[i]) {
            case DATA_ID_GYRO:
                if (!checkTXBufferOverrun(bufferPos, 12, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->rate.x);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->rate.y);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->rate.z);
                }
                break;
            case DATA_ID_ACCEL:
                if (!checkTXBufferOverrun(bufferPos, 12, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->accel.x);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->accel.y);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->accel.z);
                }
                break;
            case DATA_ID_MAGNETOMETER:
                if (!checkTXBufferOverrun(bufferPos, 12, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->magnetfield.x);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->magnetfield.y);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->magnetfield.z);
                }
                break;
            case DATA_ID_ANGLE:
                if (!checkTXBufferOverrun(bufferPos, 12, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->angle.x);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->angle.y);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->angle.z);
                }
                break;
            case DATA_ID_ANGLE_SP:
                if (!checkTXBufferOverrun(bufferPos, 12, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->angleSetpoint.x);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->angleSetpoint.y);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->angleSetpoint.z);
                }
                break;
            case DATA_ID_VELOCITY:
                if (!checkTXBufferOverrun(bufferPos, 12, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->velocity.x);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->velocity.y);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->velocity.z);
                }
                break;
            case DATA_ID_VELOCITY_SP:
                if (!checkTXBufferOverrun(bufferPos, 12, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->velocitySetpoint.x);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->velocitySetpoint.y);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->velocitySetpoint.z);
                }
                break;
            case DATA_ID_HEIGHT:
                if (!checkTXBufferOverrun(bufferPos, 12, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->height);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->height_rel);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos, status->d_h);
                }
                break;
            case DATA_ID_RC:
                if (!checkTXBufferOverrun(bufferPos, 22, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->rcSignalNick);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->rcSignalRoll);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->rcSignalYaw);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->rcSignalThrottle);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->rcSignalLinPoti);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->rcSignalEnable);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->rcSignalSwitch);
                }
                break;
            case DATA_ID_MOTOR_SP:
                if (!checkTXBufferOverrun(bufferPos, 12, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->motorSetpoint.x);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->motorSetpoint.y);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->motorSetpoint.z);
                }
                break;
            case DATA_ID_MOTOR:
                if (!checkTXBufferOverrun(bufferPos, 16, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->motorValues[0]);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->motorValues[1]);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->motorValues[2]);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->motorValues[3]);
                }
                break;
            case DATA_ID_CPU:
                if (!checkTXBufferOverrun(bufferPos, 4, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->cpuLoad);
                }
                break;
            case DATA_ID_AKKU:
                if (!checkTXBufferOverrun(bufferPos, 4, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->akkuVoltage);
                }
                break;
            case DATA_ID_TEMP:
                if (!checkTXBufferOverrun(bufferPos, 4, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos, status->temp);
                }
                break;
            case DATA_ID_UPTIME:
                if (!checkTXBufferOverrun(bufferPos, 4, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->uptime);
                }
                break;
            case DATA_ID_GPS_LLH:
                if (!checkTXBufferOverrun(bufferPos, 56, &bufferOverrun)) {
                    /* postition llh with accuracy*/
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.llh_data.lat);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.llh_data.lon);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.llh_data.h);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.llh_data.hMSL);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.llh_data.vAcc);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.llh_data.hAcc);
                    /* ned*/
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.ned_data.vN);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.ned_data.vE);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.ned_data.vD);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.ned_data.speed);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.ned_data.gSpeed);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.ned_data.sAcc);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.ned_data.heading);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.ned_data.cAcc);

                }
                break;
            case DATA_ID_GPS_ECEF:
                if (!checkTXBufferOverrun(bufferPos, 32, &bufferOverrun)) {
                    /* position ecef */
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.ecef_data.x);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.ecef_data.y);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.ecef_data.z);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.ecef_data.vx);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.ecef_data.vy);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.ecef_data.vz);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.ecef_data.pAcc);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.ecef_data.sAcc);
                }
                break;
            case DATA_ID_GPS_DOP:
                if (!checkTXBufferOverrun(bufferPos, 14, &bufferOverrun)) {
                    /* dilution of precission*/
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.dop.pDOP);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.dop.gDOP);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.dop.tDOP);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.dop.vDOP);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.dop.hDOP);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.dop.nDOP);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.dop.eDOP);
                }
                break;
            case DATA_ID_GPS_TIME:
                if (!checkTXBufferOverrun(bufferPos, 15, &bufferOverrun)) {
                    /* gps time of week*/
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.iTOW);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.gpsWeek);
                    /* gps time */
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.time.hours);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.time.minutes);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.time.seconds);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.time.hundredths);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.time.validity);
                    /* gps date*/
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.date.year);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.date.month);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.date.day);
                }
                break;
            case DATA_ID_GPS_FIX:
                /* fix + flags*/
                if (!checkTXBufferOverrun(bufferPos, 5, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                (uint8_t) status->gpsData.gpsFix);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.FixStatus);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.numSV);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->gpsData.navStatusFlags);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                (uint8_t) status->gpsData.psmState);
                }
                break;
            case DATA_ID_COMP_FILTER:
                if (!checkTXBufferOverrun(bufferPos, 8, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->filterCoefficientXY);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->filterCoefficientZ);
                }
                break;
            case DATA_ID_PID_ANGLE_XY:
                if (!checkTXBufferOverrun(bufferPos, 20, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsAngleXY.p);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsAngleXY.i);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsAngleXY.d);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsAngleXY.gain);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsAngleXY.scaleSetPoint);
                }
                break;
            case DATA_ID_PID_ROT_Z:
                if (!checkTXBufferOverrun(bufferPos, 20, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsRotationZ.p);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsRotationZ.i);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsRotationZ.d);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsRotationZ.gain);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsRotationZ.scaleSetPoint);
                }
                break;
            case DATA_ID_PID_VEL:
                if (!checkTXBufferOverrun(bufferPos, 20, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsVelocity.p);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsVelocity.i);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsVelocity.d);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsVelocity.gain);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsVelocity.scaleSetPoint);
                }
                break;
            case DATA_ID_PID_ACCEL:
                if (!checkTXBufferOverrun(bufferPos, 20, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsAcceleration.p);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsAcceleration.i);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsAcceleration.d);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsAcceleration.gain);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->pidSettingsAcceleration.scaleSetPoint);
                }
                break;
            case DATA_ID_QC_SETTING:
                if (!checkTXBufferOverrun(bufferPos, 4, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->qcSettings.enableBuzzerWarningLowVoltage);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->qcSettings.enableBuzzerWarningRCLost);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->qcSettings.enableFlightLeds);
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->qcSettings.enableMotors);
                }
                break;
            case DATA_ID_GLOBAL_FLAGS:
                if (!checkTXBufferOverrun(bufferPos, 4, &bufferOverrun)) {
                    bufferPos = fillBuffer(UserTxBufferFS, bufferPos,
                                status->globalFlags );
                }
                break;
            case DATA_ID_EOF:
                /* end of frame*/
                /* send buffer */
                usbTransmit(UserTxBufferFS, bufferPos);
                break;
            default:
                /* unknown id */
                break;

        }
        if (bufferOverrun == 1) {
            /* Buffer Overrun
             * not enough space to push requested data into buffer
             *
             * send only DATA_ID_BUFFER_OVERRUN
             */
            UserTxBufferFS[0] = DATA_ID_BUFFER_OVERRUN;
            bufferPos = 1;
            usbTransmit(UserTxBufferFS, bufferPos);
            break;
        }
    }
}

void USBHandler::resetTransmissionState() {
    /* clear endpoint*/
    usb->pClass->DataIn(usb, 1);
    PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *) usb->pData;
    PCD_EPTypeDef *ep = (PCD_EPTypeDef*) &hpcd->IN_ep[1];
    ep->xfer_count = 0;
    usbTransmitBusyCounter = 0;
}

uint8_t USBHandler::fillBuffer(uint8_t* buffer, uint8_t pos, float var) {
    uint8_t* tmp = (uint8_t*) &var;
    buffer[pos++] = *tmp++;
    buffer[pos++] = *(tmp++);
    buffer[pos++] = *(tmp++);
    buffer[pos++] = *(tmp);
    return pos;
}

uint8_t USBHandler::fillBuffer(uint8_t* buffer, uint8_t pos, uint32_t var) {
    uint8_t* tmp = (uint8_t*) &var;
    buffer[pos++] = *tmp++;
    buffer[pos++] = *(tmp++);
    buffer[pos++] = *(tmp++);
    buffer[pos++] = *(tmp);
    return pos;
}

uint8_t USBHandler::fillBuffer(uint8_t* buffer, uint8_t pos, int32_t var) {
    uint8_t* tmp = (uint8_t*) &var;
    buffer[pos++] = *tmp++;
    buffer[pos++] = *(tmp++);
    buffer[pos++] = *(tmp++);
    buffer[pos++] = *(tmp);
    return pos;
}

uint8_t USBHandler::fillBuffer(uint8_t* buffer, uint8_t pos, int16_t var) {
    uint8_t* tmp = (uint8_t*) &var;
    buffer[pos++] = *tmp++;
    buffer[pos++] = *(tmp);
    return pos;
}

uint8_t USBHandler::fillBuffer(uint8_t* buffer, uint8_t pos, uint16_t var) {
    uint8_t* tmp = (uint8_t*) &var;
    buffer[pos++] = *tmp++;
    buffer[pos++] = *(tmp);
    return pos;
}

uint8_t USBHandler::fillBuffer(uint8_t* buffer, uint8_t pos, int8_t var) {
    buffer[pos++] = var;
    ;
    return pos;
}

uint8_t USBHandler::fillBuffer(uint8_t* buffer, uint8_t pos, uint8_t var) {
    buffer[pos++] = var;
    return pos;
}

uint8_t USBHandler::checkTXBufferOverrun(uint8_t currentPos, uint8_t dataToAdd,
            uint8_t* overrun) {
    if ((currentPos + dataToAdd) < USB_TX_BUFF_SIZE) {
        return 0;
    } else {
        *overrun = 1;
        return 1;
    }
}

void USBHandler::decodeConfigMSG() {

    switch (UserRxBufferFS[0]) {
        case USB_CMD_GET_CONFIG:
            createCustomFrame(USB_CMD_GET_CONFIG);
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
        case USB_CMD_QUADROCONFIG:
            if (number_received_data == 2) {
                /* check and set settings */
                if (UserRxBufferFS[1] & QUADROCONFIG_ENABLE_LOW_VOLT) {
                    status->qcSettings.enableBuzzerWarningLowVoltage = 1;
                } else {
                    status->qcSettings.enableBuzzerWarningLowVoltage = 0;
                }
                if (UserRxBufferFS[1] & QUADROCONFIG_ENABLE_RC_LOST) {
                    status->qcSettings.enableBuzzerWarningRCLost = 1;
                } else {
                    status->qcSettings.enableBuzzerWarningRCLost = 0;
                }
                if (UserRxBufferFS[1] & QUADROCONFIG_ENABLE_FLIGHTLED) {
                    status->qcSettings.enableFlightLeds = 1;
                } else {
                    status->qcSettings.enableFlightLeds = 0;
                }
                if (UserRxBufferFS[1] & QUADROCONFIG_ENABLE_MOTORS) {
                    status->qcSettings.enableMotors = 1;
                } else {
                    status->qcSettings.enableMotors = 0;
                }

                /* send confirmation */
                usbTransmit(UserRxBufferFS, 1);
            } else {
                /* reply with current setting*/
                UserTxBufferFS[0] = UserRxBufferFS[0];
                UserTxBufferFS[1] = (uint8_t) ((status->qcSettings.enableMotors
                            * QUADROCONFIG_ENABLE_MOTORS)
                            | (status->qcSettings.enableFlightLeds
                                        * QUADROCONFIG_ENABLE_FLIGHTLED)
                            | (status->qcSettings.enableBuzzerWarningRCLost
                                        * QUADROCONFIG_ENABLE_RC_LOST)
                            | (status->qcSettings.enableBuzzerWarningLowVoltage
                                        * QUADROCONFIG_ENABLE_LOW_VOLT));
                usbTransmit(UserTxBufferFS, 2);
            }
            break;
        case USB_CMD_RELOAD_EEPROM:
            if (usb_mode_request == USB_MODE_CONFIG) {
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
        default:
            break;
    }

}

/* generated array of frame idetifiers for old usb cmds */
void USBHandler::createCustomFrame(uint8_t frameID) {
    switch (frameID) {
        case USB_CMD_GET_CONFIG:
            /* pid xy */
            UserRxBufferFS[1] = DATA_ID_PID_ANGLE_XY;
            /* pid z  */
            UserRxBufferFS[2] = DATA_ID_PID_ROT_Z;
            /* comp filter */
            UserRxBufferFS[3] = DATA_ID_COMP_FILTER;
            /* pid accel  */
            UserRxBufferFS[4] = DATA_ID_PID_ACCEL;
            /* pid vel */
            UserRxBufferFS[5] = DATA_ID_PID_VEL;
            /* qc settings*/
            UserRxBufferFS[6] = DATA_ID_QC_SETTING;
            number_received_data = 7;
            break;
        case USB_CMD_GLOBAL_FLAGS:
            UserRxBufferFS[1] = DATA_ID_GLOBAL_FLAGS;
            number_received_data = 2;
            break;
        default:
            break;
    }

    UserRxBufferFS[number_received_data++] = DATA_ID_EOF;
    /* send generated custom frame */
    sendCustomFrame();
}
