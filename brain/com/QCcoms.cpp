/*
 * QCcoms.cpp
 *
 *  Created on: Nov 22, 2017
 *      Author: chris
 */

#include "QCcoms.h"

static float byteToFloat(uint8_t* array, uint16_t* offset) {
    float val;
    memcpy(&val, array + *offset, 4l);
    *offset = *offset + 4;
    return val;
}

QCcoms::QCcoms(Status* statusPtr, uint8_t defaultPrio,
        ConfigReader* _confReader, RxTxHandler* _rxtxHandler,
        FlightLED* _flightLEDs) :
        Task(statusPtr, defaultPrio) {

    confReader = _confReader;
    rxtxHandler = _rxtxHandler;
    flightLEDs = _flightLEDs;
    bufferOverrun = 0;
    customFramePos = 2;
}

QCcoms::~QCcoms() {

}

void QCcoms::initialize() {

    rxtxHandler->startRX();
    taskActive = true;
}

void QCcoms::update() {

    /* reset transmission led */
    rxtxHandler->setLED(false);

    if (bufferOverrun > 0) {
        /* continue transmission if needed to be splitted 
         * RX is still blocked
         */
        answerCusomFrame();
    } else if ((*rxtxHandler->numberReceivedData > 1)
            && rxtxHandler->receptionComplete) {
        /* go trough fist received byte */
        switch (rxtxHandler->RxBuffer[1]) {
//        case QC_CMD_LOOP:
//            /* loop received stuff */
//            loopback();
//            break;
        case QC_CMD_SEND_CUSTOM_FRAME:
            answerCusomFrame();
            break;
        case QC_CMD_GLOBAL_FLAGS:
            /* create new frame with global flags */
            rxtxHandler->RxBuffer[2] = DATA_ID_GLOBAL_FLAGS;
            rxtxHandler->RxBuffer[3] = DATA_ID_EOF;
            *rxtxHandler->numberReceivedData = 4;
            answerCusomFrame();
            break;
        case QC_CMD_SET_FLIGHT_LED_PATTERN:
            /* change flight led pattern */
            flightLEDs->setLEDpattern(
                    (uint16_t) ((rxtxHandler->RxBuffer[3] << 8)
                            | rxtxHandler->RxBuffer[4]));
            sendConfirmation();
            break;
        case QC_CMD_CONFIG_MODE:
            /* trigger quadrocopter reset */

            /* switch mode */
            if (*rxtxHandler->numberReceivedData > 2) {
                if (rxtxHandler->RxBuffer[2] == 1) {
                    /* enter config mode */
                    status->globalFlags.resetToConfig = true;
                } else if (rxtxHandler->RxBuffer[2] == 0) {
                    /* leave config mode */
                    status->globalFlags.resetToFlight = true;
                }
            } else {
                /* toggle mode */
                if (status->globalFlags.configMode) {
                    status->globalFlags.resetToFlight = true;
                } else {
                    status->globalFlags.resetToConfig = true;
                }
            }
            /* request reset */
            status->globalFlags.resetRequested = true;
            sendConfirmation();
            break;
        case QC_CMD_READ_CONFIG:
            /* create custom frame */
            /* pid xy */
            rxtxHandler->RxBuffer[2] = DATA_ID_PID_ANGLE_XY;
            /* pid z  */
            rxtxHandler->RxBuffer[3] = DATA_ID_PID_ROT_Z;
            /* comp filter */
            rxtxHandler->RxBuffer[4] = DATA_ID_COMP_FILTER;
            /* pid accel  */
            rxtxHandler->RxBuffer[5] = DATA_ID_PID_ACCEL;
            /* pid vel */
            rxtxHandler->RxBuffer[6] = DATA_ID_PID_VEL;
            /* qc settings*/
            rxtxHandler->RxBuffer[7] = DATA_ID_QC_SETTINGS;

            rxtxHandler->RxBuffer[8] = DATA_ID_EOF;
            *rxtxHandler->numberReceivedData = 9;
            answerCusomFrame();
            break;
        case QC_CMD_RESET:
            /* trigger quadrocopter reset */
            /* keep current mode */
            if (status->globalFlags.flightMode) {
                status->globalFlags.resetToFlight = true;
            } else {
                status->globalFlags.resetToConfig = true;
            }
            /* request reset */
            status->globalFlags.resetRequested = true;
            sendConfirmation();
            break;
        default:
            /* ignore unknown command */

            if (status->globalFlags.configMode) {
                /* falls im config mode
                 *
                 */
                decodeConfigMSG();
            }
            break;
        }
    }
    if (bufferOverrun == 0) {
        /* previous request completed 
         * start RX again 
         */
        rxtxHandler->startRX();
    }
}

void QCcoms::reset() {

    rxtxHandler->reset();
}

void QCcoms::answerCusomFrame() {

    uint16_t txBufferPos = 0;
    bufferOverrun = 0;
    /* start loop at beginning of data ids or last position before overrun */
    for (uint16_t i = customFramePos; i < *rxtxHandler->numberReceivedData;
            i++) {
        /* add requested values
         * but check fist if tx buffer has enough space
         */
        switch (rxtxHandler->RxBuffer[i]) {
        case DATA_ID_GYRO:
            if (!checkTXBufferOverrun(txBufferPos, 12)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->rate.x);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->rate.y);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->rate.z);
            }
            break;
        case DATA_ID_ACCEL:
            if (!checkTXBufferOverrun(txBufferPos, 12)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->accel.x);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->accel.y);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->accel.z);
            }
            break;
        case DATA_ID_MAGNETOMETER:
            if (!checkTXBufferOverrun(txBufferPos, 12)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->magnetfield.x);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->magnetfield.y);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->magnetfield.z);
            }
            break;
        case DATA_ID_ANGLE:
            if (!checkTXBufferOverrun(txBufferPos, 12)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->angle.x);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->angle.y);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->angle.z);
            }
            break;
        case DATA_ID_ANGLE_SP:
            if (!checkTXBufferOverrun(txBufferPos, 12)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->angleSetpoint.x);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->angleSetpoint.y);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->angleSetpoint.z);
            }
            break;
        case DATA_ID_HOR_ACCEL:
            if (!checkTXBufferOverrun(txBufferPos, 12)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->horizontalAcceleration.x);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->horizontalAcceleration.y);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->horizontalAcceleration.z);
            }
            break;
        case DATA_ID_ACCEL_SP:
            if (!checkTXBufferOverrun(txBufferPos, 12)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->accelerationSetpoint.x);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->accelerationSetpoint.y);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->accelerationSetpoint.z);
            }
            break;
        case DATA_ID_VELOCITY:
            if (!checkTXBufferOverrun(txBufferPos, 12)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->velocity.x);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->velocity.y);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->velocity.z);
            }
            break;
        case DATA_ID_VELOCITY_SP:
            if (!checkTXBufferOverrun(txBufferPos, 12)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->velocitySetpoint.x);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->velocitySetpoint.y);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->velocitySetpoint.z);
            }
            break;
        case DATA_ID_HEIGHT:
            if (!checkTXBufferOverrun(txBufferPos, 12)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->height);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->height_rel);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->d_h);
            }
            break;
        case DATA_ID_RC:
            if (!checkTXBufferOverrun(txBufferPos, 22)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->rcSignalNick);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->rcSignalRoll);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->rcSignalYaw);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->rcSignalThrottle);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->rcSignalLinPoti);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->rcSignalEnable);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->rcSignalSwitch);
            }
            break;
        case DATA_ID_MOTOR_SP:
            if (!checkTXBufferOverrun(txBufferPos, 12)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->motorSetpoint.x);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->motorSetpoint.y);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->motorSetpoint.z);
            }
            break;
        case DATA_ID_MOTOR:
            if (!checkTXBufferOverrun(txBufferPos, 16)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->motorValues[0]);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->motorValues[1]);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->motorValues[2]);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->motorValues[3]);
            }
            break;
        case DATA_ID_CPU:
            if (!checkTXBufferOverrun(txBufferPos, 4)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->cpuLoad);
            }
            break;
        case DATA_ID_AKKU:
            if (!checkTXBufferOverrun(txBufferPos, 4)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->akkuVoltage);
            }
            break;
        case DATA_ID_TEMP:
            if (!checkTXBufferOverrun(txBufferPos, 4)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->temp);
            }
            break;
        case DATA_ID_UPTIME:
            if (!checkTXBufferOverrun(txBufferPos, 4)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->uptime);
            }
            break;
        case DATA_ID_GPS_LLH:
            if (!checkTXBufferOverrun(txBufferPos, 56)) {
                /* postition llh with accuracy*/
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.llh_data.lat);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.llh_data.lon);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.llh_data.h);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.llh_data.hMSL);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.llh_data.vAcc);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.llh_data.hAcc);
                /* ned*/
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.ned_data.vN);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.ned_data.vE);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.ned_data.vD);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.ned_data.speed);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.ned_data.gSpeed);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.ned_data.sAcc);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.ned_data.heading);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.ned_data.cAcc);

            }
            break;
        case DATA_ID_GPS_ECEF:
            if (!checkTXBufferOverrun(txBufferPos, 32)) {
                /* position ecef */
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.ecef_data.x);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.ecef_data.y);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.ecef_data.z);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.ecef_data.vx);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.ecef_data.vy);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.ecef_data.vz);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.ecef_data.pAcc);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.ecef_data.sAcc);
            }
            break;
        case DATA_ID_GPS_DOP:
            if (!checkTXBufferOverrun(txBufferPos, 14)) {
                /* dilution of precission*/
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.dop.pDOP);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.dop.gDOP);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.dop.tDOP);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.dop.vDOP);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.dop.hDOP);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.dop.nDOP);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.dop.eDOP);
            }
            break;
        case DATA_ID_GPS_TIME:
            if (!checkTXBufferOverrun(txBufferPos, 15)) {
                /* gps time of week*/
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.iTOW);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.gpsWeek);
                /* gps time */
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.time.hours);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.time.minutes);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.time.seconds);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.time.hundredths);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.time.validity);
                /* gps date*/
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.date.year);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.date.month);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.date.day);
            }
            break;
        case DATA_ID_GPS_FIX:
            /* fix + flags*/
            if (!checkTXBufferOverrun(txBufferPos, 5)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        (uint8_t) status->gpsData.gpsFix);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.FixStatus);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.numSV);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->gpsData.navStatusFlags);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        (uint8_t) status->gpsData.psmState);
            }
            break;
        case DATA_ID_COMP_FILTER:
            if (!checkTXBufferOverrun(txBufferPos, 8)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->filterCoefficientXY);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->filterCoefficientZ);
            }
            break;
        case DATA_ID_PID_ANGLE_XY:
            if (!checkTXBufferOverrun(txBufferPos, 20)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsAngleXY.p);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsAngleXY.i);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsAngleXY.d);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsAngleXY.gain);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsAngleXY.scaleSetPoint);
            }
            break;
        case DATA_ID_PID_ROT_Z:
            if (!checkTXBufferOverrun(txBufferPos, 20)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsRotationZ.p);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsRotationZ.i);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsRotationZ.d);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsRotationZ.gain);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsRotationZ.scaleSetPoint);
            }
            break;
        case DATA_ID_PID_VEL:
            if (!checkTXBufferOverrun(txBufferPos, 20)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsVelocity.p);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsVelocity.i);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsVelocity.d);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsVelocity.gain);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsVelocity.scaleSetPoint);
            }
            break;
        case DATA_ID_PID_ACCEL:
            if (!checkTXBufferOverrun(txBufferPos, 20)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsAcceleration.p);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsAcceleration.i);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsAcceleration.d);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsAcceleration.gain);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->pidSettingsAcceleration.scaleSetPoint);
            }
            break;
        case DATA_ID_QC_SETTINGS:
            if (!checkTXBufferOverrun(txBufferPos, 4)) {
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->qcSettings.enableBuzzerWarningLowVoltage);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->qcSettings.enableBuzzerWarningRCLost);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->qcSettings.enableFlightLeds);
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        status->qcSettings.enableMotors);
            }
            break;
        case DATA_ID_GLOBAL_FLAGS:
            /* global flags übertragung */
            if (!checkTXBufferOverrun(txBufferPos, 4)) {
                uint32_t tmp = 0;
                /* shitcode */
                if (status->globalFlags.flightMode) {
                    tmp |= FLIGHT_MODE_FLAG;
                } else {
                    tmp |= CONFIG_MODE_FLAG;
                }
                if (status->globalFlags.error) {
                    tmp |= ERROR_FLAG;
                }
                if (status->globalFlags.usbError) {
                    tmp |= USB_ERROR_FLAG;
                }
                if (status->globalFlags.cpuOverload) {
                    tmp |= CPU_OVERLOAD_FLAG;
                }
                if (status->globalFlags.noRCSignal) {
                    tmp |= NO_RC_SIGNAL_FLAG;
                }
                if (status->globalFlags.lowVoltage) {
                    tmp |= LOW_VOLTAGE_FLAG;
                }
                if (status->globalFlags.MPU9150ok) {
                    tmp |= MPU9150_OK_FLAG;
                }
                if (status->globalFlags.BMP180ok) {
                    tmp |= BMP180_OK_FLAG;
                }
                if (status->globalFlags.RCReceiverOk) {
                    tmp |= RC_RECEIVER_OK_FLAG;
                }
                if (status->globalFlags.EEPROMok) {
                    tmp |= EEPROM_OK_FLAG;
                }
                if (status->globalFlags.emergency) {
                    tmp |= EMERGENCY_FLAG;
                }
                txBufferPos = fillBuffer(rxtxHandler->TxBuffer, txBufferPos,
                        tmp);
            }
            break;
        case DATA_ID_EOF:
            /* end of frame
             * reset position for next request
             * send buffer */
            bufferOverrun = 0;
            customFramePos = 2;
            rxtxHandler->sendTXBuffer(txBufferPos);
            break;
        default:
            /* unknown id */
            break;

        }
        if (bufferOverrun == 1) {
            /* Buffer Overrun
             * save position in custom frame
             * send package 
             * next update will continue sending data
             */
            customFramePos = i;
            rxtxHandler->sendTXBuffer(txBufferPos);
            return;
        }
    }
}

void QCcoms::sendConfirmation() {
    /* send last command as confirmation */
    rxtxHandler->TxBuffer[0] = rxtxHandler->RxBuffer[1];
    rxtxHandler->sendTXBuffer(1);
}

void QCcoms::decodeConfigMSG() {
    /* only in config mode */
    if (status->globalFlags.configMode) {

        switch (rxtxHandler->RxBuffer[1]) {
        case QC_CMD_READ_CONFIG:
            /* create custom frame */
            /* pid xy */
            rxtxHandler->RxBuffer[2] = DATA_ID_PID_ANGLE_XY;
            /* pid z  */
            rxtxHandler->RxBuffer[3] = DATA_ID_PID_ROT_Z;
            /* comp filter */
            rxtxHandler->RxBuffer[4] = DATA_ID_COMP_FILTER;
            /* pid accel  */
            rxtxHandler->RxBuffer[5] = DATA_ID_PID_ACCEL;
            /* pid vel */
            rxtxHandler->RxBuffer[6] = DATA_ID_PID_VEL;
            /* qc settings*/
            rxtxHandler->RxBuffer[7] = DATA_ID_QC_SETTINGS;

            rxtxHandler->RxBuffer[8] = DATA_ID_EOF;
            *rxtxHandler->numberReceivedData = 9;
            answerCusomFrame();
            break;

        case QC_CMD_UPDATE_CONFIG:
            updateConfig();
            break;
        case QC_CMD_EEPROM_READ_BYTE:
            readEEPROM(1);
            break;
        case QC_CMD_EEPROM_READ_2BYTES:
            readEEPROM(2);
            break;
        case QC_CMD_EEPROM_READ_4BYTES:
            readEEPROM(4);
            break;
        case QC_CMD_EEPROM_WRITE_BYTE:
            writeEEPROM(1);
            break;
        case QC_CMD_EEPROM_WRITE_2BYTES:
            writeEEPROM(2);
            break;
        case QC_CMD_EEPROM_WRITE_4BYTES:
            writeEEPROM(4);
            break;

        case QC_CMD_QUADROCONFIG:
            if (*rxtxHandler->numberReceivedData == 3) {
                /* check and set settings */
                if (rxtxHandler->RxBuffer[2] & QUADROCONFIG_ENABLE_LOW_VOLT) {
                    status->qcSettings.enableBuzzerWarningLowVoltage = 1;
                } else {
                    status->qcSettings.enableBuzzerWarningLowVoltage = 0;
                }
                if (rxtxHandler->RxBuffer[2] & QUADROCONFIG_ENABLE_RC_LOST) {
                    status->qcSettings.enableBuzzerWarningRCLost = 1;
                } else {
                    status->qcSettings.enableBuzzerWarningRCLost = 0;
                }
                if (rxtxHandler->RxBuffer[2] & QUADROCONFIG_ENABLE_FLIGHTLED) {
                    status->qcSettings.enableFlightLeds = 1;
                } else {
                    status->qcSettings.enableFlightLeds = 0;
                }
                if (rxtxHandler->RxBuffer[2] & QUADROCONFIG_ENABLE_MOTORS) {
                    status->qcSettings.enableMotors = 1;
                } else {
                    status->qcSettings.enableMotors = 0;
                }

                /* send confirmation */
                sendConfirmation();
            } else {
                /* reply with current setting*/
                rxtxHandler->TxBuffer[0] = rxtxHandler->RxBuffer[1];
                rxtxHandler->TxBuffer[1] =
                        (uint8_t) ((status->qcSettings.enableMotors
                                * QUADROCONFIG_ENABLE_MOTORS)
                                | (status->qcSettings.enableFlightLeds
                                        * QUADROCONFIG_ENABLE_FLIGHTLED)
                                | (status->qcSettings.enableBuzzerWarningRCLost
                                        * QUADROCONFIG_ENABLE_RC_LOST)
                                | (status->qcSettings.enableBuzzerWarningLowVoltage
                                        * QUADROCONFIG_ENABLE_LOW_VOLT));
                rxtxHandler->sendTXBuffer(2);
            }
            break;
        case QC_CMD_RELOAD_CONFIG_FROM_EEPROM:
            confReader->loadConfiguration(status);
            /* send confirmation */
            sendConfirmation();
            break;
        case QC_CMD_SAVE_CONFIG_TO_EEPROM:
            confReader->saveConfiguration(status);
            /* send confirmation */
            sendConfirmation();
            break;
        case QC_CMD_RESTORE_HARDCODED_CONFIG:
            status->restoreConfig();
            /* send confirmation */
            sendConfirmation();
            break;
        default:
            break;
        }
    }
}

void QCcoms::updateConfig() {
    /* set position to 1
     * select first idetifier
     */
    uint16_t txBufferPos = 2;
    while (txBufferPos < *rxtxHandler->numberReceivedData) {

        switch (rxtxHandler->RxBuffer[txBufferPos]) {
        /* each case:
         *   jump to first value:
         *   txBufferPos++
         *   then set values with byteToFloat function
         *   byteToFloat increases txBufferPos by 4
         */
        case DATA_ID_PID_ANGLE_XY:
            txBufferPos++;
            status->pidSettingsAngleXY.p = byteToFloat(rxtxHandler->RxBuffer,
                    &txBufferPos);
            status->pidSettingsAngleXY.i = byteToFloat(rxtxHandler->RxBuffer,
                    &txBufferPos);
            status->pidSettingsAngleXY.d = byteToFloat(rxtxHandler->RxBuffer,
                    &txBufferPos);
            status->pidSettingsAngleXY.gain = byteToFloat(rxtxHandler->RxBuffer,
                    &txBufferPos);
            status->pidSettingsAngleXY.scaleSetPoint = byteToFloat(
                    rxtxHandler->RxBuffer, &txBufferPos);
            break;
        case DATA_ID_PID_ROT_Z:
            txBufferPos++;
            status->pidSettingsRotationZ.p = byteToFloat(rxtxHandler->RxBuffer,
                    &txBufferPos);
            status->pidSettingsRotationZ.i = byteToFloat(rxtxHandler->RxBuffer,
                    &txBufferPos);
            status->pidSettingsRotationZ.d = byteToFloat(rxtxHandler->RxBuffer,
                    &txBufferPos);
            status->pidSettingsRotationZ.gain = byteToFloat(
                    rxtxHandler->RxBuffer, &txBufferPos);
            status->pidSettingsRotationZ.scaleSetPoint = byteToFloat(
                    rxtxHandler->RxBuffer, &txBufferPos);
            break;
        case DATA_ID_COMP_FILTER:
            txBufferPos++;
            status->filterCoefficientXY = byteToFloat(rxtxHandler->RxBuffer,
                    &txBufferPos);
            status->filterCoefficientZ = byteToFloat(rxtxHandler->RxBuffer,
                    &txBufferPos);
            break;
        case DATA_ID_PID_ACCEL:
            txBufferPos++;
            status->pidSettingsAcceleration.p = byteToFloat(
                    rxtxHandler->RxBuffer, &txBufferPos);
            status->pidSettingsAcceleration.i = byteToFloat(
                    rxtxHandler->RxBuffer, &txBufferPos);
            status->pidSettingsAcceleration.d = byteToFloat(
                    rxtxHandler->RxBuffer, &txBufferPos);
            status->pidSettingsAcceleration.gain = byteToFloat(
                    rxtxHandler->RxBuffer, &txBufferPos);
            status->pidSettingsAcceleration.scaleSetPoint = byteToFloat(
                    rxtxHandler->RxBuffer, &txBufferPos);
            break;
        case DATA_ID_PID_VEL:
            txBufferPos++;
            status->pidSettingsVelocity.p = byteToFloat(rxtxHandler->RxBuffer,
                    &txBufferPos);
            status->pidSettingsVelocity.i = byteToFloat(rxtxHandler->RxBuffer,
                    &txBufferPos);
            status->pidSettingsVelocity.d = byteToFloat(rxtxHandler->RxBuffer,
                    &txBufferPos);
            status->pidSettingsVelocity.gain = byteToFloat(
                    rxtxHandler->RxBuffer, &txBufferPos);
            status->pidSettingsVelocity.scaleSetPoint = byteToFloat(
                    rxtxHandler->RxBuffer, &txBufferPos);
            break;
        case DATA_ID_QC_SETTINGS:
            txBufferPos++;
            status->qcSettings.enableBuzzerWarningRCLost =
                    rxtxHandler->RxBuffer[txBufferPos++];
            status->qcSettings.enableBuzzerWarningLowVoltage =
                    rxtxHandler->RxBuffer[txBufferPos++];
            status->qcSettings.enableFlightLeds =
                    rxtxHandler->RxBuffer[txBufferPos++];
            status->qcSettings.enableMotors =
                    rxtxHandler->RxBuffer[txBufferPos++];
            break;
        case DATA_ID_EOF:
            txBufferPos++;
            /* send Confirmation*/
            sendConfirmation();
            break;
        default:
            /* error*/
            break;
        }
    }
}

uint16_t QCcoms::fillBuffer(uint8_t* buffer, uint16_t pos, float var) {
    uint8_t* tmp = (uint8_t*) &var;
    buffer[pos++] = *tmp++;
    buffer[pos++] = *(tmp++);
    buffer[pos++] = *(tmp++);
    buffer[pos++] = *(tmp);
    return pos;
}

uint16_t QCcoms::fillBuffer(uint8_t* buffer, uint16_t pos, uint32_t var) {
    uint8_t* tmp = (uint8_t*) &var;
    buffer[pos++] = *tmp++;
    buffer[pos++] = *(tmp++);
    buffer[pos++] = *(tmp++);
    buffer[pos++] = *(tmp);
    return pos;
}

uint16_t QCcoms::fillBuffer(uint8_t* buffer, uint16_t pos, int32_t var) {
    uint8_t* tmp = (uint8_t*) &var;
    buffer[pos++] = *tmp++;
    buffer[pos++] = *(tmp++);
    buffer[pos++] = *(tmp++);
    buffer[pos++] = *(tmp);
    return pos;
}

uint16_t QCcoms::fillBuffer(uint8_t* buffer, uint16_t pos, int16_t var) {
    uint8_t* tmp = (uint8_t*) &var;
    buffer[pos++] = *tmp++;
    buffer[pos++] = *(tmp);
    return pos;
}

uint16_t QCcoms::fillBuffer(uint8_t* buffer, uint16_t pos, uint16_t var) {
    uint8_t* tmp = (uint8_t*) &var;
    buffer[pos++] = *tmp++;
    buffer[pos++] = *(tmp);
    return pos;
}

uint16_t QCcoms::fillBuffer(uint8_t* buffer, uint16_t pos, int8_t var) {
    buffer[pos++] = var;
    return pos;
}

uint16_t QCcoms::fillBuffer(uint8_t* buffer, uint16_t pos, uint8_t var) {
    buffer[pos++] = var;
    return pos;
}

void QCcoms::readEEPROM(uint8_t byteCount) {

    switch (byteCount) {
    case 1: {
        uint8_t tmp;
        confReader->loadVariable(&tmp,
                (uint16_t) (rxtxHandler->RxBuffer[2] << 8
                        | rxtxHandler->RxBuffer[3]));
        rxtxHandler->TxBuffer[0] = tmp;
        break;
    }
    case 2: {
        uint16_t tmp;
        confReader->loadVariable(&tmp,
                (uint16_t) (rxtxHandler->RxBuffer[2] << 8
                        | rxtxHandler->RxBuffer[3]));
        rxtxHandler->TxBuffer[0] = (uint8_t) ((tmp >> 8) & 0xff);
        rxtxHandler->TxBuffer[1] = (uint8_t) (tmp & 0xff);
        break;
    }
    case 4: {
        uint32_t tmp;
        confReader->loadVariable(&tmp,
                (uint16_t) (rxtxHandler->RxBuffer[2] << 8
                        | rxtxHandler->RxBuffer[3]));
        rxtxHandler->TxBuffer[0] = (uint8_t) ((tmp >> 24) & 0xff);
        rxtxHandler->TxBuffer[1] = (uint8_t) ((tmp >> 16) & 0xff);
        rxtxHandler->TxBuffer[2] = (uint8_t) ((tmp >> 8) & 0xff);
        rxtxHandler->TxBuffer[3] = (uint8_t) (tmp & 0xff);
        break;
    }
    }
    /* send eeprom content*/
    rxtxHandler->sendTXBuffer(byteCount);
}

void QCcoms::writeEEPROM(uint8_t byteCount) {
    /* direct write to eeprom */
    switch (byteCount) {
    case 1: {
        uint8_t tmp = rxtxHandler->RxBuffer[4];
        confReader->saveVariable(&tmp,
                (uint16_t) (rxtxHandler->RxBuffer[2] << 8
                        | rxtxHandler->RxBuffer[3]), 0);
        break;
    }
    case 2: {
        uint16_t tmp = (uint16_t) ((rxtxHandler->RxBuffer[4] << 8)
                | rxtxHandler->RxBuffer[5]);
        confReader->saveVariable(&tmp,
                (uint16_t) (rxtxHandler->RxBuffer[2] << 8
                        | rxtxHandler->RxBuffer[3]), 0);
        break;
    }

    case 4: {
        uint32_t tmp = ((rxtxHandler->RxBuffer[4] << 24)
                | (rxtxHandler->RxBuffer[5] << 16)
                | (rxtxHandler->RxBuffer[6] << 8) | rxtxHandler->RxBuffer[7]);
        confReader->saveVariable(&tmp,
                (uint16_t) (rxtxHandler->RxBuffer[2] << 8
                        | rxtxHandler->RxBuffer[3]), 0);
        break;
    }
    }
    /* send confirmation */
    sendConfirmation();
}

uint16_t QCcoms::checkTXBufferOverrun(uint16_t currentPos, uint16_t dataToAdd) {
    if ((currentPos + dataToAdd) < rxtxHandler->rxtxBuffersize) {
        bufferOverrun = 0;
        return 0;
    } else {
        bufferOverrun = 1;
        return 1;
    }
}

void QCcoms::loopback() {
    /* copy input buffer into output buffer
     * and send
     */
    memcpy(rxtxHandler->TxBuffer, rxtxHandler->RxBuffer,
            *rxtxHandler->numberReceivedData);
    rxtxHandler->sendTXBuffer(*rxtxHandler->numberReceivedData);
}
