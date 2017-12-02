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
    customFramePos = 1;
}

QCcoms::~QCcoms() {

}

void QCcoms::initialize() {

    rxtxHandler->startRX();
    SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}

void QCcoms::update() {

    /* reset transmission led */
    rxtxHandler->setLED(false);

    if (bufferOverrun > 0) {
        /* continue transmission if needed to be splitted 
         * RX is still blocked
         */
        answerCusomFrame();
    } else if (*rxtxHandler->numberReceivedData > 0) {
        /* go trough fist received byte */
        switch (rxtxHandler->RxBuffer[0]) {
        case QC_CMD_LOOP:
            /* loop received stuff */
            loopback();
            break;
        case QC_CMD_SEND_CUSTOM_FRAME:
            answerCusomFrame();
            break;
        case QC_CMD_GLOBAL_FLAGS:
            /* create new frame with global flags */
            rxtxHandler->RxBuffer[1] = DATA_ID_GLOBAL_FLAGS;
            rxtxHandler->RxBuffer[2] = DATA_ID_EOF;
            *rxtxHandler->numberReceivedData = 3;
            answerCusomFrame();
            break;
        case QC_CMD_SET_FLIGHT_LED_PATTERN:
            /* change flight led pattern */
            flightLEDs->setLEDpattern(
                    (uint16_t) (rxtxHandler->RxBuffer[1] << 8)
                            | rxtxHandler->RxBuffer[2]);
            sendConfirmation();
            break;
        case QC_CMD_CONFIG_MODE:
            /*entering config mode */
            /* trigger quadrocopter reset */
            /* switch current mode */
            if (GET_FLAG(status->globalFlags, CONFIG_MODE_FLAG)) {
                SET_FLAG(status->globalFlags, RESET_TO_FLIGHT);
            } else {
                SET_FLAG(status->globalFlags, RESET_TO_CONFIG);
            }
            /* request reset */
            SET_FLAG(status->globalFlags, RESET_REQUEST);
            sendConfirmation();
            break;
        case QC_CMD_READ_CONFIG:
            /* create custom frame */
            /* pid xy */
            rxtxHandler->RxBuffer[1] = DATA_ID_PID_ANGLE_XY;
            /* pid z  */
            rxtxHandler->RxBuffer[2] = DATA_ID_PID_ROT_Z;
            /* comp filter */
            rxtxHandler->RxBuffer[3] = DATA_ID_COMP_FILTER;
            /* pid accel  */
            rxtxHandler->RxBuffer[4] = DATA_ID_PID_ACCEL;
            /* pid vel */
            rxtxHandler->RxBuffer[5] = DATA_ID_PID_VEL;
            /* qc settings*/
            rxtxHandler->RxBuffer[6] = DATA_ID_QC_SETTINGS;

            rxtxHandler->RxBuffer[7] = DATA_ID_EOF;
            *rxtxHandler->numberReceivedData = 8;
            answerCusomFrame();
            break;
        case QC_CMD_RESET:
            /* trigger quadrocopter reset */
            /* keep current mode */
            if (GET_FLAG(status->globalFlags, FLIGHT_MODE_FLAG)) {
                SET_FLAG(status->globalFlags, RESET_TO_FLIGHT);
            } else {
                SET_FLAG(status->globalFlags, RESET_TO_CONFIG);
            }
            /* request reset */
            SET_FLAG(status->globalFlags, RESET_REQUEST);
            sendConfirmation();
            break;
        default:

            /* falls im config mode
             *
             */
            if (GET_FLAG(status->globalFlags, CONFIG_MODE_FLAG)) {
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

    uint16_t bufferPos = 0;
    bufferOverrun = 0;
    /* start loop at beginning of data ids or last position before overrun */
    for (uint16_t i = customFramePos; i < *rxtxHandler->numberReceivedData;
            i++) {
        /* add requested values
         * but check fist if tx buffer has enough space
         */
        switch (rxtxHandler->RxBuffer[i]) {
        case DATA_ID_GYRO:
            if (!checkTXBufferOverrun(bufferPos, 12)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->rate.x);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->rate.y);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->rate.z);
            }
            break;
        case DATA_ID_ACCEL:
            if (!checkTXBufferOverrun(bufferPos, 12)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->accel.x);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->accel.y);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->accel.z);
            }
            break;
        case DATA_ID_MAGNETOMETER:
            if (!checkTXBufferOverrun(bufferPos, 12)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->magnetfield.x);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->magnetfield.y);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->magnetfield.z);
            }
            break;
        case DATA_ID_ANGLE:
            if (!checkTXBufferOverrun(bufferPos, 12)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->angle.x);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->angle.y);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->angle.z);
            }
            break;
        case DATA_ID_ANGLE_SP:
            if (!checkTXBufferOverrun(bufferPos, 12)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->angleSetpoint.x);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->angleSetpoint.y);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->angleSetpoint.z);
            }
            break;
        case DATA_ID_HOR_ACCEL:
            if (!checkTXBufferOverrun(bufferPos, 12)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->horizontalAcceleration.x);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->horizontalAcceleration.y);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->horizontalAcceleration.z);
            }
            break;
        case DATA_ID_ACCEL_SP:
            if (!checkTXBufferOverrun(bufferPos, 12)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->accelerationSetpoint.x);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->accelerationSetpoint.y);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->accelerationSetpoint.z);
            }
            break;
        case DATA_ID_VELOCITY:
            if (!checkTXBufferOverrun(bufferPos, 12)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->velocity.x);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->velocity.y);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->velocity.z);
            }
            break;
        case DATA_ID_VELOCITY_SP:
            if (!checkTXBufferOverrun(bufferPos, 12)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->velocitySetpoint.x);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->velocitySetpoint.y);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->velocitySetpoint.z);
            }
            break;
        case DATA_ID_HEIGHT:
            if (!checkTXBufferOverrun(bufferPos, 12)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->height);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->height_rel);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->d_h);
            }
            break;
        case DATA_ID_RC:
            if (!checkTXBufferOverrun(bufferPos, 22)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->rcSignalNick);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->rcSignalRoll);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->rcSignalYaw);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->rcSignalThrottle);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->rcSignalLinPoti);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->rcSignalEnable);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->rcSignalSwitch);
            }
            break;
        case DATA_ID_MOTOR_SP:
            if (!checkTXBufferOverrun(bufferPos, 12)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->motorSetpoint.x);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->motorSetpoint.y);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->motorSetpoint.z);
            }
            break;
        case DATA_ID_MOTOR:
            if (!checkTXBufferOverrun(bufferPos, 16)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->motorValues[0]);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->motorValues[1]);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->motorValues[2]);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->motorValues[3]);
            }
            break;
        case DATA_ID_CPU:
            if (!checkTXBufferOverrun(bufferPos, 4)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->cpuLoad);
            }
            break;
        case DATA_ID_AKKU:
            if (!checkTXBufferOverrun(bufferPos, 4)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->akkuVoltage);
            }
            break;
        case DATA_ID_TEMP:
            if (!checkTXBufferOverrun(bufferPos, 4)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->temp);
            }
            break;
        case DATA_ID_UPTIME:
            if (!checkTXBufferOverrun(bufferPos, 4)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->uptime);
            }
            break;
        case DATA_ID_GPS_LLH:
            if (!checkTXBufferOverrun(bufferPos, 56)) {
                /* postition llh with accuracy*/
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.llh_data.lat);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.llh_data.lon);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.llh_data.h);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.llh_data.hMSL);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.llh_data.vAcc);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.llh_data.hAcc);
                /* ned*/
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.ned_data.vN);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.ned_data.vE);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.ned_data.vD);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.ned_data.speed);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.ned_data.gSpeed);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.ned_data.sAcc);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.ned_data.heading);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.ned_data.cAcc);

            }
            break;
        case DATA_ID_GPS_ECEF:
            if (!checkTXBufferOverrun(bufferPos, 32)) {
                /* position ecef */
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.ecef_data.x);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.ecef_data.y);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.ecef_data.z);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.ecef_data.vx);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.ecef_data.vy);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.ecef_data.vz);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.ecef_data.pAcc);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.ecef_data.sAcc);
            }
            break;
        case DATA_ID_GPS_DOP:
            if (!checkTXBufferOverrun(bufferPos, 14)) {
                /* dilution of precission*/
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.dop.pDOP);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.dop.gDOP);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.dop.tDOP);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.dop.vDOP);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.dop.hDOP);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.dop.nDOP);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.dop.eDOP);
            }
            break;
        case DATA_ID_GPS_TIME:
            if (!checkTXBufferOverrun(bufferPos, 15)) {
                /* gps time of week*/
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.iTOW);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.gpsWeek);
                /* gps time */
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.time.hours);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.time.minutes);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.time.seconds);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.time.hundredths);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.time.validity);
                /* gps date*/
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.date.year);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.date.month);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.date.day);
            }
            break;
        case DATA_ID_GPS_FIX:
            /* fix + flags*/
            if (!checkTXBufferOverrun(bufferPos, 5)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        (uint8_t) status->gpsData.gpsFix);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.FixStatus);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.numSV);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->gpsData.navStatusFlags);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        (uint8_t) status->gpsData.psmState);
            }
            break;
        case DATA_ID_COMP_FILTER:
            if (!checkTXBufferOverrun(bufferPos, 8)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->filterCoefficientXY);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->filterCoefficientZ);
            }
            break;
        case DATA_ID_PID_ANGLE_XY:
            if (!checkTXBufferOverrun(bufferPos, 20)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsAngleXY.p);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsAngleXY.i);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsAngleXY.d);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsAngleXY.gain);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsAngleXY.scaleSetPoint);
            }
            break;
        case DATA_ID_PID_ROT_Z:
            if (!checkTXBufferOverrun(bufferPos, 20)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsRotationZ.p);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsRotationZ.i);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsRotationZ.d);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsRotationZ.gain);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsRotationZ.scaleSetPoint);
            }
            break;
        case DATA_ID_PID_VEL:
            if (!checkTXBufferOverrun(bufferPos, 20)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsVelocity.p);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsVelocity.i);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsVelocity.d);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsVelocity.gain);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsVelocity.scaleSetPoint);
            }
            break;
        case DATA_ID_PID_ACCEL:
            if (!checkTXBufferOverrun(bufferPos, 20)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsAcceleration.p);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsAcceleration.i);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsAcceleration.d);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsAcceleration.gain);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->pidSettingsAcceleration.scaleSetPoint);
            }
            break;
        case DATA_ID_QC_SETTINGS:
            if (!checkTXBufferOverrun(bufferPos, 4)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->qcSettings.enableBuzzerWarningLowVoltage);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->qcSettings.enableBuzzerWarningRCLost);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->qcSettings.enableFlightLeds);
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->qcSettings.enableMotors);
            }
            break;
        case DATA_ID_GLOBAL_FLAGS:
            if (!checkTXBufferOverrun(bufferPos, 4)) {
                bufferPos = fillBuffer(rxtxHandler->TxBuffer, bufferPos,
                        status->globalFlags);
            }
            break;
        case DATA_ID_EOF:
            /* end of frame
             * reset position for next request
             * send buffer */
            bufferOverrun = 0;
            customFramePos = 1;
            rxtxHandler->sendTXBuffer(bufferPos);
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
            rxtxHandler->sendTXBuffer(bufferPos);
            return;
        }
    }
}

void QCcoms::sendConfirmation() {
    /* send last command as confirmation */
    rxtxHandler->TxBuffer[0] = rxtxHandler->RxBuffer[0];
    rxtxHandler->sendTXBuffer(1);
}

void QCcoms::decodeConfigMSG() {
    /* only in config mode */
    if (GET_FLAG(status->globalFlags, CONFIG_MODE_FLAG)) {

        switch (rxtxHandler->RxBuffer[0]) {
        case QC_CMD_READ_CONFIG:
            /* create custom frame */
            /* pid xy */
            rxtxHandler->RxBuffer[1] = DATA_ID_PID_ANGLE_XY;
            /* pid z  */
            rxtxHandler->RxBuffer[2] = DATA_ID_PID_ROT_Z;
            /* comp filter */
            rxtxHandler->RxBuffer[3] = DATA_ID_COMP_FILTER;
            /* pid accel  */
            rxtxHandler->RxBuffer[4] = DATA_ID_PID_ACCEL;
            /* pid vel */
            rxtxHandler->RxBuffer[5] = DATA_ID_PID_VEL;
            /* qc settings*/
            rxtxHandler->RxBuffer[6] = DATA_ID_QC_SETTINGS;

            rxtxHandler->RxBuffer[7] = DATA_ID_EOF;
            *rxtxHandler->numberReceivedData = 8;
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
            if (*rxtxHandler->numberReceivedData == 2) {
                /* check and set settings */
                if (rxtxHandler->RxBuffer[1] & QUADROCONFIG_ENABLE_LOW_VOLT) {
                    status->qcSettings.enableBuzzerWarningLowVoltage = 1;
                } else {
                    status->qcSettings.enableBuzzerWarningLowVoltage = 0;
                }
                if (rxtxHandler->RxBuffer[1] & QUADROCONFIG_ENABLE_RC_LOST) {
                    status->qcSettings.enableBuzzerWarningRCLost = 1;
                } else {
                    status->qcSettings.enableBuzzerWarningRCLost = 0;
                }
                if (rxtxHandler->RxBuffer[1] & QUADROCONFIG_ENABLE_FLIGHTLED) {
                    status->qcSettings.enableFlightLeds = 1;
                } else {
                    status->qcSettings.enableFlightLeds = 0;
                }
                if (rxtxHandler->RxBuffer[1] & QUADROCONFIG_ENABLE_MOTORS) {
                    status->qcSettings.enableMotors = 1;
                } else {
                    status->qcSettings.enableMotors = 0;
                }

                /* send confirmation */
                sendConfirmation();
            } else {
                /* reply with current setting*/
                rxtxHandler->TxBuffer[0] = rxtxHandler->RxBuffer[0];
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
    uint16_t bufferpos = 1;
    while (bufferpos < *rxtxHandler->numberReceivedData) {

        switch (rxtxHandler->RxBuffer[bufferpos]) {
        /* each case:
         *   jump to first value:
         *   bufferpos++
         *   then set values with byteToFloat function
         *   byteToFloat increases bufferpos by 4
         */
        case DATA_ID_PID_ANGLE_XY:
            bufferpos++;
            status->pidSettingsAngleXY.p = byteToFloat(rxtxHandler->RxBuffer,
                    &bufferpos);
            status->pidSettingsAngleXY.i = byteToFloat(rxtxHandler->RxBuffer,
                    &bufferpos);
            status->pidSettingsAngleXY.d = byteToFloat(rxtxHandler->RxBuffer,
                    &bufferpos);
            status->pidSettingsAngleXY.gain = byteToFloat(rxtxHandler->RxBuffer,
                    &bufferpos);
            status->pidSettingsAngleXY.scaleSetPoint = byteToFloat(
                    rxtxHandler->RxBuffer, &bufferpos);
            break;
        case DATA_ID_PID_ROT_Z:
            bufferpos++;
            status->pidSettingsRotationZ.p = byteToFloat(rxtxHandler->RxBuffer,
                    &bufferpos);
            status->pidSettingsRotationZ.i = byteToFloat(rxtxHandler->RxBuffer,
                    &bufferpos);
            status->pidSettingsRotationZ.d = byteToFloat(rxtxHandler->RxBuffer,
                    &bufferpos);
            status->pidSettingsRotationZ.gain = byteToFloat(
                    rxtxHandler->RxBuffer, &bufferpos);
            status->pidSettingsRotationZ.scaleSetPoint = byteToFloat(
                    rxtxHandler->RxBuffer, &bufferpos);
            break;
        case DATA_ID_COMP_FILTER:
            bufferpos++;
            status->filterCoefficientXY = byteToFloat(rxtxHandler->RxBuffer,
                    &bufferpos);
            status->filterCoefficientZ = byteToFloat(rxtxHandler->RxBuffer,
                    &bufferpos);
            break;
        case DATA_ID_PID_ACCEL:
            bufferpos++;
            status->pidSettingsAcceleration.p = byteToFloat(
                    rxtxHandler->RxBuffer, &bufferpos);
            status->pidSettingsAcceleration.i = byteToFloat(
                    rxtxHandler->RxBuffer, &bufferpos);
            status->pidSettingsAcceleration.d = byteToFloat(
                    rxtxHandler->RxBuffer, &bufferpos);
            status->pidSettingsAcceleration.gain = byteToFloat(
                    rxtxHandler->RxBuffer, &bufferpos);
            status->pidSettingsAcceleration.scaleSetPoint = byteToFloat(
                    rxtxHandler->RxBuffer, &bufferpos);
            break;
        case DATA_ID_PID_VEL:
            bufferpos++;
            status->pidSettingsVelocity.p = byteToFloat(rxtxHandler->RxBuffer,
                    &bufferpos);
            status->pidSettingsVelocity.i = byteToFloat(rxtxHandler->RxBuffer,
                    &bufferpos);
            status->pidSettingsVelocity.d = byteToFloat(rxtxHandler->RxBuffer,
                    &bufferpos);
            status->pidSettingsVelocity.gain = byteToFloat(
                    rxtxHandler->RxBuffer, &bufferpos);
            status->pidSettingsVelocity.scaleSetPoint = byteToFloat(
                    rxtxHandler->RxBuffer, &bufferpos);
            break;
        case DATA_ID_QC_SETTINGS:
            bufferpos++;
            status->qcSettings.enableBuzzerWarningRCLost =
                    rxtxHandler->RxBuffer[bufferpos++];
            status->qcSettings.enableBuzzerWarningLowVoltage =
                    rxtxHandler->RxBuffer[bufferpos++];
            status->qcSettings.enableFlightLeds =
                    rxtxHandler->RxBuffer[bufferpos++];
            status->qcSettings.enableMotors =
                    rxtxHandler->RxBuffer[bufferpos++];
            break;
        case DATA_ID_EOF:
            bufferpos++;
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
                (uint16_t) (rxtxHandler->RxBuffer[1] << 8
                        | rxtxHandler->RxBuffer[2]));
        rxtxHandler->TxBuffer[0] = tmp;
        break;
    }
    case 2: {
        uint16_t tmp;
        confReader->loadVariable(&tmp,
                (uint16_t) (rxtxHandler->RxBuffer[1] << 8
                        | rxtxHandler->RxBuffer[2]));
        rxtxHandler->TxBuffer[0] = (uint8_t) ((tmp >> 8) & 0xff);
        rxtxHandler->TxBuffer[1] = (uint8_t) (tmp & 0xff);
        break;
    }
    case 4: {
        uint32_t tmp;
        confReader->loadVariable(&tmp,
                (uint16_t) (rxtxHandler->RxBuffer[1] << 8
                        | rxtxHandler->RxBuffer[2]));
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
        uint8_t tmp = rxtxHandler->RxBuffer[3];
        confReader->saveVariable(&tmp,
                (uint16_t) (rxtxHandler->RxBuffer[1] << 8
                        | rxtxHandler->RxBuffer[2]), 0);
        break;
    }
    case 2: {
        uint16_t tmp = (uint16_t) ((rxtxHandler->RxBuffer[3] << 8)
                | rxtxHandler->RxBuffer[4]);
        confReader->saveVariable(&tmp,
                (uint16_t) (rxtxHandler->RxBuffer[1] << 8
                        | rxtxHandler->RxBuffer[2]), 0);
        break;
    }

    case 4: {
        uint32_t tmp = ((rxtxHandler->RxBuffer[3] << 24)
                | (rxtxHandler->RxBuffer[4] << 16)
                | (rxtxHandler->RxBuffer[5] << 8) | rxtxHandler->RxBuffer[6]);
        confReader->saveVariable(&tmp,
                (uint16_t) (rxtxHandler->RxBuffer[1] << 8
                        | rxtxHandler->RxBuffer[2]), 0);
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
