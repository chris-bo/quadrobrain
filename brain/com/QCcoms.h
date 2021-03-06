/*
 * QCcoms.h
 *
 *  Created on: Nov 22, 2017
 *      Author: chris
 */

#ifndef QCCOMS_H_
#define QCCOMS_H_

#include <core/Task.h>
#include <com/RxTxHandler.h>
#include <utility/ConfigReader.h>
#include <utility/FlightLED.h>
#include <string.h>

/**************************************************************************/

/*
 *  Protocoll structure
 * RX:
 * | number of following bytes | CMD | payload |
 *
 *  number of following bytes  = cmd + payload
 *
 */

/* resend received */
#define QC_CMD_LOOP				0x01

/* send global flags */
#define QC_CMD_GLOBAL_FLAGS		0x04

/*******************/
/* custom frame
 *
 * PC sends custom frame command and identifiers for requested data
 * terminated by one byte of DATA_ID_EOF
 *
 * RXTXHandler responds with requested data
 * if inserting data will cause buffer overrun. the remaining buffer
 * will be 0xFF
 *
 * Unknown data id will be ignored
 *
 */

#define QC_CMD_SEND_CUSTOM_FRAME       0x15

/*******************/
/* Set Flight LED Pattern */
#define QC_CMD_SET_FLIGHT_LED_PATTERN  0x30

/*******************/
/* Config Commands
 *
 * */

/* enter and leave config mode
 * exit config mode triggers reset
 * */
#define QC_CMD_CONFIG_MODE         	0xC0

/* read or update current configuration */
#define QC_CMD_READ_CONFIG          0xC1
#define QC_CMD_UPDATE_CONFIG      	0xC2



/* random config flags */
#define QC_CMD_QUADROCONFIG		        	0xCA

/* save or reload config
 * triggers reset
 */
#define QC_CMD_SAVE_CONFIG_TO_EEPROM        0xCE
#define QC_CMD_RELOAD_CONFIG_FROM_EEPROM    0xC9

/* restore hardcoded config values*/
#define QC_CMD_RESTORE_HARDCODED_CONFIG     0xCF

/*******************/
#define QC_CMD_SOFT_RESET               0xFF /* reset flight mode */
#define QC_CMD_HARD_RESET               0xFE /* reset cpu -> software restarts at main entry point */

/*******************/
/*
 * Custom Frame Identifiers
 */

#define DATA_ID_GYRO                0x01
#define DATA_ID_ACCEL               0x02
#define DATA_ID_MAGNETOMETER        0x03
#define DATA_ID_ANGLE               0x04
#define DATA_ID_ANGLE_SP            0x05
#define DATA_ID_HOR_ACCEL           0x1C
#define DATA_ID_ACCEL_SP            0x1D
#define DATA_ID_VELOCITY            0x06
#define DATA_ID_VELOCITY_SP         0x07
#define DATA_ID_HEIGHT              0x08
#define DATA_ID_RC                  0x09
#define DATA_ID_MOTOR               0x0A
#define DATA_ID_MOTOR_SP            0x0B
#define DATA_ID_CPU                 0x0C
#define DATA_ID_AKKU                0x0D
#define DATA_ID_TEMP                0x0E
#define DATA_ID_UPTIME              0x0F

/* gps */
#define DATA_ID_GPS_LLH             0x10
#define DATA_ID_GPS_ECEF            0x11
#define DATA_ID_GPS_TIME            0x12
#define DATA_ID_GPS_FIX             0x13
#define DATA_ID_GPS_DOP             0x14

/* configuration */
#define DATA_ID_COMP_FILTER         0x15
#define DATA_ID_PID_ANGLE_XY        0x16
#define DATA_ID_PID_ROT_Z           0x17
#define DATA_ID_PID_VEL             0x18
#define DATA_ID_PID_ACCEL           0x19
#define DATA_ID_QC_SETTINGS         0x1A

#define DATA_ID_GLOBAL_FLAGS        0x1B

/* 0x1c and 0x1d used (acceleration)*/
//next free data_id         0x1E
#define DATA_ID_EOF                 0x00
#define DATA_ID_BUFFER_OVERRUN      0xFF

/****************/
/* Quadroconfig */
#define QUADROCONFIG_ENABLE_LOW_VOLT    0x01
#define QUADROCONFIG_ENABLE_RC_LOST     0x02
#define QUADROCONFIG_ENABLE_FLIGHTLED   0x04
#define QUADROCONFIG_ENABLE_MOTORS      0x08

/* global flags */

#define FLIGHT_MODE_FLAG            0x00000001
#define CONFIG_MODE_FLAG            0x00000002
#define ERROR_FLAG                  0x00000004
#define USB_ERROR_FLAG              0x00000008

#define CPU_OVERLOAD_FLAG           0x00000010
#define NO_RC_SIGNAL_FLAG           0x00000020
#define LOW_VOLTAGE_FLAG            0x00000040
#define FREE_FLAG3                  0x00000080

#define MPU9150_OK_FLAG             0x00000100
#define RC_RECEIVER_OK_FLAG         0x00000200
#define BMP180_OK_FLAG              0x00000400
#define EEPROM_OK_FLAG              0x00000800

#define EMERGENCY_FLAG              0x80000000

class QCcoms: public Task {
public:
    QCcoms(Status* statusPtr, uint8_t defaultPrio, ConfigReader* _confReader,
            RxTxHandler* _rxtxHandler, FlightLED* _flightLEDs);
    virtual ~QCcoms();

    void initialize();
    void update();
    void reset();

private:

    RxTxHandler* rxtxHandler;
    ConfigReader* confReader;
    FlightLED* flightLEDs;

    /* loopback for communication test */
    void loopback();

    /* reply to custom frame request */
    void answerCustomFrame();

    /* confirm command
     * for commands without return
     */
    void sendConfirmation();

    /* config */
    void decodeConfigMSG();
    void updateConfig();

    /* fillBuffer returns next free pos in buffer*/
    uint16_t fillBuffer(uint8_t* buffer, uint16_t pos, float var);
    uint16_t fillBuffer(uint8_t* buffer, uint16_t pos, uint32_t var);
    uint16_t fillBuffer(uint8_t* buffer, uint16_t pos, int32_t var);
    uint16_t fillBuffer(uint8_t* buffer, uint16_t pos, int16_t var);
    uint16_t fillBuffer(uint8_t* buffer, uint16_t pos, uint16_t var);
    uint16_t fillBuffer(uint8_t* buffer, uint16_t pos, int8_t var);
    uint16_t fillBuffer(uint8_t* buffer, uint16_t pos, uint8_t var);

    /* used to create packages of RXTX buffsize*/
    uint16_t checkTXBufferOverrun(uint16_t currentPos, uint16_t dataToAdd);
    uint16_t bufferOverrun;
    uint16_t customFramePos;
};

#endif /* QCCOMS_H_ */
