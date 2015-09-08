/*
 * USBHandler.h
 *
 *  Created on: Mar 29, 2015
 *      Author: bohni
 */

#ifndef USBHANDLER_H_
#define USBHANDLER_H_

#include "main.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "Task.h"
#include "config.h"

/**************************************************************************/
/*
 * USB_CMD
 */

/* resend received */
#define USB_CMD_LOOP				0x01

#define USB_CMD_GLOBAL_FLAGS		0x04

/*******************/
/* custom frame
 *
 * PC sends custom frame command and identifiers for requested data
 * terminated by one byte of DATA_ID_EOF
 *
 * USBHandler responds with requested data
 * if inserting data will cause buffer overrun. the remaining buffer
 * will be 0xFF
 *
 * Unknown data id will be ignored
 *
 */

#define USB_CMD_SEND_CUSTOM_FRAME       0x15

/*******************/
/* Set Flight LED Pattern */
#define USB_CMD_SET_FLIGHT_LED_PATTERN  0x30

/*******************/
/* Config Commands
 *  (except USB_CMD_CONFIG_MODE)
 * */

/* enter and leave config mode*/
#define USB_CMD_CONFIG_MODE         0xC0

/* get or update whole configuration*/
#define USB_CMD_GET_CONFIG          0xC1
#define USB_CMD_UPDATE_CONFIG       0xC2

/* direct EEPROM access
 * only working if you are in config mode
 *
 * needs Address: MSB first
 * structure: USB_CMD; ADDR_MSB; ADDR_LSB; DATA;
 * */
#define USB_CMD_READ_BYTE           0xC3
#define USB_CMD_READ_2BYTES         0xC4
#define USB_CMD_READ_4BYTES         0xC5

#define USB_CMD_WRITE_BYTE          0xC6
#define USB_CMD_WRITE_2BYTES        0xC7
#define USB_CMD_WRITE_4BYTES        0xC8

#define USB_CMD_RELOAD_EEPROM       0xC9

#define USB_CMD_QUADROCONFIG        0xCA
/* save config
 * triggers reset
 */
#define USB_CMD_SAVE_CONFIG         0xCE
/* restore hardcoded config values*/
#define USB_CMD_RESTORE_CONFIG      0xCF

/*******************/

#define USB_CMD_RESET               0xFF

#define USB_TIMEOUT 				0xFFFF

/*******************/
/*
 * Custom Frame Identifiers
 */

#define DATA_ID_GYRO                0x01
#define DATA_ID_ACCEL               0x02
#define DATA_ID_MAGNETOMETER        0x03
#define DATA_ID_ANGLE               0x04
#define DATA_ID_ANGLE_SP            0x05
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
#define DATA_ID_QC_SETTING          0x1A

#define DATA_ID_GLOBAL_FLAGS        0x1B

#define DATA_ID_EOF                 0x00
#define DATA_ID_BUFFER_OVERRUN      0xFF

/****************/
/* Quadroconfig */
#define QUADROCONFIG_ENABLE_LOW_VOLT    0x01
#define QUADROCONFIG_ENABLE_RC_LOST     0x02
#define QUADROCONFIG_ENABLE_FLIGHTLED   0x04
#define QUADROCONFIG_ENABLE_MOTORS      0x08

/**************************************************************************/
/* USB_MODES */

#define USB_MODE_NORMAL             0x00
#define USB_MODE_CONFIG             0x01
#define USB_MODE_LEAVE_CONFIG       0x02
#define USB_MODE_SAVE_CONFIG        0x03
#define USB_MODE_RELOAD_EEPROM      0x04

#define USB_MODE_RESET              0xFF

/**************************************************************************/

class USBHandler: public Task {
public:
    USBHandler(Status* statusPtr, uint8_t defaultPrio, USBD_HandleTypeDef* husb);
    virtual ~USBHandler();

    void update();
    void initialize(ConfigReader* _confReader);
    void kill();
    uint8_t usb_mode_request;

private:
    uint8_t usb_state;
    uint8_t usbTransmitBusyCounter;
    USBD_HandleTypeDef* usb;

    ConfigReader* confReader;
    void usbTransmit(uint8_t* buffer, uint16_t len);
    void sendCustomFrame();
    void createCustomFrame(uint8_t frameID);

    void decodeConfigMSG();

    /* fillBuffer returns next free pos in buffer*/
    uint8_t fillBuffer(uint8_t* buffer, uint8_t pos, float var);
    uint8_t fillBuffer(uint8_t* buffer, uint8_t pos, uint32_t var);
    uint8_t fillBuffer(uint8_t* buffer, uint8_t pos, int32_t var);
    uint8_t fillBuffer(uint8_t* buffer, uint8_t pos, int16_t var);
    uint8_t fillBuffer(uint8_t* buffer, uint8_t pos, uint16_t var);
    uint8_t fillBuffer(uint8_t* buffer, uint8_t pos, int8_t var);
    uint8_t fillBuffer(uint8_t* buffer, uint8_t pos, uint8_t var);
    void readEEPROM(uint8_t byteCount);
    void writeEEPROM(uint8_t byteCount);
    void sendConfig();
    void updateConfig();
    void resetTransmissionState();
    uint8_t checkTXBufferOverrun(uint8_t currentPos, uint8_t dataToAdd, uint8_t* overrun);

};

#endif /* USBHANDLER_H_ */
