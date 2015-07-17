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

#define USB_CMD_SEND_STATUS_8BIT	0x02

/*old*/
#define USB_CMD_SEND_STATUS_FLOAT	0x03

#define USB_CMD_GLOBAL_FLAGS		0x04

/*******************/
/* Status Data sending Commands
 * */

/* sends all sensor data:
 * accelerometer
 * gyro
 * magnetometer
 * barometer
 * rc receiver
 */
#define USB_CMD_SEND_SENSOR_DATA    0x10


/* angle
 * angle setpoint
 *
 * velocity
 * velocity setpoint
 *
 * motorSetpoint
 * motorValues
 */
#define USB_CMD_SEND_FLIGHT_DATA    0x11

/* system states
 *
 * uptime
 * akkuVoltage
 * cpuLoad
 *
 */
#define USB_CMD_SEND_SYSTEM_STATE   0x12


/* gps data */

#define USB_CMD_SEND_GPS_DATA_1     0x13
#define USB_CMD_SEND_GPS_DATA_2     0x14

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
/* save config
 * triggers reset
 */
#define USB_CMD_SAVE_CONFIG         0xCE
/* restore hardcoded config values*/
#define USB_CMD_RESTORE_CONFIG      0xCF

/*******************/

#define USB_CMD_RESET               0xFF

#define USB_TIMEOUT 				0xFFFF

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
    void sendStatusFloat(uint8_t part);
    void fillBuffer(uint8_t* buffer, uint8_t pos, float var);
    void readEEPROM(uint8_t byteCount);
    void writeEEPROM(uint8_t byteCount);
    void sendConfig();
    void updateConfig();
    void resetTransmissionState();

};

#endif /* USBHANDLER_H_ */
