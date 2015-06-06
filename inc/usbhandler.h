/*
 * usbhandler.h
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


#define BUFFER_TO_FLOAT(buffer,start) (float)((buffer[start]<<24)|(buffer[start + 1]<<16)|(buffer[start + 2]<<8)| buffer[start + 3] )

/**************************************************************************/
/* Defines
 * USB_CMD
 */

#define USB_CMD_LOOP				0x01
#define USB_CMD_SEND_STATUS_8BIT	0x02
#define USB_CMD_SEND_STATUS_FLOAT	0x03
#define USB_CMD_GLOBAL_FLAGS		0x04

/*******************/
/* Config Commands
 * only working if you are in config mode (except USB_CMD_CONFIG_MODE)
 * */

/* enter and leave config mode*/
#define USB_CMD_CONFIG_MODE         0xC0

#define USB_CMD_GET_CONFIG          0xC1
#define USB_CMD_UPDATE_CONFIG       0xC2

/* direct EEPROM access */
/* needs Address: MSB first
 * structure: USB_CMD; ADDR_MSB; ADDR_LSB; DATA;
 * */
#define USB_CMD_READ_BYTE           0xC3
#define USB_CMD_READ_2BYTES         0xC4
#define USB_CMD_READ_4BYTES         0xC5

#define USB_CMD_WRITE_BYTE          0xC6
#define USB_CMD_WRITE_2BYTES        0xC7
#define USB_CMD_WRITE_4BYTES        0xC8

/* restore hardcoded config values*/
#define USB_CMD_RESTORE_CONFIG      0xCF

/*******************/

#define USB_CMD_RESET               0xFF

#define USB_TIMEOUT 				0xFFFF

class usb_handler: public Task {
public:
    usb_handler(Status* statusPtr, uint8_t defaultPrio, USBD_HandleTypeDef* husb);
    virtual ~usb_handler();

    void update();
    void initialize(ConfigReader* _confReader);
    void sendMSGstring(uint8_t * buffer, uint8_t length);
    uint8_t usb_mode_request;

private:
    uint8_t usb_state;

    USBD_HandleTypeDef* usb;

    ConfigReader* confReader;
    void sendStatus8Bit();
    void sendStatusFloat(uint8_t part);
    void fillBuffer(uint8_t* buffer, uint8_t pos, float var);
    void readEEPROM(uint8_t byteCount);
    void writeEEPROM(uint8_t byteCount);
    void sendConfig();
    void updateConfig();
};

#endif /* USBHANDLER_H_ */
