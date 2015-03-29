/*
 * usbhandler.h
 *
 *  Created on: Mar 29, 2015
 *      Author: bohni
 */

#ifndef USBHANDLER_H_
#define USBHANDLER_H_

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "Task.h"
#include "config.h"

/**************************************************************************/
/* Defines
 * USB_CMD
 */

#define USB_CMD_LOOP				0x01
#define USB_CMD_SEND_STATUS_8BIT	0x02
#define USB_CMD_SEND_STATUS_FLOAT	0x03
#define USB_CMD_GLOBAL_FLAGS		0x04

class usb_handler: public Task {
public:
	usb_handler(Status* statusPtr, uint8_t defaultPrio,
	        USBD_HandleTypeDef* husb);
	virtual ~usb_handler();

	void update();
	void initialize();

private:
	USBD_HandleTypeDef* usb;

	void sendStatus8Bit();
	void sendStatusFloat(uint8_t part);
	void fillBuffer(uint8_t* buffer, uint8_t pos, float var);
};

#endif /* USBHANDLER_H_ */
