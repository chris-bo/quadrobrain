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


/** TODO USB:
 *
 * 	methods to update pid configuration
 * 	methods to update filter configuration
 *
 */



/**************************************************************************/
/* Defines
 * USB_CMD
 */

#define USB_CMD_LOOP				0x01
#define USB_CMD_SEND_STATUS_8BIT	0x02
#define USB_CMD_SEND_STATUS_FLOAT	0x03
#define USB_CMD_GLOBAL_FLAGS		0x04



#define USB_TIMEOUT 				0xFFFF


class usb_handler: public Task {
public:
	usb_handler(Status* statusPtr, uint8_t defaultPrio,
	        USBD_HandleTypeDef* husb);
	virtual ~usb_handler();

	void update();
	void initialize();
	void sendMSGstring(uint8_t * buffer, uint8_t length);

private:
	USBD_HandleTypeDef* usb;

	uint8_t usb_state;
	void sendStatus8Bit();
	void sendStatusFloat(uint8_t part);
	void fillBuffer(uint8_t* buffer, uint8_t pos, float var);
};

#endif /* USBHANDLER_H_ */
