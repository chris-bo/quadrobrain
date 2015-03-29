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

/* CMD: 0x0X -> data requests
 * CMD: 0xX0 -> sending data
 * */

#define USB_CMD_LOOP				0x01
#define USB_CMD_SEND_STATUS			0x02

class usb_handler: public Task {
public:
	usb_handler(Status* statusPtr, uint8_t defaultPrio, USBD_HandleTypeDef* husb);
	virtual ~usb_handler();

	void update();
	void initialize();


private:
	USBD_HandleTypeDef* usb;

	void sendRequestedData( uint8_t cmd);
};

#endif /* USBHANDLER_H_ */
