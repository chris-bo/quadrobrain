/*
 * usbhandler.h
 *
 *  Created on: Mar 29, 2015
 *      Author: bohni
 */

#ifndef USBHANDLER_H_
#define USBHANDLER_H_


#define USB_RX_BUFF_SIZE		128
#define USB_TX_BUFF_SIZE		128


#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "Task.h"

class usb_handler: public Task {
public:
	usb_handler(Status* statusPtr, uint8_t defaultPrio, USBD_HandleTypeDef* husb);
	virtual ~usb_handler();

	void update();
	void initialize();

	uint8_t num_received_data;

private:
	USBD_HandleTypeDef* usb;


};

#endif /* USBHANDLER_H_ */
