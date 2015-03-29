/*
 * usbhandler.cpp
 *
 *  Created on: Mar 29, 2015
 *      Author: bohni
 */

#include "usbhandler.h"

usb_handler::usb_handler(Status* statusPtr, uint8_t defaultPrio, USBD_HandleTypeDef* husb) : Task(statusPtr, defaultPrio) {


	usb = husb;

}

usb_handler::~usb_handler() {

}

void usb_handler::update() {
	if (number_received_data > 0) {

		if ((UserRxBufferFS[0] & 0x0F) > 0) {
			sendRequestedData(UserRxBufferFS[0]);
		}
		number_received_data = 0;
		USBD_CDC_ReceivePacket(usb);
	}

	this->resetPriority();

}

void usb_handler::initialize() {
	MX_USB_DEVICE_Init();
	SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}

void usb_handler::sendRequestedData(uint8_t cmd) {

	switch (cmd) {
		case USB_CMD_LOOP:
			CDC_Transmit_FS(UserRxBufferFS, number_received_data);
			break;
		case USB_CMD_SEND_STATUS:

			break;
		default:
			break;
	}
}
