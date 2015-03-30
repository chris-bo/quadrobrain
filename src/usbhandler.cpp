/*
 * usbhandler.cpp
 *
 *  Created on: Mar 29, 2015
 *      Author: bohni
 */

#include "usbhandler.h"

usb_handler::usb_handler(Status* statusPtr, uint8_t defaultPrio,
        USBD_HandleTypeDef* husb)
		: Task(statusPtr, defaultPrio) {

	usb_state = USBD_BUSY;
	usb = husb;

}

usb_handler::~usb_handler() {

}

void usb_handler::update() {
	if (number_received_data > 0) {

		switch (UserRxBufferFS[0]) {
			case USB_CMD_LOOP:
				usb_state = CDC_Transmit_FS(UserRxBufferFS, number_received_data);
				USBD_CDC_ReceivePacket(usb);
				break;
			case USB_CMD_SEND_STATUS_8BIT:
				sendStatus8Bit();
				USBD_CDC_ReceivePacket(usb);
				break;
			case USB_CMD_SEND_STATUS_FLOAT:
				sendStatusFloat(0);
				break;
			case USB_CMD_GLOBAL_FLAGS:
				/* HIGH -> LOW */
				UserTxBufferFS[0] = (uint8_t) ((status->globalFlags >> 24)
				        & 0xFF);
				UserTxBufferFS[1] = (uint8_t) ((status->globalFlags >> 16)
				        & 0xFF);
				UserTxBufferFS[2] =
				        (uint8_t) ((status->globalFlags >> 8) & 0xFF);
				UserTxBufferFS[3] = (uint8_t) ((status->globalFlags) & 0xFF);
				usb_state = CDC_Transmit_FS(UserTxBufferFS, 4);
				USBD_CDC_ReceivePacket(usb);
				break;
			default:
				/* TODO Send error?? */
				uint8_t error_msg[] = {"ERROR:unknown cmd!"};
				usb_state = CDC_Transmit_FS(error_msg, sizeof(error_msg));
				USBD_CDC_ReceivePacket(usb);
				break;
		}
		number_received_data = 0;
	}

	this->resetPriority();

}

void usb_handler::initialize() {
	MX_USB_DEVICE_Init();
	SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}

void usb_handler::sendStatus8Bit() {

	/* needs to be updated, if new variables are needed to be sent */

	/* Accel XYZ in 0,01 G */
	UserTxBufferFS[0] = (int8_t) (status->accelX / G * 100);
	UserTxBufferFS[1] = (int8_t) (status->accelY / G * 100);
	UserTxBufferFS[2] = (int8_t) (status->accelZ / G * 100);

	/* Gyro XYZ in 10 deg/s */
	UserTxBufferFS[3] = (int8_t) status->rateX / 10;
	UserTxBufferFS[4] = (int8_t) status->rateX / 10;
	UserTxBufferFS[5] = (int8_t) status->rateX / 10;

	/* Magn XYZ */
	// TODO /* Magn XYZ */
	UserTxBufferFS[6] = 0;
	UserTxBufferFS[7] = 0;
	UserTxBufferFS[8] = 0;

	//TODO sendStatus8Bit

	usb_state = CDC_Transmit_FS(UserTxBufferFS, 9);
}

void usb_handler::sendStatusFloat(uint8_t part) {
/* needs to be updated, if new variables are needed to be sent */

	if (part == 0) {
		/* accelerometer XYZ in m/s^2 */
		fillBuffer(UserTxBufferFS, 0, status->accelX);
		fillBuffer(UserTxBufferFS, 4, status->accelY);
		fillBuffer(UserTxBufferFS, 8, status->accelZ);

		/* rate in deg/s */
		fillBuffer(UserTxBufferFS, 12, status->rateX);
		fillBuffer(UserTxBufferFS, 16, status->rateY);
		fillBuffer(UserTxBufferFS, 20, status->rateZ);

		/* magn in gauss ??? */
		fillBuffer(UserTxBufferFS, 24, status->magnetX);
		fillBuffer(UserTxBufferFS, 28, status->magnetY);
		fillBuffer(UserTxBufferFS, 32, status->magnetZ);

		/* angles in deg */
		fillBuffer(UserTxBufferFS, 36, status->angleX);
		fillBuffer(UserTxBufferFS, 40, status->angleY);
		fillBuffer(UserTxBufferFS, 44, status->angleNorth);

		/* rc signals */
		fillBuffer(UserTxBufferFS, 48, status->rcSignalX);
		fillBuffer(UserTxBufferFS, 52, status->rcSignalY);
		fillBuffer(UserTxBufferFS, 56, status->rcSignalZ);
		fillBuffer(UserTxBufferFS, 60, status->rcSignalThrottle);
		fillBuffer(UserTxBufferFS, 64, status->rcSignalEnable);

		/* motor control values */
		UserTxBufferFS[68] = status->motorValues[0];
		UserTxBufferFS[69] = status->motorValues[1];
		UserTxBufferFS[70] = status->motorValues[2];
		UserTxBufferFS[71] = status->motorValues[3];

		usb_state = CDC_Transmit_FS(UserTxBufferFS, 72);
		USBD_CDC_ReceivePacket(usb);
	}
}

void usb_handler::fillBuffer(uint8_t* buffer, uint8_t pos, float var) {

	uint8_t* tmp = (uint8_t*) &var;
	buffer[pos] = *tmp++;
	buffer[pos + 1] = *(tmp++);
	buffer[pos + 2] = *(tmp++);
	buffer[pos + 3] = *(tmp);

}

void usb_handler::sendMSGstring(uint8_t* buffer, uint8_t length) {

	uint32_t timeout = USB_TIMEOUT;
	while ((usb_state = CDC_Transmit_FS(buffer, length)) != USBD_OK){
		if ((timeout--) == 0){
			return;
		}
	}
}
