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

	usb = husb;

}

usb_handler::~usb_handler() {

}

void usb_handler::update() {
	if (number_received_data > 0) {

		switch (UserRxBufferFS[0]) {
			case USB_CMD_LOOP:
				CDC_Transmit_FS(UserRxBufferFS, number_received_data);
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
				CDC_Transmit_FS(UserTxBufferFS, 4);
				break;
			default:
				/* TODO Send error?? */
				uint8_t error_msg[] = {"error wrong cmd"};
				CDC_Transmit_FS(error_msg, 16);
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

	CDC_Transmit_FS(UserTxBufferFS, 9);
}

void usb_handler::sendStatusFloat(uint8_t part) {

	if (part == 0) {
		fillBuffer(UserTxBufferFS, 0, status->accelX);
		fillBuffer(UserTxBufferFS, 4, status->accelY);
		fillBuffer(UserTxBufferFS, 8, status->accelZ);

		fillBuffer(UserTxBufferFS, 12, status->rateX);
		fillBuffer(UserTxBufferFS, 16, status->rateY);
		fillBuffer(UserTxBufferFS, 20, status->rateZ);

		fillBuffer(UserTxBufferFS, 24, status->magnetX);
		fillBuffer(UserTxBufferFS, 28, status->magnetY);
		fillBuffer(UserTxBufferFS, 32, status->magnetZ);

		fillBuffer(UserTxBufferFS, 36, status->angleX);
		fillBuffer(UserTxBufferFS, 40, status->angleY);
		fillBuffer(UserTxBufferFS, 44, status->angleNorth);

		// TODO rc values

		// TODO motor values

		// TODO sendStatusFloat
		CDC_Transmit_FS(UserTxBufferFS, 12);
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
