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
				usb_state = CDC_Transmit_FS(UserRxBufferFS,
				        number_received_data);
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
				uint8_t error_msg[] = { "ERROR:unknown cmd!" };
				usb_state = CDC_Transmit_FS(error_msg, sizeof(error_msg));
				USBD_CDC_ReceivePacket(usb);
				break;
		}
		number_received_data = 0;
	}

	this->resetPriority();

}

void usb_handler::initialize() {

	SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}

void usb_handler::sendStatus8Bit() {
	/*
	 * old Debug_Data
	 *
	 0. Byte: Kanal1	= roll
	 1. Byte: Kanal2	= throttle
	 2. Byte: Kanal3	= nick
	 3. Byte: Kanal4	= yaw
	 4. Byte: Kanal5	= enable
	 5. Byte: Kanal6	= lin control
	 6. Byte: Kanal7	= left switch
	 7. Byte: Kanal8	= --
	 8. Byte: Motor1
	 9. Byte: Motor2
	 10. Byte: Motor3
	 11. Byte: Motor4
	 12. Byte: AccX
	 13. Byte: AccY
	 14. Byte: AccZ
	 15. Byte: GyroX
	 16. Byte: GyroY
	 17. Byte: GyroZ
	 18. Byte: Druck
	 19. Byte: Temp
	 20. Byte: H�he
	 21. Byte: Winkel x
	 22. Byte: Winkel y
	 23. Byte: Winkel z
	 */

	/* RC signals*/

	UserTxBufferFS[0] = (uint8_t) (status->rcSignalRoll * 100 + 50);
	UserTxBufferFS[1] = (uint8_t) (status->rcSignalThrottle * 100);
	UserTxBufferFS[2] = (uint8_t) (status->rcSignalNick * 100 + 50);
	UserTxBufferFS[3] = (uint8_t) (status->rcSignalYaw * 100 + 50);
	UserTxBufferFS[4] = (uint8_t) (status->rcSignalEnable * 100);
	UserTxBufferFS[5] = (uint8_t) (status->rcSignalLinPoti * 100);
	UserTxBufferFS[6] = (uint8_t) (status->rcSignalSwitch * 100);

	/* Motor controls*/
	UserTxBufferFS[8] = (uint8_t) (status->motorValues[0] * 100);
	UserTxBufferFS[9] = (uint8_t) (status->motorValues[1] * 100);
	UserTxBufferFS[10] = (uint8_t) (status->motorValues[2] * 100);
	UserTxBufferFS[11] = (uint8_t) (status->motorValues[3] * 100);

	/* Accel XYZ in 0,01 G */
	UserTxBufferFS[12] = (int8_t) (status->accelX / G * 100);
	UserTxBufferFS[13] = (int8_t) (status->accelY / G * 100);
	UserTxBufferFS[14] = (int8_t) (status->accelZ / G * 100);

	/* Gyro XYZ in 10 deg/s */
	UserTxBufferFS[15] = (int8_t) (status->rateX / 10);
	UserTxBufferFS[16] = (int8_t) (status->rateX / 10);
	UserTxBufferFS[17] = (int8_t) (status->rateX / 10);

	/* Temp */
	UserTxBufferFS[19] = (int8_t) (status->temp);
	/* akku voltage */
	UserTxBufferFS[20] = (uint8_t) (status->akkuVoltage);

	/*angles in deg*/
	UserTxBufferFS[21] = (int8_t) (status->angleX);
	UserTxBufferFS[22] = (int8_t) (status->angleY);
	/* north in 10 deg*/
	UserTxBufferFS[23] = (int8_t) (status->angleNorth / 10);

	// TODO usb: send 8bit Magn XYZ */

	usb_state = CDC_Transmit_FS(UserTxBufferFS, 24);
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
		fillBuffer(UserTxBufferFS, 48, status->rcSignalRoll);
		fillBuffer(UserTxBufferFS, 52, status->rcSignalNick);
		fillBuffer(UserTxBufferFS, 56, status->rcSignalYaw);
		fillBuffer(UserTxBufferFS, 60, status->rcSignalThrottle);
		fillBuffer(UserTxBufferFS, 64, status->rcSignalEnable);

		/* motor control values */
		fillBuffer(UserTxBufferFS, 68, status->motorValues[0]);
		fillBuffer(UserTxBufferFS, 72, status->motorValues[1]);
		fillBuffer(UserTxBufferFS, 76, status->motorValues[2]);
		fillBuffer(UserTxBufferFS, 80, status->motorValues[3]);

		usb_state = CDC_Transmit_FS(UserTxBufferFS, 84);
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
	while ((usb_state = CDC_Transmit_FS(buffer, length)) != USBD_OK) {
		if ((timeout--) == 0) {
			return;
		}
	}
}
