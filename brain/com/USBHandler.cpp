/*
 * usbhandler.cpp
 *
 *  Created on: Mar 29, 2015
 *      Author: bohni
 */

#include "USBHandler.h"

USBHandler::USBHandler(Status* statusPtr, uint8_t defaultPrio,
		DiscoveryLEDs* _ledctrl, Led_TypeDef _led, USBD_HandleTypeDef* husb) :
		RxTxHandler(statusPtr, defaultPrio, _ledctrl, _led) {

	usb_state = USBD_BUSY;
	usb_handle = husb;
	usbTransmitBusyCounter = 0;
}

USBHandler::~USBHandler() {
}

void USBHandler::sendTXBuffer(uint16_t byte_count) {
	setLED(true);
	usb_state = (USBD_StatusTypeDef) CDC_Transmit_FS(TxBuffer, byte_count);

	if (usb_state == USBD_BUSY) {
		usbTransmitBusyCounter++;
		if (usbTransmitBusyCounter == USB_TRANSMIT_BUSY_MAX) {
			resetTransmissionState();
			SET_FLAG(status->globalFlags, USB_ERROR_FLAG | ERROR_FLAG);
		}
	} else {
		usbTransmitBusyCounter = 0;
		RESET_FLAG(status->globalFlags, USB_ERROR_FLAG);
	}
}

void USBHandler::resetTransmissionState() {
	/* clear endpoint*/
	usb_handle->pClass->DataIn(usb_handle, 1);
	PCD_HandleTypeDef *hpcd = (PCD_HandleTypeDef *) usb_handle->pData;
	PCD_EPTypeDef *ep = (PCD_EPTypeDef*) &hpcd->IN_ep[1];
	ep->xfer_count = 0;
	usbTransmitBusyCounter = 0;
	usb_state = USBD_OK;
}

void USBHandler::initialize() {

	initializeBuffers(UserRxBufferFS, UserTxBufferFS, &number_received_data);

	SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}

void USBHandler::kill() {
	/* override default kill function
	 * usb must not be killed
	 */
	memset(TxBuffer, 0, RXTX_BUFF_SIZE);
	memset(RxBuffer, 0, RXTX_BUFF_SIZE);
}

void USBHandler::startRX() {
//	if (usb_state == USBD_OK) {
//			numberReceivedData = 0;
//			newDataReceived = false;
//	}

	usb_state = (USBD_StatusTypeDef) USBD_CDC_ReceivePacket(usb_handle);
}
