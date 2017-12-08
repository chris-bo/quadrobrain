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

    initializeBuffers(UserRxBufferFS, UserTxBufferFS, &number_received_data,
    USB_RXTX_BUFF_SIZE);
    receptionComplete = true;

    /* dummy padding variable */
    pad = 0;
}

USBHandler::~USBHandler() {
}

void USBHandler::sendTXBuffer(uint16_t byte_count) {
    setLED(true);
    usb_state = (USBD_StatusTypeDef) CDC_Transmit_FS(TxBuffer, byte_count);
    if (usb_state != USBD_OK) {
        usbTransmitBusyCounter++;
        if (usbTransmitBusyCounter == USB_TRANSMIT_BUSY_MAX) {
            resetTransmissionState();
            status->globalFlags.usbError = true;
            status->globalFlags.error = true;
        }
    } else {
        usbTransmitBusyCounter = 0;
        status->globalFlags.usbError = false;
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
void USBHandler::kill() {
    /* override default kill function
     * usb must not be killed
     */
    reset();
}

void USBHandler::startRX() {

    /* listen for new messages */
    *numberReceivedData = 0;
    (USBD_StatusTypeDef) USBD_CDC_ReceivePacket(usb_handle);
}
