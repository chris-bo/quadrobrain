/*
 * USBHandler.h
 *
 *  Created on: Mar 29, 2015
 *      Author: bohni
 */

#ifndef USBHANDLER_H_
#define USBHANDLER_H_

#include "usb_device.h"
#include "usbd_cdc_if.h"
#include <com/RxTxHandler.h>

/**************************************************************************/

#define USB_TIMEOUT 				0xFFFF


/**************************************************************************/
/* USB_MODES */

#define USB_MODE_NORMAL             0x00
#define USB_MODE_CONFIG             0x01
#define USB_MODE_LEAVE_CONFIG       0x02
#define USB_MODE_SAVE_CONFIG        0x03
#define USB_MODE_RELOAD_EEPROM      0x04

#define USB_MODE_RESET              0xFF

/**************************************************************************/

class USBHandler: public RxTxHandler {
public:
    USBHandler(Status* statusPtr, uint8_t defaultPrio, USBD_HandleTypeDef* husb);
    virtual ~USBHandler();


//    void update();
//    void initialize();
//    void kill();
//    uint8_t usb_mode_request;

private:
    USBD_HandleTypeDef* usb;

    uint8_t usb_state;
    uint8_t usbTransmitBusyCounter;

    void usbTransmit(uint8_t* buffer, uint16_t len);

    void resetTransmissionState();

};

#endif /* USBHANDLER_H_ */
