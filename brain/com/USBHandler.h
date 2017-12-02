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
///* USB_MODES */
//
//#define USB_MODE_NORMAL             0x00
//#define USB_MODE_CONFIG             0x01
//#define USB_MODE_LEAVE_CONFIG       0x02
//#define USB_MODE_SAVE_CONFIG        0x03
//#define USB_MODE_RELOAD_EEPROM      0x04
//
//#define USB_MODE_RESET              0xFF
/**************************************************************************/

class USBHandler: public RxTxHandler {
public:
    USBHandler(Status* statusPtr, uint8_t defaultPrio, DiscoveryLEDs* _ledctrl,
            Led_TypeDef _led, USBD_HandleTypeDef* husb);
    virtual ~USBHandler();

//      void update();
//    void initialize();
    void kill();
    void startRX();

    void sendTXBuffer(uint16_t byte_count);

private:
    USBD_HandleTypeDef* usb_handle;

    USBD_StatusTypeDef usb_state;
    uint8_t usbTransmitBusyCounter;

    void resetTransmissionState();

};

#endif /* USBHANDLER_H_ */
