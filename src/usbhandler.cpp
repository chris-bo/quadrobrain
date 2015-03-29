/*
 * usbhandler.cpp
 *
 *  Created on: Mar 29, 2015
 *      Author: bohni
 */

#include "usbhandler.h"

usb_handler::usb_handler(Status* statusPtr, uint8_t defaultPrio, USBD_HandleTypeDef* husb) : Task(statusPtr, defaultPrio) {


	num_received_data = 0;
	usb = husb;

}

usb_handler::~usb_handler() {
	// TODO Auto-generated destructor stub
}

void usb_handler::update() {
}

void usb_handler::initialize() {
}
