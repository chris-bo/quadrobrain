/*
 * main.h
 *
 *  Created on: Jan 6, 2015
 *      Author: bohni
 */

#ifndef MAIN_H_
#define MAIN_H_


/* autogenerated includes  -----------------------------------------*/
#include "stm32f3xx_hal.h"
#include "stm32f3xx_it.h"
#include "diag/Trace.h"
#include "i2c.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "adc.h"


/* classes  ---------------------------------------------------------*/
#include "config.h"
#include "ConfigReader.h"
#include "Scheduler.h"
#include "Task.h"
#include "LedBlink.h"
#include "RCreceiver.h"
#include "PPMGenerator.h"
#include "ComplementaryFilter.h"
#include "Compass.h"
#include "PIDController.h"
#include "MPU9150.h"
#include "usbhandler.h"
#include "AkkuMonitor.h"


/* Private variables ---------------------------------------------------------*/

extern Status status;
extern ConfigReader configReader;
extern Scheduler scheduler;
extern RCreceiver rcReceiver;
extern MPU9150 mpu9150;
extern PIDController pidControllerX;
extern PIDController pidControllerY;
extern usb_handler usb;
extern AkkuMonitor akku;

#endif /* MAIN_H_ */
