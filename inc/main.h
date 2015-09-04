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
#include "usart.h"
#include "spi.h"

/* classes  ---------------------------------------------------------*/
#include "config.h"
#include "ConfigReader.h"
#include "Scheduler.h"
#include "Task.h"
#include "RCreceiver.h"
#include "PPMGenerator.h"
#include "ComplementaryFilter.h"
#include "Compass.h"
#include "PIDController.h"
#include "MPU9150.h"
#include "USBHandler.h"
#include "AkkuMonitor.h"
#include "BMP180.h"
#include "DiscoveryLEDs.h"
#include "MAfilterF.h"
#include "Buzzer.h"
#include "MotionController.h"
#include "GPS.h"
#include "FlightLED.h"

/* Private variables ---------------------------------------------------------*/

extern Status status;
extern ConfigReader configReader;
extern Scheduler scheduler;
extern RCreceiver rcReceiver;
extern MPU9150 mpu9150;
extern AkkuMonitor akku;
extern BMP180 baro;
extern DiscoveryLEDs leds;
extern Buzzer beep1;
extern Buzzer beep2;
extern GPS gpsReceiver;
extern FlightLED flightLEDs;
#endif /* MAIN_H_ */
