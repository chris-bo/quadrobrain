/*
 * main.h
 *
 *  Created on: Jan 6, 2015
 *      Author: bohni
 */

#ifndef MAIN_H_
#define MAIN_H_

/* autogenerated includes  -----------------------------------------*/
/* HAL Stuff */
#include "stm32f3xx_hal.h"
#include "stm32f3xx_it.h"
#include "i2c.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"
#include "adc.h"
#include "usart.h"
#include "spi.h"
#include "hal_defines.h"

/* classes  ---------------------------------------------------------*/
#include <core/config.h>
#include <core/Status.h>
#include <core/Task.h>
#include <core/Scheduler.h>
#include <core/PPMGenerator.h>

#include <sensor/MPU9150.h>
#include <sensor/BMP180.h>
#include <sensor/GPS.h>
#include <sensor/AkkuMonitor.h>
#include <sensor/RCreceiver.h>
#include <sensor/ComplementaryFilter.h>
#include <sensor/Compass.h>
#include <sensor/IMU.h>
#include <sensor/Altimeter.h>

#include <utility/ConfigReader.h>
#include <utility/Buzzer.h>
#include <utility/DiscoveryLEDs.h>
#include <utility/FlightLED.h>

#include <controller/PIDController.h>

#include <com/QCcoms.h>
#include <com/RxTxHandler.h>
#include <com/USBHandler.h>

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

#ifdef __cplusplus
extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_ */
