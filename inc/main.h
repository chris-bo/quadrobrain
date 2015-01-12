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
#include "spi.h"
#include "dma.h"
#include "tim.h"
#include "gpio.h"


/* classes  ---------------------------------------------------------*/
#include "config.h"
#include "Scheduler.h"
#include "Task.h"
#include "LedBlink.h"
//#include "TestTask.h"
#include "RCreceiver.h"
#include "AccelerometerLSM303dlhc.h"
#include "GyroL3GD20.h"



/* Private variables ---------------------------------------------------------*/

extern Status status;
extern Scheduler scheduler;
extern RCreceiver rcReceiver;
extern Accelerometer_LSM303dlhc accelerometer;
extern Gyro_L3GD20 gyro;




#endif /* MAIN_H_ */