/*
 * config.h
 *
 *  Created on: Dec 13, 2014
 *      Author: bohni
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "stm32f3xx_hal.h"
#include "lsm303dlhc.h"
#include "l3gd20.h"

/*********************************************************************************/
/* Peripheral definitions */

/* Scheduler Timer */
#define SCHEDULER_TIMER							TIM2
#define SCHEDULER_INTERVALL_ms					10

/* RC receiver
 * Input capture
 */
#define RC_RECEIVER_TIMER						TIM4
#define RC_RECEIVER_INPUT_CHANNEL				TIM_CHANNEL_3
#define RC_RECEIVER_DEFAULT_PRIORITY			3

#define RC_RECEIVER_GPIO_PORT					GPIOB
#define RC_RECEIVER_GPIO_PIN					GPIO_Pin_8	// TIM4 CH3 (CH1 and CH2 used by I2C1)

/* signal detection defines */
#define RC_RECEIVER_OverrunTime_us				4000
#define RC_RECEIVER_MinHighTime_us				950
#define RC_RECEIVER_MaxHighTime_us				2050
//#define RC_RECEIVER_SyncTime_us				300		// included in RC_RECEIVER_MinHighTime_us

/* PPMGenerator
 * PWM
 */
// TODO: richtige Pinkonfiguration!
#define PPM_TIMER TIM4
#define PPM_PERIPH RCC_APB1Periph_TIM4
#define PPM_PORT_PERIPH RCC_AHBPeriph_GPIOD
#define PPM_PORT_CHANNEL1 GPIOD
#define PPM_PIN_CHANNEL1 GPIO_Pin_12
#define PPM_PINSOURCE_CHANNEL1 GPIO_PinSource12
#define PPM_PORT_CHANNEL2 GPIOD
#define PPM_PIN_CHANNEL2 GPIO_Pin_13
#define PPM_PINSOURCE_CHANNEL2 GPIO_PinSource13
#define PPM_PORT_CHANNEL3 GPIOD
#define PPM_PIN_CHANNEL3 GPIO_Pin_14
#define PPM_PINSOURCE_CHANNEL3 GPIO_PinSource14
#define PPM_PORT_CHANNEL4 GPIOD
#define PPM_PIN_CHANNEL4 GPIO_Pin_15
#define PPM_PINSOURCE_CHANNEL4 GPIO_PinSource15
//TODO: Dummy-Werte ersetzen
// Eine Periode ist ~ 20ms lang
#define PPM_TIMER_PERIOD 20000
// Minimaler Ausschaltwert bei 1ms
#define PPM_TIMER_MIN_PULSE_LENGTH 1000
// Maximaler Ausschaltwert bei 2ms
#define PPM_TIMER_MAX_PULSE_LENGTH 2000
#define PPM_TIMER_PRESCALER 1000000

/* Accelerometer MAgnetometer
 * LSM303DLHC
 */
#define ACCELEROMETER_I2C							I2C1
#define ACCELEROMETER_DEFAULT_PRIORITY				0

/* TODO interrupt lines */

#define ACCELEROMETER_RANGE						LSM303DLHC_FULLSCALE_16G
#define ACCELEROMETER_OUTPUT_RATE				LSM303DLHC_ODR_200_HZ
#define ACCELEROMETER_HR						LSM303DLHC_HR_DISABLE
/* TODO highpass filter config*/

/* TODO Magnetometer Class */
#define Magn_I2C
#define Magn_DEFAULT_PRIORITY

#define MAGNETOMETER_RANGE						LSM303DLHC_FS_4_0_GA
#define MAGNETOMETER_OUTPUT_RATE				LSM303DLHC_ODR_220_HZ


/* Gyro L3GD20
 *
 */

#define GYRO_SPI								SPI1
#define GYRO_CS_GPIO_PORT						GPIOE
#define GYRO_CS_PIN								GPIO_PIN_3

#define GYRO_DEFAULT_PRIORITY					0

#define GYRO_RANGE								L3GD20_FULLSCALE_500
#define GYRO_OUTPUT_RATE						L3GD20_OUTPUT_DATARATE_2
#define GYRO_BANDWIDTH							L3GD20_BANDWIDTH_3

/*********************************************************************************/
/*global macros */


/* flag macros */
#define GET_FLAG(var, flag)			var & flag
#define SET_FLAG(var, flag)			var |= flag
#define RESET_FLAG(var, flag)		var &= ~flag


/*********************************************************************************/
/*global flags */

#define EMERGENCY_FLAG				0x00000001

#endif /* CONFIG_H_ */
