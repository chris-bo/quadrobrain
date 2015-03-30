/*
 * config.h
 *
 *  Created on: Dec 13, 2014
 *      Author: bohni
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "stm32f3xx_hal.h"


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
#define PPMGENERATOR_DEFAULT_PRIORITY			3
// Eine Periode ist ~ 20ms lang
#define PPM_TIMER_PERIOD 65535
// Minimaler Ausschaltwert bei 1ms
#define PPM_TIMER_MIN_PULSE_LENGTH 3273
// Maximaler Ausschaltwert bei 2ms
#define PPM_TIMER_MAX_PULSE_LENGTH 6546
#define PPM_TIMER_PRESCALER 22
// Einstellzeit für Regler
#define PPM_SETUP_TIME 71925500

/* Sensorboard MPU9150
 *
 */
#define MPU9150_I2C								I2C1
#define MPU9150_DEFAULT_PRIORITY				0
#define MPU9150_INTERRUPT_PIN					GPIO_PIN_0 /*connect to PE0*/

/* define G for scaling to m/s^2 */
#define G										9.81f

/* USB
 *
 */
#define USB_DEFAULT_PRIORITY					2

/*********************************************************************************/
/*global macros */


/* flag macros */
#define GET_FLAG(var, flag)			(var & flag)
#define SET_FLAG(var, flag)			(var |= flag)
#define RESET_FLAG(var, flag)		(var &= ~flag)


/*********************************************************************************/
/*global flags */

#define EMERGENCY_FLAG				0x00000001

#endif /* CONFIG_H_ */
