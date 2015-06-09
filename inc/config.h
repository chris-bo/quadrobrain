/*
 * config.h
 *
 *  Created on: Dec 13, 2014
 *      Author: bohni
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include "stm32f3xx_hal.h"
#include "stm32f3_discovery.h"

/*********************************************************************************/
/* Peripheral definitions */

/****************************/
/* Scheduler */
#define SCHEDULER_TIMER							TIM2
#define SCHEDULER_INTERVALL_ms					10
#define CPU_LOAD_HISTORY                        4.0f

/****************************/
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

/****************************/
/* PPMGenerator
 * PWM
 */
#define PPMGENERATOR_DEFAULT_PRIORITY			3
// Eine Periode ist ~ 20ms lang
#define PPM_TIMER_PERIOD 65535
// Minimaler Ausschaltwert bei 1ms
#define PPM_TIMER_MIN_PULSE_LENGTH 3273
// Maximaler Ausschaltwert bei 2ms
#define PPM_TIMER_MAX_PULSE_LENGTH 6382
#define PPM_TIMER_PRESCALER 22
// Einstellzeit f�r Regler
#define PPM_SETUP_TIME 3000000

/****************************/
/* Sensorboard MPU9150
 *
 */
#define MPU9150_I2C								I2C1
#define MPU9150_DEFAULT_PRIORITY				0
#define MPU9150_INTERRUPT_PIN					GPIO_PIN_0 /*connect to PE0*/

/* define G for scaling to m/s^2 */
#define G										9.81f

/****************************/
/*
 * PIDController
 */
#define PID_DEFAULT_PRIORITY					0

/****************************/
/* USB
 *
 */
#define USB_DEFAULT_PRIORITY					2
#define USB_TRANSMIT_BUSY_MAX                   2

/****************************/
/* ADC to Check akku voltage
 *
 * */
#define AKKUMONITOR_DEFAULT_PRIORITY			10
#define LOW_VOLTAGE_WARNING_THRESHOLD           10.0f // [V]

/****************************/
/* Pressure Sensor */
#define BMP180_DEFAULT_PRIORITY                 10
#define BMP180_I2C                              I2C2

/****************************/
/* LEDs
 *
 */
#define LEDs_DEFAULT_PRIORITY                   20

#define USB_RECEIVE_LED                         LED7
#define USB_TRANSMIT_LED                        LED5
#define CONFIG_LED                              LED8
#define POWER_LED                               LED4
#define FLIGHT_LED                              LED6
#define OVERLOAD_LED                            LED3
#define ERROR_LED                               LED10

/*********************************************************************************/
/* hard coded settings*/
#define PID_XY_P                            0.21f
#define PID_XY_I                            0.22f
#define PID_XY_D                            0.23f

#define PID_Z_P                             0.31f
#define PID_Z_I                             0.32f
#define PID_Z_D                             0.33f

#define FILTER_COEFFICIENT                  0.98f
/*********************************************************************************/
/*global macros */


/* flag macros */
#define GET_FLAG(var, flag)			(var & flag)
#define SET_FLAG(var, flag)			(var |= flag)
#define RESET_FLAG(var, flag)		(var &= ~flag)


/*********************************************************************************/
/*global flags */

#define EMERGENCY_FLAG				0x00000004

#define RESET_TO_CONFIG             0x01
#define RESET_TO_FLIGHT             0x00

#endif /* CONFIG_H_ */
