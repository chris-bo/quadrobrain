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
#define SCHEDULER_INTERVALL_ms					1
/* compute CPU Load with last CPU_LOAD_HISTORY values*/
#define CPU_LOAD_HISTORY                        5
#define SCHEDULER_TIMER_PERIOD                  (uint32_t) (SCHEDULER_INTERVALL_ms * 1000)
#define SCHEDULER_TIMER_PRESCALER               (uint16_t) (HAL_RCC_GetSysClockFreq() / 1000000 - 1)

/****************************/
/* RC receiver
 * Input capture
 */
#define RC_RECEIVER_TIMER						TIM4
#define RC_RECEIVER_INPUT_CHANNEL				TIM_CHANNEL_3
#define RC_RECEIVER_DEFAULT_PRIORITY			3
#define RC_RECEIVER_TIMER_PRESCALER             (uint16_t) (HAL_RCC_GetSysClockFreq() / 1000000 - 1)

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
#define FLIGHT_DATA_RECEPTION_LED               LED9
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
#define GET_FLAGS(var, flags)       ((var & flags) == flags)
#define SET_FLAG(var, flag)			(var |= flag)
#define RESET_FLAG(var, flag)		(var &= ~flag)

/*********************************************************************************/
/*global flags */

#define FLIGHT_MODE_FLAG            0x00000001
#define CONFIG_MODE_FLAG            0x00000002
#define ERROR_FLAG                  0x00000004
#define USB_ERROR_FLAG              0x00000008

#define CPU_OVERLOAD_FLAG           0x00000010
//#define NO_RC_SIGNAL_FLAG           0x00000020
#define LOW_VOLTAGE_FLAG            0x00000040
#define FREE_FLAG3                  0x00000080

#define MPU9150_OK_FLAG             0x00000100
#define RC_RECEIVER_OK_FLAG         0x00000200
#define BMP180_OK_FLAG              0x00000400
#define EEPROM_OK_FLAG              0x00000800

#define EMERGENCY_FLAG              0x80000000

#define RESET_TO_CONFIG             0x01
#define RESET_TO_FLIGHT             0x00

#endif /* CONFIG_H_ */
