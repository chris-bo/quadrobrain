/*
 * config.h
 *
 *  Created on: Dec 13, 2014
 *      Author: bohni
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <stm32f3xx_hal.h>
#include <stm32f3_discovery.h>
#include <math.h>

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
#define RC_RECEIVER_MaxHighTime_us				2080
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

#define THROTTLE_SCALING                        0.7f
#define PPM_GENERATOR_THROTTLE_THRESHOLD        0.1f
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
#define PID_THROTTLE_THRESHOLD                  0.1f
#define PID_LIMIT                               0.30f

/* set to zero to disable limiting e_sum */
#define PID_SUM_LIMIT                           INFINITY

/* uncomment to round processvariable to 3 digits behind . */
//#define PID_ROUND_PROCESS_VARIABLE
/* HorizontalMotion Control */
#define HorizontalMotionControl_PRIORITY        0

#define ACCELERATION_MA_FILTER_SIZE             10

/* velocity pid limit = acceleration setpoint limit */
#define VELOCITY_PID_LIMIT                      G
#define VELOCITY_PID_SUM_LIMIT                  INFINITY

/* acceleration pid limit ^= angle setpoint limit */
#define ACCELERATION_PID_LIMIT                  5
#define ACCELERATION_PID_SUM_LIMIT              INFINITY

/****************************/
/* ADC to Check akku voltage
 *
 * */
#define AKKUMONITOR_DEFAULT_PRIORITY			10
#define LOW_VOLTAGE_WARNING_THRESHOLD           10.5f // [V]
#define ONLY_USB_POWER_VOLTAGE_THRESHOLD        4.0f //[V] assume only usb
// power below this threshold

/****************************/
/* Pressure Sensor */
#define BMP180_DEFAULT_PRIORITY                 10
#define BMP180_I2C                              I2C2

/****************************/
/* IMU
 *
 */
#define IMU_DEFAULT_PRIRORITY					0

/****************************/
/* IMU
 *
 */
#define ALTIMETER_DEFAULT_PRIRORITY				0

/****************************/
/* GPS
 *
 */

#define GPS_DEFAULT_PRIORITY                    5
/****************************/
/* LEDs
 *
 */
#define LEDs_DEFAULT_PRIORITY                   20

#define UART_TRANSMIT_LED                       LED7
#define USB_TRANSMIT_LED                        LED5
#define CONFIG_LED                              LED8
#define POWER_LED                               LED4
#define FLIGHT_LED                              LED6
#define OVERLOAD_LED                            LED3
#define FLIGHT_DATA_RECEPTION_LED               LED9
#define ERROR_LED                               LED10

/****************************/
/* comms
 *
 */

/* USB
 *
 */
#define USB_DEFAULT_PRIORITY                    2
#define USB_TRANSMIT_BUSY_MAX                   2

#define USB_RXTX_BUFF_SIZE 							256

/* UART
 *
 */
#define UART_DEFAULT_PRIORITY                    2

#define UART_RXTX_BUFF_SIZE							256
/* Flight LEDs
 *
 */

/****************************/
/* Buzzer
 *
 */

#define DISABLE_LOW_VOLTAGE_BUZZER_WARNING
#define DISABLE_RC_SIGNAL_LOST_BUZZER_WARNING
#define BUZZER_DEFAULT_PRIORITY                 20
#define BUZZER_PRESCALER                        71

#define BUZZER_QUEUE_SIZE 16

/* */
#define BUZZER_PAUSE 0.0f
#define BUZZER_B5   987.767f
#define BUZZER_Ap5  932.328f
#define BUZZER_A5   880.000f
#define BUZZER_Gp5  830.609f
#define BUZZER_G5   783.991f
#define BUZZER_Fp5  739.989f
#define BUZZER_F5   698.456f
#define BUZZER_E5   659.255f
#define BUZZER_Dp5  622.254f
#define BUZZER_D5   587.330f
#define BUZZER_Cp5  554.365f
#define BUZZER_C5   523.251f
#define BUZZER_B4   493.883f
#define BUZZER_Ap4  466.164f
#define BUZZER_A4   440.000f
#define BUZZER_Gp4  415.305f
#define BUZZER_G4   391.995f
#define BUZZER_Fp4  369.994f
#define BUZZER_F4   349.228f
#define BUZZER_E4   329.628f
#define BUZZER_Dp4  311.127f
#define BUZZER_D4   293.665f
#define BUZZER_Cp4  277.183f
#define BUZZER_C4   261.626f
#define BUZZER_B3   246.942f
#define BUZZER_Ap3  233.082f
#define BUZZER_A3   220.000f
#define BUZZER_Gp3  207.652f
#define BUZZER_G3   195.998f
#define BUZZER_Fp3  184.997f
#define BUZZER_F3   174.614f
#define BUZZER_E3   164.814f
#define BUZZER_Dp3  155.563f
#define BUZZER_D3   146.832f
#define BUZZER_Cp3  138.591f
#define BUZZER_C3   130.813f
#define BUZZER_B2   123.471f
#define BUZZER_Ap2  116.541f
#define BUZZER_A2   110.000f
#define BUZZER_Gp2  103.826f
#define BUZZER_G2   97.9989f
#define BUZZER_Fp2  92.4986f
#define BUZZER_F2   87.3071f
#define BUZZER_E2   82.4069f
#define BUZZER_Dp2  77.7817f
#define BUZZER_D2   73.4162f
#define BUZZER_Cp2  69.2957f
#define BUZZER_C2   65.4064f

/*********************************************************************************/
/* hard coded settings*/
#define PID_XY_P                            0.5f
#define PID_XY_I                            0.9f
#define PID_XY_D                            0.5f
#define PID_XY_GAIN                         0.1f
#define PID_XY_SCALE                        20
#define PID_XY_CONTROL_VALUE_GAIN           0.01f

#define PID_Z_P                             0.1f
#define PID_Z_I                             0.05f
#define PID_Z_D                             0.4f
#define PID_Z_GAIN                          0.001f
#define PID_Z_SCALE                         100
#define PID_Z_CONTROL_VALUE_GAIN            0.01f

#define FILTER_COEFFICIENT_XY               0.98f
#define FILTER_COEFFICIENT_Z                0.99f

#define PID_VELOCITY_P                      0.01f
#define PID_VELOCITY_I                      0.01f
#define PID_VELOCITY_D                      0.01f
#define PID_VELOCITY_GAIN                   0.01f
#define PID_VELOCITY_SCALE                  0.01f
#define PID_VELOCITY_CONTROL_VALUE_GAIN     0.01f

#define PID_ACCELERATION_P                  0.01f
#define PID_ACCELERATION_I                  0.01f
#define PID_ACCELERATION_D                  0.01f
#define PID_ACCELERATION_GAIN               0.01f
#define PID_ACCELERATION_SCALE              0.01f
#define PID_ACCELERATION_CONTROL_VALUE_GAIN 0.01f

#endif /* CONFIG_H_ */
