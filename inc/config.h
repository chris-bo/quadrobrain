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
#define SCHEDULER_TIMER				TIM2
#define SCHEDULER_TIMER_CLOCK 		RCC_APB1Periph_TIM2
#define SCHEDULER_TIMER_IRQn		TIM2_IRQn
#define SCHEDULER_INTERVALL_ms		10

/* RC receiver
 * Input capture
 */
#define RC_RECEIVER_TIMER				TIM4
#define RC_RECEIVER_TIMER_CLOCK 		RCC_APB1Periph_TIM4
#define RC_RECEIVERTIMER_IRQn			TIM4_IRQn

#define RC_RECEIVER_GPIO_PORT			GPIOB
#define RC_RECEIVER_GPIO_PORT_CLOCK		RCC_AHBPeriph_GPIOB
#define RC_RECEIVER_GPIO_PIN			GPIO_Pin_8	// TIM4 CH3 (CH1 and CH2 used by I2C1)
#define RC_RECEIVER_GPIO_PIN_SOURCE   	GPIO_PinSource8
#define RC_RECEIVER_GPIO_PIN_AF			GPIO_AF_2




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
