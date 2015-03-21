/*
 * PPMGenerator.cpp
 *
 *  Created on: 30.12.2014
 *      Author: Jonas
 */

#include "PPMGenerator.h"

MotorTask::MotorTask(Status* statusPtr, uint8_t defaultPrio) :
Task(statusPtr, defaultPrio) {
	// TODO Auto-generated constructor stub

}

MotorTask::~MotorTask() {
	// TODO Auto-generated destructor stub
}

void MotorTask::update() {
	// Für alle 4 Motoren prozentuale Throttle-Werte in Compare-Werte für den Timer wandeln
	// und anschließend dem Timer übergeben

	// Motor 1:
	TIM_OCInitStructure.TIM_Pulse = status->motorValues[0] / 100f * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH);
	TIM_OC1PreloadConfig(PPM_TIMER, TIM_OCPreload_Enable);
	// Motor 2:
	TIM_OCInitStructure.TIM_Pulse = status->motorValues[1] / 100f * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH);
	TIM_OC2PreloadConfig(PPM_TIMER, TIM_OCPreload_Enable);
	// Motor 3:
	TIM_OCInitStructure.TIM_Pulse = status->motorValues[2] / 100f * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH);
	TIM_OC3PreloadConfig(PPM_TIMER, TIM_OCPreload_Enable);
	// Motor 4:
	TIM_OCInitStructure.TIM_Pulse = status->motorValues[3] / 100f * (PPM_TIMER_MAX_PULSE_LENGTH - PPM_TIMER_MIN_PULSE_LENGTH);
	TIM_OC4PreloadConfig(PPM_TIMER, TIM_OCPreload_Enable);
}

void MotorTask::lowLevelInit() {
	//TODO: auf unsere Situation anpassen: Periode, Prescaler etc.

	/*Enable or disable the AHB peripheral clock */
	RCC_AHBPeriphClockCmd(PPM_PORT_PERIPH, ENABLE);

	// GPIO konfigurieren
	// Channel 1
	GPIO_InitStruct.GPIO_Pin = PPM_PIN_CHANNEL1;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(PPM_PORT_CHANNEL1, &GPIO_InitStruct);
	// Channel 2
	GPIO_InitStruct.GPIO_Pin = PPM_PIN_CHANNEL2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(PPM_PORT_CHANNEL2, &GPIO_InitStruct);
	// Channel 3
	GPIO_InitStruct.GPIO_Pin = PPM_PIN_CHANNEL3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(PPM_PORT_CHANNEL3, &GPIO_InitStruct);
	// Channel 4
	GPIO_InitStruct.GPIO_Pin = PPM_PIN_CHANNEL4;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(PPM_PORT_CHANNEL4, &GPIO_InitStruct);

	/* Connect PPM_TIMER pins to AF2 */
	GPIO_PinAFConfig(PPM_PORT_CHANNEL1, PPM_PINSOURCE_CHANNEL1, GPIO_AF_2);
	GPIO_PinAFConfig(PPM_PORT_CHANNEL2, PPM_PINSOURCE_CHANNEL2, GPIO_AF_2);
	GPIO_PinAFConfig(PPM_PORT_CHANNEL3, PPM_PINSOURCE_CHANNEL3, GPIO_AF_2);
	GPIO_PinAFConfig(PPM_PORT_CHANNEL4, PPM_PINSOURCE_CHANNEL4, GPIO_AF_2);

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Globaler Interrupt für PWM
	NVIC_InitStructure.NVIC_IRQChannel = PPM_TIMER_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure); */

	//  PPM Timer Peripherie aktivieren
	RCC_APB1PeriphClockCmd(PPM_PERIPH, ENABLE);

	// Timer konfigurieren
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = PPM_TIMER_PRESCALER;
	TIM_TimeBaseStructure.TIM_Period = PPM_TIMER_PERIOD - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(PPM_TIMER, &TIM_TimeBaseStructure);

	// Timerkanäle initialisieren
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;

	TIM_OCInitStructure.TIM_Pulse = 0; // Pulsweite 0...PPM_TIMER_PERIOD
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; // Pulspolarität
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

	// Timersettings
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Set;

	// 4 Kanäle einrichten
	// Channel 1, pin PD10?
	TIM_OC1Init(PPM_TIMER, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(PPM_TIMER, TIM_OCPreload_Enable);

	// Channel 2, pin PD11?
	TIM_OC2Init(PPM_TIMER, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(PPM_TIMER, TIM_OCPreload_Enable);

	// Channel 3, pin PD12?
	TIM_OC3Init(PPM_TIMER, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(PPM_TIMER, TIM_OCPreload_Enable);

	// Channel 4, pin PD13?
	TIM_OC4Init(PPM_TIMER, &TIM_OCInitStructure);
	TIM_OC4PreloadConfig(PPM_TIMER, TIM_OCPreload_Enable);

	// Timer starten
	TIM_ARRPreloadConfig(PPM_TIMER, DISABLE);
	TIM_CtrlPWMOutputs(PPM_TIMER, ENABLE);
	TIM_Cmd(PPM_TIMER, ENABLE);
}
