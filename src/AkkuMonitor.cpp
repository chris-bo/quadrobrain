/*
 * AkkuMonitor.cpp
 *
 *  Created on: Apr 25, 2015
 *      Author: bohni
 */

#include "AkkuMonitor.h"

AkkuMonitor::AkkuMonitor(Status* statusPtr, uint8_t defaultPrio,
        ADC_HandleTypeDef* hadc) : Task (statusPtr, defaultPrio) {
	counter = 0;
	akkumonitor_adc = hadc;


}

AkkuMonitor::~AkkuMonitor() {

}

void AkkuMonitor::initialize() {


	/* calibration */
	HAL_ADCEx_Calibration_Start(akkumonitor_adc,ADC_SINGLE_ENDED);
	HAL_Delay(5);
	uint32_t tmp = HAL_ADCEx_Calibration_GetValue(akkumonitor_adc,ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_SetValue(akkumonitor_adc,ADC_SINGLE_ENDED,tmp);
}

void AkkuMonitor::update() {


	if (counter == MEASUREMENT_FREQUENCY * 1000 / SCHEDULER_INTERVALL_ms) {
			HAL_ADC_Start_IT(akkumonitor_adc);
			counter = 0;
	} else {
		counter ++;
	}

	resetPriority();
}

void AkkuMonitor::conversionComplete() {

	uint32_t tmp =	HAL_ADC_GetValue(akkumonitor_adc);
	status->akkuVoltage = tmp * SCALE_FACTOR * VOLTAGE_DIVIDER_RATIO;
	HAL_ADC_Stop_IT(akkumonitor_adc);


}
