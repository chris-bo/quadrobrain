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


}

void AkkuMonitor::update() {


	if (counter == MEASUREMENT_FREQUENCY * 1000 / SCHEDULER_INTERVALL_ms) {
			HAL_ADC_Start(akkumonitor_adc);
			counter = 0;
	} else if (counter == 0) {

		HAL_ADC_Stop(akkumonitor_adc);
		uint32_t voltage_tmp= HAL_ADC_GetValue(akkumonitor_adc);

		status->akkuVoltage = voltage_tmp * VOLTAGE_DIVIDER_RATIO;
		counter ++;
	} else {
		counter ++;
	}

	resetPriority();
}
