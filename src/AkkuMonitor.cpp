/*
 * AkkuMonitor.cpp
 *
 *  Created on: Apr 25, 2015
 *      Author: bohni
 */

#include "AkkuMonitor.h"

AkkuMonitor::AkkuMonitor(Status* statusPtr, uint8_t defaultPrio,
            ADC_HandleTypeDef* hadc)
            : Task(statusPtr, defaultPrio) {
    counter = 0;
    akkumonitor_adc = hadc;

}

AkkuMonitor::~AkkuMonitor() {

}

void AkkuMonitor::initialize() {

    /* calibration */
    HAL_ADCEx_Calibration_Start(akkumonitor_adc, ADC_SINGLE_ENDED);
    HAL_Delay(5);
    uint32_t tmp = HAL_ADCEx_Calibration_GetValue(akkumonitor_adc, ADC_SINGLE_ENDED);
    HAL_ADCEx_Calibration_SetValue(akkumonitor_adc, ADC_SINGLE_ENDED, tmp);
}

void AkkuMonitor::update() {

    if (counter == 1000 / SCHEDULER_INTERVALL_ms / MEASUREMENT_FREQUENCY) {
        HAL_ADC_Start_IT(akkumonitor_adc);
        counter = 0;

    } else {
        counter++;
    }
}

void AkkuMonitor::conversionComplete() {

    uint32_t tmp = HAL_ADC_GetValue(akkumonitor_adc);
    status->akkuVoltage = (float) (tmp * SCALE_FACTOR * VOLTAGE_DIVIDER_RATIO);
    HAL_ADC_Stop_IT(akkumonitor_adc);

    if (status->akkuVoltage > ONLY_USB_POWER_VOLTAGE_THRESHOLD) {
        if (status->akkuVoltage < LOW_VOLTAGE_WARNING_THRESHOLD) {
            lowVoltageWarning();
        } else {
            RESET_FLAG(status->globalFlags, (LOW_VOLTAGE_FLAG | ERROR_FLAG));
        }
    }
    // else ignore akku voltage
}

void AkkuMonitor::lowVoltageWarning() {
    /* akkuVoltate below threshold */
    SET_FLAG(status->globalFlags, (LOW_VOLTAGE_FLAG | ERROR_FLAG));
    if (status->qcSettings.enableBuzzerWarningLowVoltage) {
        status->addToneToQueue(&status->buzzerQueue2, BUZZER_PAUSE, 10);
        status->addToneToQueue(&status->buzzerQueue2, BUZZER_A5, 100);
        status->addToneToQueue(&status->buzzerQueue2, BUZZER_PAUSE, 10);
        status->addToneToQueue(&status->buzzerQueue2, BUZZER_B5, 100);
        status->addToneToQueue(&status->buzzerQueue1, BUZZER_PAUSE, 10);
        status->addToneToQueue(&status->buzzerQueue1, BUZZER_B5, 100);
        status->addToneToQueue(&status->buzzerQueue1, BUZZER_PAUSE, 10);
        status->addToneToQueue(&status->buzzerQueue1, BUZZER_A5, 100);
    }

}
