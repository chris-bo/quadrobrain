/*
 * AkkuMonitor.h
 *
 *  Created on: Apr 25, 2015
 *      Author: bohni
 */

#ifndef AKKUMONITOR_H_
#define AKKUMONITOR_H_


#include "Task.h"
#include "config.h"


/*********************************************************
 *
 *  ADC in : PA1
 *
 *  */

#define REFERENCE_VOLTAGE			2.97f	// V
//#define VOLTAGE_DIVIDER_RATIO 		5.68f
#define VOLTAGE_DIVIDER_RATIO       7.193f
#define MEASUREMENT_FREQUENCY		1  // Hz
#define SCALE_FACTOR				(REFERENCE_VOLTAGE / 4095.00f)

/****************************/

class AkkuMonitor: public Task {
public:
    AkkuMonitor(Status* statusPtr, uint8_t defaultPrio, ADC_HandleTypeDef* hadc);
    virtual ~AkkuMonitor();

    void initialize();
    void update();
    void conversionComplete();

private:
    ADC_HandleTypeDef* akkumonitor_adc;
    uint16_t counter;

    void lowVoltageWarning();
};

#endif /* AKKUMONITOR_H_ */
