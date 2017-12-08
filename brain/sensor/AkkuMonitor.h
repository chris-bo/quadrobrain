/*
 * AkkuMonitor.h
 *
 *  Created on: Apr 25, 2015
 *      Author: bohni
 */

#ifndef AKKUMONITOR_H_
#define AKKUMONITOR_H_

#include <core/Task.h>

/*********************************************************
 *
 *  ADC in : PA1
 *
 *  */

#define REFERENCE_VOLTAGE			2.97f	// V
//#define VOLTAGE_DIVIDER_RATIO 		5.68f
#define VOLTAGE_DIVIDER_RATIO       7.193f
#define MEASUREMENT_FREQUENCY		1  // Hz
#define SCALE_FACTOR				((float) REFERENCE_VOLTAGE / 4095.0f)

/****************************/

class AkkuMonitor: public Task {
public:
    AkkuMonitor(Status* statusPtr, uint8_t defaultPrio,
            ADC_HandleTypeDef* hadc);
    virtual ~AkkuMonitor();

    void initialize();
    void update();
    void conversionComplete();

private:
    ADC_HandleTypeDef* akkumonitor_adc;
    uint16_t counter;

    /* dummy padding variable */
    int pad :16;
    void lowVoltageWarning();
};

#endif /* AKKUMONITOR_H_ */
