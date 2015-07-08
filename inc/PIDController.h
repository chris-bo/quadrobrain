/*
 * PIDController.h
 *
 *  Created on: 22.03.2015
 *      Author: Jonas
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include "Task.h"
#include "config.h"

class PIDController: public Task {
public:
    PIDController(Status* statusPtr, uint8_t defaultPrio, float _sampleTime,
                float* _processVariable, float* _derivedProcessVariable,
                float* _setPoint, float* _controlValue, float _controlValueGain, float _limit,
                float _sumLimit, bool _useDerivedInput);
    virtual ~PIDController();
    void update();
    void initialize(PID_Settings* _settings);
    void kill();
private:
    Status* status;
    /* PID settings */
    PID_Settings* settings;


    /* measured input  */
    float* processVariable;
    float* derivedProcessVariable;

    /* desired value */
    float* setPoint;

    /* output*/
    float* controlValue;
    float controlValueGain;

    float eSum;
    float oldValue;

    /* settings*/
    float limit; /* output limit*/
    float sumLimit; /* e_sum limit*/
    float sampleTime;
    /* use derived input for differential part*/
    bool useDerivedInput;
};

#endif /* PIDCONTROLLER_H_ */
