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
    void initialize(float* _p, float* _i, float* _d, float* _gain, float* _scale);
    void kill();
private:
    Status* status;
    /* PID coefficients*/
    float* p;
    float* i;
    float* d;

    /* measured input  */
    float* processVariable;
    float* derivedProcessVariable;

    /* desired value */
    float* setPoint;
    /* scales setPoint to fit processVariable */
    float* scale;

    /* overall gain added to pid output*/
    float* gain;

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
