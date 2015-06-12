/*
 * PPMGenerator.h
 *
 *  Created on: 30.12.2014
 *      Author: Jonas
 */

#ifndef PPMGENERATOR_H_
#define PPMGENERATOR_H_

#include "config.h"
#include "Task.h"

/**TIM3 GPIO Configuration
 PC6     ------> TIM3_CH1
 PC7     ------> TIM3_CH2
 PC8     ------> TIM3_CH3
 PC9     ------> TIM3_CH4
 */

#define THROTTLE_MIN 0.1f

class PPMGenerator: public Task {
public:
    PPMGenerator(Status* statusPtr, uint8_t defaultPrio, TIM_HandleTypeDef* htim,
                float* controllerValueX, float* controllerValueY);
    virtual ~PPMGenerator();
    void update();
    void initialize();
    void kill();
    void disableMotors();

private:
    TIM_HandleTypeDef* PPMGenerator_htim;
    float* controllerValueX;
    float* controllerValueY;
};

#endif /* PPMGENERATOR_H_ */
