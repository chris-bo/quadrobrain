/*
 * PPMGenerator.h
 *
 *  Created on: 30.12.2014
 *      Author: Jonas
 */

#ifndef PPMGENERATOR_H_
#define PPMGENERATOR_H_

#include <core/config.h>
#include <core/Task.h>

/**TIM3 GPIO Configuration
 PC6     ------> TIM3_CH1
 PC7     ------> TIM3_CH2
 PC8     ------> TIM3_CH3
 PC9     ------> TIM3_CH4
 */

#define THROTTLE_MIN 0.1f

class PPMGenerator: public Task {
public:
    PPMGenerator(Status* statusPtr, uint8_t defaultPrio,
            TIM_HandleTypeDef* htim, float* controllerValueX,
            float* controllerValueY, float* controllerValueRotZ,
            float* throttle);
    virtual ~PPMGenerator();
    void update();
    void initialize();
    void kill();
    void disableMotors();

private:
    TIM_HandleTypeDef* PPMGenerator_htim;
    float* controllerValueX;
    float* controllerValueY;
    float* controllerValueZ;
    float* throttle;

    float motorValuesAvg[4];
    uint8_t counter;

    /* dummy padding variable */
    int pad :24;

};

#endif /* PPMGENERATOR_H_ */
