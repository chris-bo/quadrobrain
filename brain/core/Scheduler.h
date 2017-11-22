/*
 * Scheduler.h
 *
 *  Created on: Dec 10, 2014
 *      Author: bohni
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_


#include <core/Task.h>
#include <utility/DiscoveryLEDs.h>
#include "stm32f3xx_hal.h"


class Scheduler {
public:
    Scheduler(Status* systemStatus, TIM_HandleTypeDef* htim, DiscoveryLEDs* leds);

    void start(Task** tasks, uint8_t taskAmount);
    void pause();
    void reset();
    void kill();
    void timerIRQ();
    virtual ~Scheduler();
private:
    TIM_HandleTypeDef* scheduler_htim;
    DiscoveryLEDs* leds;
    Task** taskArray;
    Status* status;
    uint8_t numberOfTasks;
    uint8_t checkedTasks;

    void executeTasks();
    void checkTaskDurations(uint8_t taskIndex);
    void initializeTaskDurations();
    void overrunError();
    void errorHandler();
};

#endif /* SCHEDULER_H_ */
