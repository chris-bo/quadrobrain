/*
 * Scheduler.h
 *
 *  Created on: Dec 10, 2014
 *      Author: bohni
 */

#ifndef SCHEDULER_H_
#define SCHEDULER_H_

#include "stm32f3xx_hal.h"
#include "config.h"
#include "Task.h"
#include "Status.h"
#include "DiscoveryLEDs.h"

class Scheduler {
public:
    Scheduler(Status* systemStatus, TIM_HandleTypeDef* htim, DiscoveryLEDs* leds);

    void start(Task** tasks, uint8_t taskAmount);
    void pause();
    void reset();
    void kill();
    void errorHandler();
    void timerIRQ();
    virtual ~Scheduler();
private:
    TIM_HandleTypeDef* scheduler_htim;
    DiscoveryLEDs* leds;
    Task** taskArray;
    Status* status;
    uint8_t numberOfTasks;
    uint8_t checkedTasks;

    uint32_t maxIdleTime;
    uint32_t minIdleTime;

    void executeTasks();
    void checkTaskDurations(uint8_t taskIndex);
    void initializeTaskDurations();

};

#endif /* SCHEDULER_H_ */
