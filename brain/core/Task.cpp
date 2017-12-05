/*
 * Task.cpp
 *
 *  Created on: Dec 10, 2014
 *      Author: bohni
 */

#include "Task.h"
/* Priority:	-1 = deactivated
 * 				 0 = force execute
 * 				 1 = highest normal
 */
Task::Task(Status* statusPtr, int8_t defaultPrio) {

    status = statusPtr;
    if (defaultPrio == 1) {
        defaultPrio = 2;
    }

    defaultPriority = defaultPrio;
    priority = defaultPriority;
    duration = 0;
    taskActive = false;
    taskChecked = false;

    /* dummy padding variable */
    pad = 0;
}

Task::~Task() {
}

void Task::update() {

    /* Standard update function */

}

void Task::initialize() {

    /* standard initialization function
     *
     * set task active */
    taskActive = true;

}

void Task::resetPriority() {
    priority = defaultPriority;
}

void Task::reset() {

    priority = defaultPriority;
    duration = 0;
    //  taskActive = false;

}

void Task::kill() {
    this->reset();
    taskActive = false;
}
