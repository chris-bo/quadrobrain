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
	taskStatusFlags = 0;

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
    SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);

}

void Task::resetPriority() {
	priority = defaultPriority;
}

void Task::reset() {

    priority = defaultPriority;
    duration = 0;
    taskStatusFlags = 0;

}

void Task::kill() {
    this->reset();
    RESET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
}
