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
	maxDuration = 0;
	taskStatusFlags = 0;

}

Task::~Task() {
}

void Task::update() {

	/* Standard update function */

	/* reset priority to default after execution */

}

void Task::initialize() {

	/* standard initialization function
	 *
	 * set task active */

}

void Task::resetPriority() {
	priority = defaultPriority;
}
