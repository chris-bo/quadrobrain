/*
 * Task.h
 *
 *  Created on: Dec 10, 2014
 *      Author: bohni
 */

#ifndef TASK_H_
#define TASK_H_

#include "Status.h"
#include "config.h"

#define FLAG_ACTIVE		0x01
#define FLAG_CHECKED	0x02


/* TODO: statusbits besser bennenen */
#define STATUSBIT1 0b10000000
#define STATUSBIT2 0b01000000
#define STATUSBIT3 0b00100000
#define STATUSBIT4 0b00010000
#define STATUSBIT5 0b00001000
#define STATUSBIT6 0b00000100
#define STATUSBIT7 0b00000010
#define STATUSBIT8 0b00000001

class Task {
public:

	Task(Status* statusPtr, int8_t defaultPrio);
	virtual ~Task();

	virtual void update();



	uint32_t maxDuration;
	int8_t priority;

	/* status bits */
	/*
	 * 	|		|free	|free	|free	|free	|free	|checked| active| */
	uint8_t statusFlags;


protected:
	int8_t defaultPriority;
	Status* status;

};

#endif /* TASK_H_ */
