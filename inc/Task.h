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

#define TASK_FLAG_ACTIVE		0x0001
#define TASK_FLAG_CHECKED		0x0002

class Task {
public:

    Task(Status* statusPtr, int8_t defaultPrio);
    virtual ~Task();

    virtual void update();
    virtual void initialize();
    virtual void reset();
    virtual void kill();

    uint32_t maxDuration;
    int8_t priority;
    int8_t defaultPriority;
    /* status bits */
    /* first byte: child class specific flags
     *
     * second byte:
     * flags handled by scheduler:
     * 	|		|free	|free	|free	|free	|free	|checked| active| */
    uint16_t taskStatusFlags;

protected:

    Status* status;

    void resetPriority();
};

#endif /* TASK_H_ */
