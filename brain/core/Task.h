/*
 * Task.h
 *
 *  Created on: Dec 10, 2014
 *      Author: bohni
 */

#ifndef TASK_H_
#define TASK_H_

#include <core/Status.h>
#include <core/config.h>

class Task {
public:

    Task(Status* statusPtr, int8_t defaultPrio);
    virtual ~Task();

    virtual void update();
    virtual void initialize();
    virtual void reset();
    virtual void kill();
    void resetPriority();
    uint32_t duration;
    int8_t priority;
    int8_t defaultPriority;

    bool taskActive :1;
    bool taskChecked :1;
protected:

    /* dummy padding variable */
    int pad :14;

    Status* status;

};

#endif /* TASK_H_ */
