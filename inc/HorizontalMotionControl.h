/*
 * HorizontalMotionControl.h
 *
 *  Created on: Jul 8, 2015
 *      Author: bohni
 */

#ifndef HORIZONTALMOTIONCONTROL_H_
#define HORIZONTALMOTIONCONTROL_H_

#include "Task.h"
#include "PIDController.h"

class HorizontalMotionControl: public Task {
public:
    HorizontalMotionControl();
    virtual ~HorizontalMotionControl();

    void initialize();
    void update();
    void reset();

private:
    XYZ_Data horizontalAcceleration;
    XYZ_Data accelerationSetpoint;



};

#endif /* HORIZONTALMOTIONCONTROL_H_ */
