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
#include "MAfilterF.h"

class HorizontalMotionControl: public Task {
public:
    HorizontalMotionControl(Status* _status, int8_t _defaultPrio);
    virtual ~HorizontalMotionControl();

    void initialize(PID_Settings* _velocityPIDsettings,
                PID_Settings* _accelerationPIDsettings);
    void update();
    void reset();

private:

    /* take these directly from status*/

    XYZ_Data horizontalAcceleration;
    XYZ_Data accelerationSetpoint;

    PIDController* veloctityPIDx;
    PIDController* veloctityPIDy;
    PIDController* veloctityPIDz;

    PIDController* accelerationPIDx;
    PIDController* accelerationPIDy;
    PIDController* accelerationPIDz;

    float* throttle_out;

};

#endif /* HORIZONTALMOTIONCONTROL_H_ */
