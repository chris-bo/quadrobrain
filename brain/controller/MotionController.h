/*
 * MotionController.h
 *
 *  Created on: Jul 8, 2015
 *      Author: bohni
 */

#ifndef MOTIONCONTROLLER_H_
#define MOTIONCONTROLLER_H_

#include <core/Task.h>
#include <controller/PIDController.h>
#include <utility/MAfilterF.h>

class MotionController: public Task {
public:
    MotionController(Status* _status, int8_t _defaultPrio);
    virtual ~MotionController();

    void initialize(PID_Settings* _velocityPIDsettings,
                PID_Settings* _accelerationPIDsettings);
    void update();
    void reset();

private:

    XYZ_Data horizontalAccelerationFiltered;

    PIDController* veloctityPIDx;
    PIDController* veloctityPIDy;
    PIDController* veloctityPIDz;

    PIDController* accelerationPIDx;
    PIDController* accelerationPIDy;
    PIDController* accelerationPIDz;

    MAfilterF* accelerationFilterX;
    MAfilterF* accelerationFilterY;
    MAfilterF* accelerationFilterZ;

    float* throttle_out;

};

#endif /* MOTIONCONTROLLER_H_ */
