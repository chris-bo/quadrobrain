/*
 * IMU.h
 *
 *  Created on: Nov 16, 2017
 *      Author: chris
 */

#ifndef IMU_H_
#define IMU_H_


#include <core/Task.h>

#include <sensor/ComplementaryFilter.h>
#include <sensor/Compass.h>
#include <utility/MAfilterF.h>


class IMU : public Task {
public:
	IMU(Status* _status, int8_t _defaultPrio);
	virtual ~IMU();

    void initialize();
    void update();
    void reset();

private:

    /* Sensor data fusion Filters*/
    ComplementaryFilter* compFilterX;
    ComplementaryFilter* compFilterY;
    Compass* compFilterNorth;

    /* unfiltered data for horizontal acceleration */
    XYZ_Data horizontalAccelerationUnfiltered;

    /* Filter accelerations */
//    MAfilterF* accelerationFilterX;
//    MAfilterF* accelerationFilterY;
//    MAfilterF* accelerationFilterZ;



};

#endif /* IMU_H_ */
