/*
 * IMU.h
 *
 *  Created on: Nov 16, 2017
 *      Author: chris
 */

#ifndef IMU_H_
#define IMU_H_


#include "Task.h"

#include "ComplementaryFilter.h"
#include "Compass.h"
#include "MAfilterF.h"


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

    /* filter for horizontal acceleration */
    XYZ_Data horizontalAccelerationUnfiltered;

    MAfilterF* accelerationFilterX;
    MAfilterF* accelerationFilterY;
    MAfilterF* accelerationFilterZ;



};

#endif /* IMU_H_ */
