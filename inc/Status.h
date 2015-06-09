/*
 * Status.h
 *
 *  Created on: Dec 10, 2014
 *      Author: bohni
 */

#ifndef STATUS_H_
#define STATUS_H_

#include "stm32f3xx_hal.h"
#include "config.h"

class Status {
public:
    Status();
    virtual ~Status();

    void reset();
    void restoreConfig();

    /* all status variables */

    /* Akku Voltage*/
    float akkuVoltage;

    /* status flags */
    uint32_t globalFlags;

    /* acceleration [m/s ^ 2]*/
    float accelX;
    float accelY;
    float accelZ;

    /* gyro rates [°/s]*/
    float rateX;
    float rateY;
    float rateZ;

    /* Magnet fields */
    float magnetX;
    float magnetY;
    float magnetZ;

    /* angle
     * X,Y: 	angle between axis and horizont
     * North:	angle between X-Axis and North.
     * */
    float angleX;
    float angleY;
    float angleNorth;

    float temp;
    int32_t pressure;  // in Pa

    float height;       // height in m over p0
    float height_rel;   // height in m over starting point
    float d_h;          // height difference since last measurement

    /* receiver values */
    float rcSignalRoll;			// [-0.5 , 0.5]
    float rcSignalNick;			// [-0.5 , 0.5]
    float rcSignalYaw;			// [-0.5 , 0.5]
    float rcSignalThrottle;		// [0,1]
    float rcSignalLinPoti;		// [0,1]
    uint8_t rcSignalSwitch;		//
    uint8_t rcSignalEnable;		// 0 -> motors inactive
                                // 1 -> motors active

    /* Motor values
     * 1.0f ^= max
     * 0.0f ^= min
     */
    float motorValues[4];

    /* Filter Coefficients */
    float filterCoefficientXY;
    float filterCoefficientZ;

    /* PID factors */
    float pXY;
    float iXY;
    float dXY;
    float pZ;
    float iZ;
    float dZ;

    /* PID Outputs */
    float pidXOut;
    float pidYOut;
    float pidZOut;

    /* cpu load calculated via idle time*/
    float cpuLoad; // %

};

#endif /* STATUS_H_ */
