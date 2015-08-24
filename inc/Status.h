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
#include "GPS_defines.h"

typedef struct {
    float x;
    float y;
    float z;
}XYZ_Data;

typedef struct {
    float p;
    float i;
    float d;
    /* scales setPoint to fit processVariable */
    float scaleSetPoint;
    /* overall gain added to pid output*/
    float gain;
}PID_Settings;

typedef struct {
    float frequency[BUZZER_QUEUE_SIZE];
    uint16_t lenght[BUZZER_QUEUE_SIZE];
    uint8_t index;
    uint8_t currentTone;
}Buzzer_Queue_t;

typedef struct {
    uint8_t enableBuzzerWarningLowVoltage;
    uint8_t enableBuzzerWarningRCLost;
    uint8_t enableFlightLeds;
    uint8_t enableMotors;
}Quadrocopter_Settings_t;

class Status {
public:
    Status();
    virtual ~Status();

    void reset();
    void restoreConfig();
    void addToneToQueue(Buzzer_Queue_t* queue, float frequency, uint16_t length);

    /* all status variables */

    /* uptime since power on [SCHEDULER_INTERVALL_ms]*/

    uint32_t uptime;

    /* Akku Voltage*/
    float akkuVoltage;

    /* status flags */
    uint32_t globalFlags;

    /* acceleration [m/s ^ 2]*/
    XYZ_Data accel;

    /* gyro rates [°/s]*/
    XYZ_Data rate;

    /* Magnet fields */
    XYZ_Data magnetfield;

    /* angle
     * X,Y: 	angle between axis and horizont
     * Z:	angle between X-Axis and North.
     * */
    XYZ_Data angle;
    XYZ_Data angleSetpoint;

    /* velocity setpoint */
    XYZ_Data velocitySetpoint;
    XYZ_Data velocity;

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

    /* PID settings */
    PID_Settings pidSettigsAngleXY;
    PID_Settings pidSettigsRotationZ;

    PID_Settings pidSettingsVelocity;
    PID_Settings pidSettingsacceleration;

    /* PID Outputs */
    XYZ_Data motorSetpoint;

    /* cpu load calculated via idle time*/
    float cpuLoad; // %

    Buzzer_Queue_t buzzerQueue1;
    Buzzer_Queue_t buzzerQueue2;

    /* GPS Data*/
    GPS_Data_t gpsData;

    /* QC Settings */
    Quadrocopter_Settings_t qcSettings;
};

#endif /* STATUS_H_ */
