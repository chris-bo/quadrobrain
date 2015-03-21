/*
 * kalman.h
 *
 *  Created on: Jan 18, 2015
 *      Author: bohni
 */

#ifndef KALMAN_H_
#define KALMAN_H_

#include "Task.h"
#include "config.h"

#define Q1			0.001f
#define Q2			0.001f
#define Q3			0.001f

#define R1			0.001f
#define R2			0.001f


class Kalman: public Task {
public:
	Kalman(Status* statusPtr, uint8_t defaultPrio);

	void update();
	void initialize(float* ang, float* rot, float* rot_bias, float* acc_z_, float* acc_xy);
	virtual ~Kalman();

private:
	float dt;

	float* angle;
	float* rotation;
	float* rotation_bias;
	float* acc_z;
	float* acc_axis;


	float Q[3];
	float R[2];
	float P[3][3];

};

#endif /* KALMAN_H_ */
