/*
 * kalman.cpp
 *
 *  Created on: Jan 18, 2015
 *      Author: bohni
 */

#include "Kalman.h"



Kalman::Kalman(Status* statusPtr, uint8_t defaultPrio) : Task(statusPtr, defaultPrio) {
	dt = SCHEDULER_INTERVALL_ms * 0.001f;
	angle = 0;
	rotation = 0;
	rotation_bias = 0;
	acc_z = 0;
	acc_axis = 0;


	Q[0] = Q1;
	Q[1] = Q2;
	Q[2] = Q3;
	R[0] = R1;
	R[1] = R2;
	P[0][0] = 1.0f;
	P[1][1] = 1.0f;
	P[2][2] = 1.0f;

	P[0][1] = 0.0f;
	P[0][2] = 0.0f;
	P[1][0] = 0.0f;
	P[1][2] = 0.0f;
	P[2][0] = 0.0f;
	P[2][1] = 0.0f;
}

void Kalman::update() {
	/* Kalman Update sequence */

	/* 1. update angle */






}

void Kalman::initialize(float* ang, float* rot, float* rot_bias,
		float* acc_z_, float* acc_xy) {
	angle =ang;
	rotation = rot;
	rotation_bias = rot_bias;
	acc_z = acc_z_;
	acc_axis = acc_xy;



	SET_FLAG(taskStatusFlags,TASK_FLAG_ACTIVE);
}

Kalman::~Kalman() {

}

