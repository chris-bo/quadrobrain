/*
 * PPMGenerator.cpp
 *
 *  Created on: 30.12.2014
 *      Author: Jonas
 */

#include "PPMGenerator.h"

MotorTask::MotorTask(Status* statusPtr, uint8_t defaultPrio) :
Task(statusPtr, defaultPrio) {
	// TODO Auto-generated constructor stub

}

MotorTask::~MotorTask() {
	// TODO Auto-generated destructor stub
}

void MotorTask::update() {
	// F�r alle 4 Motoren prozentuale Throttle-Werte in Compare-Werte f�r den Timer wandeln
	// und anschlie�end dem Timer �bergeben
}

void MotorTask::initialize() {

}
