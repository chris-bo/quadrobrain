/*
 * MPU9150.cpp
 *
 *  Created on: Mar 21, 2015
 *      Author: bohni
 */

#include "MPU9150.h"

MPU9150::MPU9150(Status* statusPtr, uint8_t defaultPrio, I2C_HandleTypeDef* i2c) :
		Task(statusPtr, defaultPrio) {
	mpu9150_i2c = i2c;

}

MPU9150::~MPU9150() {

}

void MPU9150::update() {
}

void MPU9150::initialize() {
}
