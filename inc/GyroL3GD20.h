/*
 * GyroL3GD20.h
 *
 *  Created on: Jan 12, 2015
 *      Author: bohni
 */

#ifndef GYROL3GD20_H_
#define GYROL3GD20_H_

#include "Task.h"
#include "stm32f3xx_hal.h"
#include "l3gd20.h"
#include "config.h"
#include "spi.h"

#define GYRO_READ_CMD								0x80
#define GYRO_MULTIPLE_BYTES_CMD						0x60

#define GYRO_CS_LOW()       HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_RESET)
#define GYRO_CS_HIGH()      HAL_GPIO_WritePin(GYRO_CS_GPIO_PORT, GYRO_CS_PIN, GPIO_PIN_SET)

#define GYRO_INIT_TIMEOUT							0xFFFFF

/* Accelerometer Flags */
#define GYRO_FLAG_TRANSFER_RUNNING					0x0100
#define GYRO_FLAG_TRANSFER_COMPLETE					0x0200
#define GYRO_FLAG_DATA_PROCESSED					0x0400

#define GYRO_FLAG_ERROR								0x8000

class Gyro_L3GD20: public Task {
public:
	Gyro_L3GD20(Status* statusPtr, uint8_t defaultPrio, SPI_HandleTypeDef* spi);
	virtual ~Gyro_L3GD20();
	void update();
	void receptionCompleteCallback();
	void transmissionCompleteCallback();
	void initialize();
	void getGyroData();

private:
	SPI_HandleTypeDef* gyro_spi;
	float scale;
	int16_t rawGyroValues[3]; /* X,Y,Z */

	void gyroInit();
	uint8_t getIdentification();

};

#endif /* GYROL3GD20_H_ */
