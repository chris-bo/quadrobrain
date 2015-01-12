/*
 * GyroL3GD20.cpp
 *
 *  Created on: Jan 12, 2015
 *      Author: bohni
 */

#include "GyroL3GD20.h"

uint8_t gyro_tx_buffer[7];
uint8_t gyro_rx_buffer[7];
Gyro_L3GD20::Gyro_L3GD20(Status* statusPtr, uint8_t defaultPrio,
		SPI_HandleTypeDef* spi) :
		Task(statusPtr, defaultPrio) {
	gyro_spi = spi;
	scale = 1;
	rawGyroValues[0] = 0;
	rawGyroValues[1] = 0;
	rawGyroValues[2] = 0;

#if (GYRO_RANGE == L3GD20_FULLSCALE_250)
	scale = L3GD20_SENSITIVITY_250DPS * 0.001f;
#elif (GYRO_RANGE == L3GD20_FULLSCALE_500)
	scale = L3GD20_SENSITIVITY_500DPS * 0.001f;
#elif (GYRO_RANGE == L3GD20_FULLSCALE_2000)
	scale = L3GD20_SENSITIVITY_500DPS * 0.001f;
#else
#error "GYRO_RANGE not set"
#endif

}

Gyro_L3GD20::~Gyro_L3GD20() {
}

void Gyro_L3GD20::update() {
	status->rateX = rawGyroValues[0] * scale;
	status->rateY = rawGyroValues[1] * scale;
	status->rateZ = rawGyroValues[2] * scale;

	if (GET_FLAG(taskStatusFlags, GYRO_FLAG_DATA_PROCESSED)) {
		/* no interrupt occured since last update*/
		RESET_FLAG(taskStatusFlags, GYRO_FLAG_DATA_PROCESSED);
		getGyroData();
	} else {
		SET_FLAG(taskStatusFlags, GYRO_FLAG_DATA_PROCESSED);
	}

}

void Gyro_L3GD20::receptionCompleteCallback() {
	/* save Raw Values
	 * LSB @ lower address
	 * */
	rawGyroValues[0] = (int16_t) (gyro_rx_buffer[0] | (gyro_rx_buffer[1] << 8));
	rawGyroValues[1] = (int16_t) (gyro_rx_buffer[2] | (gyro_rx_buffer[3] << 8));
	rawGyroValues[2] = (int16_t) (gyro_rx_buffer[4] | (gyro_rx_buffer[5] << 8));

	RESET_FLAG(taskStatusFlags, GYRO_FLAG_TRANSFER_RUNNING);
	SET_FLAG(taskStatusFlags, GYRO_FLAG_TRANSFER_COMPLETE);
	RESET_FLAG(taskStatusFlags, GYRO_FLAG_DATA_PROCESSED);

	/* todo error management */
}

void Gyro_L3GD20::transmissionCompleteCallback() {
	HAL_SPI_Receive_DMA(gyro_spi, gyro_rx_buffer, 6);
}

void Gyro_L3GD20::initialize() {

	HAL_SPI_MspInit(gyro_spi);
	GYRO_CS_HIGH();

	gyroInit();

	/* Set Flags */
	SET_FLAG(taskStatusFlags, GYRO_FLAG_TRANSFER_COMPLETE);

	getGyroData();
}

void Gyro_L3GD20::getGyroData() {
	if (GET_FLAG(taskStatusFlags, GYRO_FLAG_TRANSFER_COMPLETE)) {
		SET_FLAG(taskStatusFlags, GYRO_FLAG_TRANSFER_RUNNING);
		gyro_tx_buffer[0] = GYRO_MULTIPLE_BYTES_CMD | GYRO_READ_CMD
				| L3GD20_OUT_X_L_ADDR;
		GYRO_CS_LOW();
		HAL_SPI_Transmit_IT(gyro_spi, gyro_tx_buffer, 1);
		//	HAL_SPI_Receive_IT(gyro_spi, gyro_rx_buffer,6);
	}
}

void Gyro_L3GD20::gyroInit() {
	/*check device */
	if (getIdentification() == 1) {

		/* Config Gyro*/

		/* ADR + Multibyte */
		gyro_tx_buffer[0] = GYRO_MULTIPLE_BYTES_CMD | L3GD20_CTRL_REG1_ADDR;

		/* CTRL1
		 *
		 */
		gyro_tx_buffer[1] = GYRO_OUTPUT_RATE | GYRO_BANDWIDTH
				| L3GD20_MODE_ACTIVE | L3GD20_AXES_ENABLE;

		/* CTRL2
		 *  High Pass Config
		 */
		gyro_tx_buffer[2] = 0;

		/* CTRL3
		 * DRDY on int 2
		 */
		gyro_tx_buffer[3] = 0x08;

		/* CTRL4
		 * LSB @ lower adr
		 * continuous update
		 * Range setting
		 */
		gyro_tx_buffer[4] = L3GD20_BLE_LSB | L3GD20_BlockDataUpdate_Continous
				| GYRO_RANGE;

		/* CTRL5
		 * fifo disable
		 *
		 */
		gyro_tx_buffer[5] = 0;

		/* send config
		 * blocking mode
		 * */
		GYRO_CS_LOW();
		HAL_SPI_Transmit(gyro_spi, gyro_tx_buffer, 6, GYRO_INIT_TIMEOUT);
		GYRO_CS_HIGH();
	}
}

uint8_t Gyro_L3GD20::getIdentification() {

	gyro_tx_buffer[0] = L3GD20_WHO_AM_I_ADDR | GYRO_READ_CMD;
	GYRO_CS_LOW();
	HAL_SPI_TransmitReceive(gyro_spi, gyro_tx_buffer, gyro_rx_buffer, 2,
			GYRO_INIT_TIMEOUT);
	GYRO_CS_HIGH();
	if (gyro_rx_buffer[1] == I_AM_L3GD20) {
		return 1;
	} else {
		/* if not l3gd20 */
		RESET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);
		priority = -1;
		SET_FLAG(taskStatusFlags, GYRO_FLAG_ERROR);
		SET_FLAG(status->globalFlags, EMERGENCY_FLAG);
		/*stop program*/
		while (1)
			;
		return 0;
	}

}
