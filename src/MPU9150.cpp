/*
 * MPU9150.cpp
 *
 *  Created on: Mar 21, 2015
 *      Author: bohni
 */

#include "MPU9150.h"

uint8_t i2c_buffer[20] = { 0 };

MPU9150::MPU9150(Status* statusPtr, uint8_t defaultPrio, I2C_HandleTypeDef* i2c) :
		Task(statusPtr, defaultPrio) {
	mpu9150_i2c = i2c;

	rawAccelData[0] = 0;
	rawAccelData[1] = 0;
	rawAccelData[2] = 0;

	rawGyroData[0] = 0;
	rawGyroData[1] = 0;
	rawGyroData[2] = 0;

	rawMagnetData[0] = 0;
	rawMagnetData[1] = 0;
	rawMagnetData[2] = 0;

	rawTempData = 0;
#if MPU9150_ACCEL_FULL_SCALE == MPU9150_ACCEL_FULLSCALE_2g
	scaleAccel = MPU9150_ACCEL_SCALE_FACTOR_2g * G;
#elif MPU9150_ACCEL_FULL_SCALE == MPU9150_ACCEL_FULLSCALE_4g
	scaleAccel = MPU9150_ACCEL_SCALE_FACTOR_4g * G;
#elif MPU9150_ACCEL_FULL_SCALE == MPU9150_ACCEL_FULLSCALE_8g
	scaleAccel = MPU9150_ACCEL_SCALE_FACTOR_8g * G;
#elif MPU9150_ACCEL_FULL_SCALE == MPU9150_ACCEL_FULLSCALE_16g
	scaleAccel = MPU9150_ACCEL_SCALE_FACTOR_16g * G;
#endif

#if MPU9150_GYRO_FULL_SCALE == MPU9150_GYRO_FULLSCALE_250
	scaleGyro = MPU9150_GYRO_SCALE_FACTOR_250;
#elif MPU9150_GYRO_FULL_SCALE == MPU9150_GYRO_FULLSCALE_500
	scaleGyro = MPU9150_GYRO_SCALE_FACTOR_500;
#elif MPU9150_GYRO_FULL_SCALE == MPU9150_GYRO_FULLSCALE_1000
	scaleGyro = MPU9150_GYRO_SCALE_FACTOR_1000;
#elif MPU9150_GYRO_FULL_SCALE == MPU9150_GYRO_FULLSCALE_2000
	scaleGyro = MPU9150_GYRO_SCALE_FACTOR_2000;
#endif

	scaleManget[0] = 0.3;
	scaleManget[1] = 0.3;
	scaleManget[2] = 0.3;
}

MPU9150::~MPU9150() {

}

void MPU9150::update() {

	//scaleRawData();
}

void MPU9150::initialize() {

	if ((HAL_I2C_GetState(mpu9150_i2c) == HAL_I2C_STATE_RESET)) {
		/* I2C not initialized */
		SET_FLAG(taskStatusFlags, MPU9150_FLAG_ERROR);
		return;
	}

	if (getIdentification() == 0) {
		/* Wrong Chip with same address
		 * or communication not working
		 *  */
		SET_FLAG(taskStatusFlags, MPU9150_FLAG_ERROR);
		return;
	}

	getMagnetScale();

	/* Config MPU9150
	 *
	 * MPU9150_SMPLRT_DIV       <-	MPU9150_SAMPLE_RATE
	 * MPU9150_CONFIG           <-  (0x2|MPU9150_DLPF_SETTING) (sync gyroZ + LowPass setting)
	 * MPU9150_GYRO_CONFIG      <-	MPU9150_GYRO_FULL_SCALE (no self test)
	 * MPU9150_ACCEL_CONFIG     <-  MPU9150_ACCEL_FULL_SCALE (no self test)
	 * MPU9150_INT_PIN_CFG      <-	0x30
	 * MPU9150_INT_ENABLE		<-	0x01 (DRDY interrupt)
	 * MPU9150_USER_CTRL       	<-  0x00
	 * MPU9150_PWR_MGMT_1      	<-	0x01 ( no sleep, gyro x as clock source)
	 * MPU9150_PWR_MGMT_2 		<-	0x00
	 *
	 */
	i2c_buffer[0] = MPU9150_SAMPLE_RATE;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_SMPLRT_DIV,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = (0x2 | MPU9150_DLPF_SETTING);
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_CONFIG,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = MPU9150_GYRO_FULL_SCALE;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_GYRO_CONFIG,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = MPU9150_ACCEL_FULL_SCALE;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_ACCEL_CONFIG,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = 0x30;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_INT_PIN_CFG,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = 0x01;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_INT_ENABLE,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = 0x00;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_USER_CTRL,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = 0x01;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_PWR_MGMT_1,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = 0x00;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_PWR_MGMT_2,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);



	SET_FLAG(taskStatusFlags, MPU9150_FLAG_TRANSFER_COMPLETE);
	SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);

}

void MPU9150::receptionCompleteCallback() {
	SET_FLAG(taskStatusFlags, MPU9150_FLAG_TRANSFER_COMPLETE);

	/* compute raw values*/

	rawAccelData[0] = (int16_t) (i2c_buffer[1] | (i2c_buffer[0] << 8));
	rawAccelData[1] = (int16_t) (i2c_buffer[3] | (i2c_buffer[2] << 8));
	rawAccelData[2] = (int16_t) (i2c_buffer[5] | (i2c_buffer[4] << 8));

	rawGyroData[0] = (int16_t) (i2c_buffer[7] | (i2c_buffer[6] << 8));
	rawGyroData[1] = (int16_t) (i2c_buffer[9] | (i2c_buffer[8] << 8));
	rawGyroData[2] = (int16_t) (i2c_buffer[11] | (i2c_buffer[10] << 8));

	rawMagnetData[0] = (int16_t) (i2c_buffer[15] | (i2c_buffer[14] << 8));
	rawMagnetData[1] = (int16_t) (i2c_buffer[17] | (i2c_buffer[16] << 8));
	rawMagnetData[2] = (int16_t) (i2c_buffer[19] | (i2c_buffer[18] << 8));

	rawTempData = (int16_t) (i2c_buffer[13] | (i2c_buffer[12] << 8));

	scaleRawData();

}

void MPU9150::transmissionCompleteCallback() {
	SET_FLAG(taskStatusFlags, MPU9150_FLAG_TRANSFER_COMPLETE);
}

void MPU9150::DRDYinterrupt() {

	if (GET_FLAG(taskStatusFlags, MPU9150_FLAG_TRANSFER_COMPLETE)) {
		RESET_FLAG(taskStatusFlags, MPU9150_FLAG_TRANSFER_COMPLETE);
		getAccelGyroMagnetRawData();
	} else {
		/* TODO set queue for data reception */
	}

}

void MPU9150::scaleRawData() {

	status->accelX = rawAccelData[0] * scaleAccel;
	status->accelY = rawAccelData[1] * scaleAccel;
	status->accelZ = rawAccelData[2] * scaleAccel;

	status->rateX = rawGyroData[0] * scaleGyro;
	status->rateY = rawGyroData[1] * scaleGyro;
	status->rateZ = rawGyroData[2] * scaleGyro;

	status->magnetX = rawMagnetData[0] * scaleManget[0];
	status->magnetY = rawMagnetData[1] * scaleManget[1];
	status->magnetZ = rawMagnetData[2] * scaleManget[2];

	status->temp = rawTempData * MPU9150_TEMPERATURE_SCALE_FACTOR + 35;
}

void MPU9150::getMagnetScale() {
	// TODO MPU9150::getMagnetScale()
	/* - set MPU9150 i2c bypass mode
	 * - set AK8975C rom read mode
	 * - read magn scales
	 */

}

void MPU9150::getAccelGyroMagnetRawData() {

	RESET_FLAG(taskStatusFlags, MPU9150_FLAG_TRANSFER_COMPLETE);
	HAL_I2C_Mem_Read_DMA(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_ACCEL_XOUT_H,
	I2C_MEMADD_SIZE_8BIT, i2c_buffer, 20);

}

void MPU9150::enableMagnetData() {
	/* Config Aux I2C:
	 *
	 *		Disables Fifo
	 * 		Slave 0 performs burst reads of magnetometer data
	 * 		Slave 1 periodically activates magnetometer measurements
	 *
	 */

	/* Config Slave communication sample rates
	 *
	 * MPU9150_I2C_MST_CTRL  	<- 		0x1D (stop/start between slaves + 400kHz)
	 * MPU9150_I2C_MST_DELAY_CTRL  <-	0x03 (delay slave 0 and 1)
	 * MPU9150_I2C_SLV4_CTRL	<-		(0x0 | MPU9150_EXT_SENS_I2C_DELAY) disable slave 4 + delay config
	 * MPU9150_USER_CTRL		<- 		0x20 (disable fifo + enable aux i2c)
	 * */
	i2c_buffer[0] = 0x1D;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_I2C_MST_CTRL,
	I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = 0x03;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS,
	MPU9150_I2C_MST_DELAY_CTRL, I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1,
	MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = (0x0 | MPU9150_EXT_SENS_I2C_DELAY);
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_I2C_SLV4_CTRL,
	I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = 0x20;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_USER_CTRL,
	I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	/* Config Slave 0
	 *
	 * MPU9150_I2C_SLV0_ADDR   	<-   	(AK8975C_I2C_ADDRESS | 0x80) read mode
	 * MPU9150_I2C_SLV0_REG    	<-   	AK8975C_HXL
	 * MPU9150_I2C_SLV0_CTRL   	<- 		0xC6 ( enable + swap + writereg adr + group 0 and 1 + length = 6 )
	 * */
	i2c_buffer[0] = (AK8975C_I2C_ADDRESS | 0x80);
	i2c_buffer[1] = AK8975C_HXL;
	i2c_buffer[2] = 0xC6;

	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_I2C_SLV1_ADDR,
	I2C_MEMADD_SIZE_8BIT, i2c_buffer, 3, MPU9150_INIT_TIMEOUT);

	/* Config Slave 1
	 *
	 * MPU9150_I2C_SLV1_ADDR   	<-   	AK8975C_I2C_ADDRESS | 0x00 write mode
	 * MPU9150_I2C_SLV1_REG    	<-   	AK8975C_CNTL
	 * MPU9150_I2C_SLV1_CTRL   	<- 		0x81		(enable + length = 1)
	 * MPU9150_I2C_SLV1_DO	   	<-   	0x01
	 */
	i2c_buffer[0] = AK8975C_I2C_ADDRESS;
	i2c_buffer[1] = AK8975C_CNTL;
	i2c_buffer[2] = 0x81;

	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_I2C_SLV1_ADDR,
	I2C_MEMADD_SIZE_8BIT, i2c_buffer, 3, MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = 0x01;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_I2C_SLV1_DO,
	I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

}

uint8_t MPU9150::getIdentification() {
	HAL_I2C_Mem_Read(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_WHO_AM_I,
	I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	if (i2c_buffer[0] == I_AM_MPU9150) {
		return 1;
	}
	return 0;
}

void MPU9150::disableMagnetData() {
	/* Disable Slaves 0 and 1
	 * Disable Aux I2C
	 */
	uint8_t tmp = 0;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_I2C_SLV0_CTRL,
	I2C_MEMADD_SIZE_8BIT, &tmp, 1, MPU9150_INIT_TIMEOUT);
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_I2C_SLV1_CTRL,
	I2C_MEMADD_SIZE_8BIT, &tmp, 1, MPU9150_INIT_TIMEOUT);
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_USER_CTRL,
	I2C_MEMADD_SIZE_8BIT, &tmp, 1, MPU9150_INIT_TIMEOUT);
}
