/*
 * MPU9150.cpp
 *
 *  Created on: Mar 21, 2015
 *      Author: bohni
 */

#include "MPU9150.h"

uint8_t i2c_buffer[38] = { 0 };

MPU9150::MPU9150(Status* statusPtr, uint8_t defaultPrio, I2C_HandleTypeDef* i2c)
		: Task(statusPtr, defaultPrio) {
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

	scaleMagnet[0] = 0.3;
	scaleMagnet[1] = 0.3;
	scaleMagnet[2] = 0.3;
}

MPU9150::~MPU9150() {

}

void MPU9150::update() {
	// TODO mpu9150 CHECK errors

}

void MPU9150::initialize(uint8_t gyro_full_scale, uint8_t accel_full_scale) {


	/* reinit I2C
	 * fix for broken communication after ConfigReader fails
	 * to read from eeprom
	 */
	HAL_I2C_Init(mpu9150_i2c);


	if (getIdentification() == 0) {
		/* Wrong Chip with same address
		 * or communication not working
		 *  */
		SET_FLAG(taskStatusFlags, MPU9150_FLAG_ERROR);
		return;

	}

	/* reset MPU9150*/
	i2c_buffer[0] = 0x80;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_PWR_MGMT_1,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);
	HAL_Delay(200);


	getMagnetScale();

	/* Config MPU9150
	 *
	 * MPU9150_SMPLRT_DIV       <-	MPU9150_SAMPLE_RATE
	 * MPU9150_CONFIG           <-  (0x2|MPU9150_DLPF_SETTING) (sync gyroZ + LowPass setting)
	 * MPU9150_GYRO_CONFIG      <-	MPU9150_GYRO_FULL_SCALE (no self test)
	 * MPU9150_ACCEL_CONFIG     <-  MPU9150_ACCEL_FULL_SCALE (no self test)
	 * MPU9150_INT_PIN_CFG      <-	0x00
	 * MPU9150_INT_ENABLE		<-	0x01 (DRDY interrupt)
	 * MPU9150_USER_CTRL       	<-  0x00
	 * MPU9150_PWR_MGMT_1      	<-	0x01 ( no sleep, gyro x as clock source)
	 * MPU9150_PWR_MGMT_2 		<-	0x00
	 *
	 */
	i2c_buffer[0] = 0x01;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_PWR_MGMT_1,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);
	HAL_Delay(200);

	i2c_buffer[0] = MPU9150_SAMPLE_RATE;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_SMPLRT_DIV,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = (0x2 | MPU9150_DLPF_SETTING);
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_CONFIG,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = 0x00;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_INT_PIN_CFG,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = 0x01;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_INT_ENABLE,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = 0x00;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_USER_CTRL,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	i2c_buffer[0] = 0x00;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_PWR_MGMT_2,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);



	getOffsetRegisters();

	configFullScale(gyro_full_scale, accel_full_scale);

	enableMagnetData();

	SET_FLAG(taskStatusFlags, TASK_FLAG_ACTIVE);

}

void MPU9150::receptionCompleteCallback() {


	/* compute raw values*/

	rawAccelData[0] = (int16_t) (i2c_buffer[1] | (i2c_buffer[0] << 8));
	rawAccelData[1] = (int16_t) (i2c_buffer[3] | (i2c_buffer[2] << 8));
	rawAccelData[2] = (int16_t) (i2c_buffer[5] | (i2c_buffer[4] << 8));

	rawGyroData[0] = (int16_t) (i2c_buffer[9] | (i2c_buffer[8] << 8));
	rawGyroData[1] = (int16_t) (i2c_buffer[11] | (i2c_buffer[10] << 8));
	rawGyroData[2] = (int16_t) (i2c_buffer[13] | (i2c_buffer[12] << 8));

	rawMagnetData[0] = (int16_t) (i2c_buffer[15] | (i2c_buffer[14] << 8));
	rawMagnetData[1] = (int16_t) (i2c_buffer[17] | (i2c_buffer[16] << 8));
	rawMagnetData[2] = (int16_t) (i2c_buffer[19] | (i2c_buffer[18] << 8));

	rawTempData = (int16_t) (i2c_buffer[7] | (i2c_buffer[6] << 8));

	/* call scaling routine*/
	scaleRawData();

}

void MPU9150::transmissionCompleteCallback() {

}



void MPU9150::scaleRawData() {

	status->accelX = rawAccelData[0] * scaleAccel;
	status->accelY = rawAccelData[1] * scaleAccel;
	status->accelZ = rawAccelData[2] * scaleAccel;

	status->rateX = rawGyroData[0] * scaleGyro;
	status->rateY = rawGyroData[1] * scaleGyro;
	status->rateZ = rawGyroData[2] * scaleGyro;

	status->magnetX = rawMagnetData[0] * scaleMagnet[0];
	status->magnetY = rawMagnetData[1] * scaleMagnet[1];
	status->magnetZ = rawMagnetData[2] * scaleMagnet[2];

	status->temp = rawTempData * MPU9150_TEMPERATURE_SCALE_FACTOR + 35;

	if ( GET_FLAG(taskStatusFlags,MPU9150_FLAG_CONTINUOUS_RECEPTION )) {
		getAccelGyroMagnetRawData();
	}
}

void MPU9150::getMagnetScale() {
	// TODO MPU9150::getMagnetScale()
	/* - set MPU9150 i2c bypass mode
	 * - set AK8975C rom read mode
	 * - read magn scales
	 */

}

void MPU9150::getAccelGyroMagnetRawData() {

	HAL_I2C_Mem_Read_DMA(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_ACCEL_XOUT_H,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 38);

	// TODO mpu9150 I2C busy management

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

void MPU9150::getOffsetRegisters() {

	/* TODO mpu9150 check if using hard coded values is sufficient */


	int32_t offset_tmp[3] = { 0, 0, 0 };
	int16_t raw_tmp[3] = { 0, 0, 0 };
	int16_t accel_offset[3] = { 0, 0, 0 };


	/* Gyro Offset:
	 * set to FULLSCALLE_1000
	 * get NUMBER_Offset_VALUES Values
	 * compute average
	 * write to offset registers
	 */

	i2c_buffer[0] = MPU9150_GYRO_FULLSCALE_1000;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_GYRO_CONFIG,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	for (uint8_t i = 0; i < NUMBER_OFFSET_VALUES; i++) {
		HAL_I2C_Mem_Read(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_GYRO_XOUT_H,
		        I2C_MEMADD_SIZE_8BIT, i2c_buffer, 6, MPU9150_I2C_TIMEOUT);

		raw_tmp[0] = (int16_t) (i2c_buffer[1] | (i2c_buffer[0] << 8));
		raw_tmp[1] = (int16_t) (i2c_buffer[3] | (i2c_buffer[2] << 8));
		raw_tmp[2] = (int16_t) (i2c_buffer[5] | (i2c_buffer[4] << 8));

		offset_tmp[0] += raw_tmp[0];
		offset_tmp[1] += raw_tmp[1];
		offset_tmp[2] += raw_tmp[2];
		HAL_Delay(5);
	}

	/* convert values and transfer to MPU Offset Registers*/
	raw_tmp[0] = (int16_t) ((-offset_tmp[0]) / NUMBER_OFFSET_VALUES);
	raw_tmp[1] = (int16_t) ((-offset_tmp[1]) / NUMBER_OFFSET_VALUES);
	raw_tmp[2] = (int16_t) ((-offset_tmp[2]) / NUMBER_OFFSET_VALUES);

	i2c_buffer[0] = (uint8_t) ((raw_tmp[0] >> 8) & 0xFF);
	i2c_buffer[1] = (uint8_t) (raw_tmp[0] & 0xFF);
	i2c_buffer[2] = (uint8_t) ((raw_tmp[1] >> 8) & 0xFF);
	i2c_buffer[3] = (uint8_t) (raw_tmp[1] & 0xFF);
	i2c_buffer[4] = (uint8_t) ((raw_tmp[2] >> 8) & 0xFF);
	i2c_buffer[5] = (uint8_t) (raw_tmp[2] & 0xFF);

	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_XG_OFFS_USRH,
	        I2C_MEMADD_SIZE_8BIT, i2c_buffer, 6, MPU9150_I2C_TIMEOUT);

	/* Accel Offset:
	 * set to FULLSCALLE_8G
	 * Read offset registers
	 * get NUMBER_OFFSET_VALUES Values
	 * compute average
	 * new reg value = ((computed_bias - initial_offset) &~1)
	 *
	 */

	i2c_buffer[0] = MPU9150_ACCEL_FULLSCALE_8g;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_ACCEL_CONFIG,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 1, MPU9150_INIT_TIMEOUT);

	HAL_I2C_Mem_Read(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_XA_OFFS_USRH,
	        I2C_MEMADD_SIZE_8BIT, i2c_buffer, 6, MPU9150_I2C_TIMEOUT);

	accel_offset[0] = (int16_t) (i2c_buffer[1] | (i2c_buffer[0] << 8));
	accel_offset[1] = (int16_t) (i2c_buffer[3] | (i2c_buffer[2] << 8));
	accel_offset[2] = (int16_t) (i2c_buffer[5] | (i2c_buffer[4] << 8));

	for (uint8_t i = 0; i < NUMBER_OFFSET_VALUES; i++) {
		HAL_I2C_Mem_Read(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_ACCEL_XOUT_H,
		        I2C_MEMADD_SIZE_8BIT, i2c_buffer, 6, MPU9150_I2C_TIMEOUT);

		raw_tmp[0] = (int16_t) (i2c_buffer[1] | (i2c_buffer[0] << 8));
		raw_tmp[1] = (int16_t) (i2c_buffer[3] | (i2c_buffer[2] << 8));
		raw_tmp[2] = (int16_t) (i2c_buffer[5] | (i2c_buffer[4] << 8));

		offset_tmp[0] += raw_tmp[0];
		offset_tmp[1] += raw_tmp[1];
		offset_tmp[2] += raw_tmp[2];
		HAL_Delay(5);
	}

	raw_tmp[0] = (int16_t) ((offset_tmp[0]) / NUMBER_OFFSET_VALUES);
	raw_tmp[1] = (int16_t) ((offset_tmp[1]) / NUMBER_OFFSET_VALUES);
	raw_tmp[2] = (int16_t) ((offset_tmp[2]) / NUMBER_OFFSET_VALUES);

	// compensate G on Z axis
	if (raw_tmp[2] > 0 ) {
		raw_tmp[2] = raw_tmp[2] - 4096;
	} else {
		raw_tmp[2] = raw_tmp[2] + 4096;
	}

	accel_offset[0] = accel_offset[0] - (int16_t) (raw_tmp[0] & ~0x0001) ;
	accel_offset[1] = accel_offset[1] - (int16_t) (raw_tmp[1] & ~0x0001) ;
	accel_offset[2] = accel_offset[2] - (int16_t) (raw_tmp[2] & ~0x0001) ;

	i2c_buffer[0] = (uint8_t) ((accel_offset[0] >> 8) & 0xFF);
	i2c_buffer[1] = (uint8_t) (accel_offset[0] & 0xFF);
	i2c_buffer[2] = (uint8_t) ((accel_offset[1] >> 8) & 0xFF);
	i2c_buffer[3] = (uint8_t) (accel_offset[1] & 0xFF);
	i2c_buffer[4] = (uint8_t) ((accel_offset[2] >> 8) & 0xFF);
	i2c_buffer[5] = (uint8_t) (accel_offset[2] & 0xFF);

	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_XA_OFFS_USRH,
	        I2C_MEMADD_SIZE_8BIT, i2c_buffer, 6, MPU9150_I2C_TIMEOUT);
}

void MPU9150::configFullScale(uint8_t gyro_full_scale,
        uint8_t accel_full_scale) {

	i2c_buffer[0] = gyro_full_scale;
	i2c_buffer[1] = accel_full_scale;
	HAL_I2C_Mem_Write(mpu9150_i2c, MPU9150_I2C_ADDRESS, MPU9150_GYRO_CONFIG,
			I2C_MEMADD_SIZE_8BIT, i2c_buffer, 2, MPU9150_INIT_TIMEOUT);

	switch (gyro_full_scale) {
		case MPU9150_GYRO_FULLSCALE_250:
			scaleGyro = MPU9150_GYRO_SCALE_FACTOR_250;
			break;
		case MPU9150_GYRO_FULLSCALE_500:
			scaleGyro = MPU9150_GYRO_SCALE_FACTOR_500;
			break;
		case MPU9150_GYRO_FULLSCALE_1000:
			scaleGyro = MPU9150_GYRO_SCALE_FACTOR_1000;
			break;
		case MPU9150_GYRO_FULLSCALE_2000:
			scaleGyro = MPU9150_GYRO_SCALE_FACTOR_2000;
			break;
		default:
			scaleGyro = MPU9150_GYRO_SCALE_FACTOR_250;

	}
	switch (accel_full_scale) {
		case MPU9150_ACCEL_FULLSCALE_2g:
			scaleAccel = MPU9150_ACCEL_SCALE_FACTOR_2g * G;
			break;
		case MPU9150_ACCEL_FULLSCALE_4g:
			scaleAccel = MPU9150_ACCEL_SCALE_FACTOR_4g * G;
			break;
		case MPU9150_ACCEL_FULLSCALE_8g:
			scaleAccel = MPU9150_ACCEL_SCALE_FACTOR_8g * G;
			break;
		case MPU9150_ACCEL_FULLSCALE_16g:
			scaleAccel = MPU9150_ACCEL_SCALE_FACTOR_16g * G;
			break;
		default:
			scaleAccel = MPU9150_ACCEL_SCALE_FACTOR_4g * G;

	}
}

void MPU9150::startReception() {

	SET_FLAG(taskStatusFlags,MPU9150_FLAG_CONTINUOUS_RECEPTION);
	getAccelGyroMagnetRawData();
}
