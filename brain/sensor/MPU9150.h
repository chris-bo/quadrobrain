/*
 * MPU9150.h
 *
 *  Created on: Mar 21, 2015
 *      Author: bohni
 */

#ifndef MPU9150_H_
#define MPU9150_H_

#include <core/Task.h>
#include "stm32f3xx_hal.h"
#include "i2c.h"

/***********************************
 *
 * PINS
 *
 * I2C1
 * PB6 -> SCL
 * PB7 -> SDA
 * PE4 -> INT
 *
 */
/********************************************************************/
/* MPU9150 Register Map */

/* Bias Registers
 * Gyro bias inputs are LSB in +-1000dps format.
 * */

#define MPU9150_XG_OFFS_USRH		0x13
#define MPU9150_XG_OFFS_USRL		0x14
#define MPU9150_YG_OFFS_USRH		0x15
#define MPU9150_YG_OFFS_USRL		0x16
#define MPU9150_ZG_OFFS_USRH		0x17
#define MPU9150_ZG_OFFS_USRL		0x18

#define MPU9150_XA_OFFS_USRH		0x06
#define MPU9150_XA_OFFS_USRL		0x07
#define MPU9150_YA_OFFS_USRH		0x08
#define MPU9150_YA_OFFS_USRL		0x09
#define MPU9150_ZA_OFFS_USRH		0x0A
#define MPU9150_ZA_OFFS_USRL		0x0B

#define MPU9150_SELF_TEST_X        0x0D   // R/W
#define MPU9150_SELF_TEST_Y        0x0E   // R/W
#define MPU9150_SELF_TEST_Z        0x0F   // R/W
#define MPU9150_SELF_TEST_A        0x10   // R/W
#define MPU9150_SMPLRT_DIV         0x19   // R/W
#define MPU9150_CONFIG             0x1A   // R/W
#define MPU9150_GYRO_CONFIG        0x1B   // R/W
#define MPU9150_ACCEL_CONFIG       0x1C   // R/W
#define MPU9150_FF_THR             0x1D   // R/W
#define MPU9150_FF_DUR             0x1E   // R/W
#define MPU9150_MOT_THR            0x1F   // R/W
#define MPU9150_MOT_DUR            0x20   // R/W
#define MPU9150_ZRMOT_THR          0x21   // R/W
#define MPU9150_ZRMOT_DUR          0x22   // R/W
#define MPU9150_FIFO_EN            0x23   // R/W
#define MPU9150_I2C_MST_CTRL       0x24   // R/W
#define MPU9150_I2C_SLV0_ADDR      0x25   // R/W
#define MPU9150_I2C_SLV0_REG       0x26   // R/W
#define MPU9150_I2C_SLV0_CTRL      0x27   // R/W
#define MPU9150_I2C_SLV1_ADDR      0x28   // R/W
#define MPU9150_I2C_SLV1_REG       0x29   // R/W
#define MPU9150_I2C_SLV1_CTRL      0x2A   // R/W
#define MPU9150_I2C_SLV2_ADDR      0x2B   // R/W
#define MPU9150_I2C_SLV2_REG       0x2C   // R/W
#define MPU9150_I2C_SLV2_CTRL      0x2D   // R/W
#define MPU9150_I2C_SLV3_ADDR      0x2E   // R/W
#define MPU9150_I2C_SLV3_REG       0x2F   // R/W
#define MPU9150_I2C_SLV3_CTRL      0x30   // R/W
#define MPU9150_I2C_SLV4_ADDR      0x31   // R/W
#define MPU9150_I2C_SLV4_REG       0x32   // R/W
#define MPU9150_I2C_SLV4_DO        0x33   // R/W
#define MPU9150_I2C_SLV4_CTRL      0x34   // R/W
#define MPU9150_I2C_SLV4_DI        0x35   // R
#define MPU9150_I2C_MST_STATUS     0x36   // R
#define MPU9150_INT_PIN_CFG        0x37   // R/W
#define MPU9150_INT_ENABLE         0x38   // R/W
#define MPU9150_INT_STATUS         0x3A   // R
#define MPU9150_ACCEL_XOUT_H       0x3B   // R
#define MPU9150_ACCEL_XOUT_L       0x3C   // R
#define MPU9150_ACCEL_YOUT_H       0x3D   // R
#define MPU9150_ACCEL_YOUT_L       0x3E   // R
#define MPU9150_ACCEL_ZOUT_H       0x3F   // R
#define MPU9150_ACCEL_ZOUT_L       0x40   // R
#define MPU9150_TEMP_OUT_H         0x41   // R
#define MPU9150_TEMP_OUT_L         0x42   // R
#define MPU9150_GYRO_XOUT_H        0x43   // R
#define MPU9150_GYRO_XOUT_L        0x44   // R
#define MPU9150_GYRO_YOUT_H        0x45   // R
#define MPU9150_GYRO_YOUT_L        0x46   // R
#define MPU9150_GYRO_ZOUT_H        0x47   // R
#define MPU9150_GYRO_ZOUT_L        0x48   // R
#define MPU9150_EXT_SENS_DATA_00   0x49   // R
#define MPU9150_EXT_SENS_DATA_01   0x4A   // R
#define MPU9150_EXT_SENS_DATA_02   0x4B   // R
#define MPU9150_EXT_SENS_DATA_03   0x4C   // R
#define MPU9150_EXT_SENS_DATA_04   0x4D   // R
#define MPU9150_EXT_SENS_DATA_05   0x4E   // R
#define MPU9150_EXT_SENS_DATA_06   0x4F   // R
#define MPU9150_EXT_SENS_DATA_07   0x50   // R
#define MPU9150_EXT_SENS_DATA_08   0x51   // R
#define MPU9150_EXT_SENS_DATA_09   0x52   // R
#define MPU9150_EXT_SENS_DATA_10   0x53   // R
#define MPU9150_EXT_SENS_DATA_11   0x54   // R
#define MPU9150_EXT_SENS_DATA_12   0x55   // R
#define MPU9150_EXT_SENS_DATA_13   0x56   // R
#define MPU9150_EXT_SENS_DATA_14   0x57   // R
#define MPU9150_EXT_SENS_DATA_15   0x58   // R
#define MPU9150_EXT_SENS_DATA_16   0x59   // R
#define MPU9150_EXT_SENS_DATA_17   0x5A   // R
#define MPU9150_EXT_SENS_DATA_18   0x5B   // R
#define MPU9150_EXT_SENS_DATA_19   0x5C   // R
#define MPU9150_EXT_SENS_DATA_20   0x5D   // R
#define MPU9150_EXT_SENS_DATA_21   0x5E   // R
#define MPU9150_EXT_SENS_DATA_22   0x5F   // R
#define MPU9150_EXT_SENS_DATA_23   0x60   // R
#define MPU9150_MOT_DETECT_STATUS  0x61   // R
#define MPU9150_I2C_SLV0_DO        0x63   // R/W
#define MPU9150_I2C_SLV1_DO        0x64   // R/W
#define MPU9150_I2C_SLV2_DO        0x65   // R/W
#define MPU9150_I2C_SLV3_DO        0x66   // R/W
#define MPU9150_I2C_MST_DELAY_CTRL 0x67   // R/W
#define MPU9150_SIGNAL_PATH_RESET  0x68   // R/W
#define MPU9150_MOT_DETECT_CTRL    0x69   // R/W
#define MPU9150_USER_CTRL          0x6A   // R/W
#define MPU9150_PWR_MGMT_1         0x6B   // R/W
#define MPU9150_PWR_MGMT_2         0x6C   // R/W
#define MPU9150_FIFO_COUNTH        0x72   // R/W
#define MPU9150_FIFO_COUNTL        0x73   // R/W
#define MPU9150_FIFO_R_W           0x74   // R/W
#define MPU9150_WHO_AM_I           0x75   // R

//MPU9150 Compass

#define MPU9150_CMPS_XOUT_L        0x4A   // R
#define MPU9150_CMPS_XOUT_H        0x4B   // R
#define MPU9150_CMPS_YOUT_L        0x4C   // R
#define MPU9150_CMPS_YOUT_H        0x4D   // R
#define MPU9150_CMPS_ZOUT_L        0x4E   // R
#define MPU9150_CMPS_ZOUT_H        0x4F   // R

// I2C address 0x69 could be 0x68 wiring.
//ADO with 4.7k Pulldown on breakoutboard
#define MPU9150_I2C_ADDRESS			(0x68<<1)

/* Full Scale */
/* Register Settings */
#define MPU9150_GYRO_FULLSCALE_250			0x00
#define MPU9150_GYRO_FULLSCALE_500			0x08
#define MPU9150_GYRO_FULLSCALE_1000			0x10
#define MPU9150_GYRO_FULLSCALE_2000			0x18

#define MPU9150_ACCEL_FULLSCALE_2g			0x00
#define MPU9150_ACCEL_FULLSCALE_4g			0x08
#define MPU9150_ACCEL_FULLSCALE_8g			0x10
#define MPU9150_ACCEL_FULLSCALE_16g			0x18

/* End MPU9150 Register Map*/
/********************************************************************/
/* AK8975C Register Map*/
#define AK8975C_WIA			0x00	// R
#define AK8975C_INFO		0x01	// R
#define AK8975C_ST1			0x02	// R
#define AK8975C_HXL			0x03	// R
#define AK8975C_HXH			0x04	// R
#define AK8975C_HYL			0x05	// R
#define AK8975C_HYH			0x06	// R
#define AK8975C_HZL			0x07	// R
#define AK8975C_HZH			0x08	// R
#define AK8975C_ST2			0x09	// R
#define AK8975C_CNTL		0x0A	// R/W
#define AK8975C_ASTC		0x0C	// R/W
#define AK8975C_ASAX		0x10	// R
#define AK8975C_ASAY		0x11	// R
#define AK8975C_ASAZ		0x12	// R

#define AK8975C_I2C_ADDRESS (0x0C)
/* End AK8975C Register Map*/
/********************************************************************/

#define MPU9150_INIT_TIMEOUT							0xFFFFF
#define MPU9150_I2C_TIMEOUT								0xFFFFF

/* Flags */
#define MPU9150_FLAG_CONTINUOUS_RECEPTION				0x0001
#define MPU9150_FLAG_AK8975C_AVAILABLE					0x0002
#define MPU9150_FLAG_I2C_BUSY							0x0003
#define MPU9150_FLAG_ERROR								0x8000

/* End Flags */
/********************************************************************/
/* Settings */
#define NUMBER_OFFSET_VALUES					500

#define I_AM_MPU9150						0x68
#define I_AM_AK8975C						0x48

/* divides reading rate of i2c slaves
 * delay must be enabled for each slave
 *
 * rate = mpu_sample_rate / (1 + delay)
 *
 */
#define MPU9150_EXT_SENS_I2C_DELAY			0x05

/* sets MPU Sample Rate*
 * rate = 1kHz / (1 + delay)
 */
#define MPU9150_SAMPLE_RATE					0x01
/* sets internal Low Pass Filter */
#define MPU9150_DLPF_SETTING				0x02

/* sets initial fullscale setting */
#define MPU9150_GYRO_FULL_SCALE				MPU9150_GYRO_FULLSCALE_1000
#define MPU9150_ACCEL_FULL_SCALE			MPU9150_ACCEL_FULLSCALE_8g

/* Scale factors 
 * real_value = register_value * scale
 */

/* scale to deg/sec*/
#define MPU9150_GYRO_SCALE_FACTOR_250			0.007633588f
#define MPU9150_GYRO_SCALE_FACTOR_500			0.015267176f
#define MPU9150_GYRO_SCALE_FACTOR_1000			0.030487805f
#define MPU9150_GYRO_SCALE_FACTOR_2000			0.06097561f

/* scale to g */
#define MPU9150_ACCEL_SCALE_FACTOR_2g			0.000061035f			
#define MPU9150_ACCEL_SCALE_FACTOR_4g			0.00012207f
#define MPU9150_ACCEL_SCALE_FACTOR_8g			0.000244141f
#define MPU9150_ACCEL_SCALE_FACTOR_16g			0.000488281f

#define MPU9150_TEMPERATURE_SCALE_FACTOR		0.002941176f		// T = register * scale
#define MPU9150_TEMPERATURE_OFFSET				+521			// 35 deg C

/* End Settings*/
/********************************************************************/

class MPU9150: public Task {
public:
    MPU9150(Status* statusPtr, uint8_t defaultPrio, I2C_HandleTypeDef* i2c);
    virtual ~MPU9150();

    void update();
    void initialize(uint8_t gyro_full_scale, uint8_t accel_full_scale);
    void kill();
    void configFullScale(uint8_t gyro_full_scale, uint8_t accel_full_scale);

    void startReception();
    void stopReception();

    void receptionCompleteCallback();
    void getRawData();

private:

    I2C_HandleTypeDef* mpu9150_i2c;
    /* raw data : x,y,z */
    int16_t rawAccelData[3];
    int16_t rawGyroData[3];
    int16_t rawMagnetData[3];
    int16_t rawTempData;
    uint8_t MagnetScaleRegister[3];

    bool continousReception :1;
    bool mpuError :1;
    bool AK8975Cavailable :1;

    /* dummy padding variable */
    int pad :5;

    float scaleAccel;
    float scaleGyro;
    float scaleMagnet[3];
    float biasAccel[3];

    /* general + mpu9150 functions*/
    void getBias();
    void configOffsetRegisters();
    void scaleRawData();
    uint8_t getMPU9150Identification();

    /* Functions for magnetometer control */
    void enableMagnetData();
    void disableMagnetData();
    void getMagnetScale();

};

#endif /* MPU9150_H_ */
