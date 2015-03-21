/*
 * hal_callbacks.cpp
 *
 *  Created on: Jan 6, 2015
 *      Author: bohni
 */

#include "main.h"

/**
 * @brief  Period elapsed callback in non blocking mode
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// overrides default callback function
	if (htim->Instance == SCHEDULER_TIMER) {
		scheduler.timerIRQ();
	} else if (htim->Instance == RC_RECEIVER_TIMER) {
		rcReceiver.overrunIRQ();
	}

}
/**
 * @brief  Input Capture callback in non blocking mode
 * @param  htim : TIM IC handle
 * @retval None
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == RC_RECEIVER_TIMER) {
		rcReceiver.captureIRQ();
	}
}

/**
 * @brief  Memory Tx Transfer completed callbacks.
 * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
//	if (hi2c->Instance == ACCELEROMETER_I2C) {
//
//		if (!GET_FLAG(accelerometer.taskStatusFlags,
//				ACCEL_FLAG_TRANSFER_COMPLETE)) {
//			accelerometer.transmissionCompleteCallback();
//		}
//
//	}
}

/**
 * @brief  Memory Rx Transfer completed callbacks.
 * @param  hi2c : Pointer to a I2C_HandleTypeDef structure that contains
 *                the configuration information for the specified I2C.
 * @retval None
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {

//	if (hi2c->Instance == ACCELEROMETER_I2C) {
//
//		if (GET_FLAG(accelerometer.taskStatusFlags,
//				ACCEL_FLAG_TRANSFER_RUNNING)) {
//			accelerometer.receptionCompleteCallback();
//		}
//	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	switch (GPIO_Pin) {
//	case GPIO_PIN_1:
//		/* line 1 */
//		gyro.getGyroData();
//		break;
//	case GPIO_PIN_4:
//		/* line 4 */
//		accelerometer.getAccelerometerData();
//		break;
	default:
		break;
	}
}

/**
 * @brief Tx Transfer completed callbacks
 * @param hspi: SPI handle
 * @retval None
 */
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
//	if (hspi->Instance == GYRO_SPI) {
//
//		if (GET_FLAG(gyro.taskStatusFlags,
//				GYRO_FLAG_TRANSFER_RUNNING)) {
//			gyro.transmissionCompleteCallback();
//		}
//	}
}

/**
 * @brief Rx Transfer completed callbacks
 * @param hspi: SPI handle
 * @retval None
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {

//	if (hspi->Instance == GYRO_SPI) {
//
//		if (GET_FLAG(gyro.taskStatusFlags, GYRO_FLAG_TRANSFER_RUNNING)) {
//			GYRO_CS_HIGH();
//			gyro.receptionCompleteCallback();
//		}
//	}
}

/**
 * @brief Tx and Rx Transfer completed callbacks
 * @param hspi: SPI handle
 * @retval None
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

//	if (hspi->Instance == GYRO_SPI) {
//
//		if (GET_FLAG(gyro.taskStatusFlags, GYRO_FLAG_TRANSFER_RUNNING)) {
//			gyro.receptionCompleteCallback();
//		}
//	}
}

/**
 * @brief SPI error callbacks
 * @param hspi: SPI handle
 * @retval None
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {

}

