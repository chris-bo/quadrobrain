/**
 ******************************************************************************
 * File Name          : main.c
 * Date               : 04/01/2015 05:45:24
 * Description        : Main program body
 ******************************************************************************
 *
 * COPYRIGHT(c) 2015 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

#include "main.h"

/* Private variables ---------------------------------------------------------*/

Status status;
Scheduler scheduler(&status, &htim2);
RCreceiver rcReceiver(&status, RC_RECEIVER_DEFAULT_PRIORITY, &htim4);

/* Sensor (Gyro, Accelerometer, Compass) management*/
MPU9150 mpu9150(&status, MPU9150_DEFAULT_PRIORITY, &hi2c1);

/* Sensor data fusion Filters*/
ComplementaryFilter compFilterX(&status, 0, &status.accelY, &status.accelZ,
        &status.rateX, &status.angleX, 0.98f);
ComplementaryFilter compFilterY(&status, 0, &status.accelX, &status.accelZ,
        &status.rateY, &status.angleY, 0.98f);
ComplementaryFilter compFilterNorth(&status, 0, &status.magnetY,
        &status.magnetX, &status.rateZ, &status.angleNorth, 0.98f);

usb_handler usb(&status, USB_DEFAULT_PRIORITY, &hUsbDeviceFS);
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_TIM2_Init();
	MX_TIM4_Init();


	/* USER CODE BEGIN 2 */
	LedBlink led3(&status, 5);
	led3.setFrequency(10);
	led3.setLED(LED3);
	led3.setOffset(50);
	LedBlink led4(&status, 5);
	led4.setFrequency(10);
	led4.setLED(LED4);
	led4.setOffset(25);
	LedBlink led5(&status, 5);
	led5.setFrequency(10);
	led5.setLED(LED5);
	led5.setOffset(75);
	LedBlink led6(&status, 5);
	led6.setFrequency(10);
	led6.setLED(LED6);
	led6.setOffset(200);
	LedBlink led7(&status, 5);
	led7.setFrequency(10);
	led7.setLED(LED7);
	led7.setOffset(100);
	LedBlink led8(&status, 5);
	led8.setFrequency(10);
	led8.setLED(LED8);
	led8.setOffset(175);
	LedBlink led9(&status, 5);
	led9.setFrequency(10);
	led9.setLED(LED9);
	led9.setOffset(125);
	LedBlink led10(&status, 5);
	led10.setFrequency(10);
	led10.setLED(LED10);
	led10.setOffset(150);

	mpu9150.initialize(MPU9150_GYRO_FULL_SCALE,MPU9150_ACCEL_FULL_SCALE);
	rcReceiver.initialize();
	usb.initialize();

	Task* taskarray[] = { &mpu9150, &rcReceiver, &compFilterX, &compFilterY, &compFilterNorth, &usb,
	                      &led3, &led4, &led5, &led6, &led7, &led8, &led9,
	                      &led10 };

	scheduler.start(taskarray, sizeof(taskarray)/ 4);

	mpu9150.startReception();
	/* USER CODE END 2 */

	/* USER CODE BEGIN 3 */
	/* Infinite loop */
	while (1) {

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInit;

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
	        | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
	RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1
	        | RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_USB;
	PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
	PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
	PeriphClkInit.USBClockSelection = RCC_USBPLLCLK_DIV1_5;
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

	__SYSCFG_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
