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

/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"
#include "diag/Trace.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "config.h"
#include "Scheduler.h"
#include "Task.h"
#include "LedBlink.h"
//#include "TestTask.h"
//#include "RCreceiver.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
Status status;
Scheduler scheduler(&status, &htim2);

/* USER CODE END PV */

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
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	LedBlink led3(&status, 5);
	led3.setLED(LED3);
	led3.setFrequency(2);
	led3.setOffset(0);
	LedBlink led4(&status, 5);
	led4.setLED(LED4);
	led4.setFrequency(2);
	led4.setOffset(10);
	LedBlink led5(&status, 5);
	led5.setLED(LED5);
	led5.setFrequency(3);
	led5.setOffset(20);
	LedBlink led6(&status, 5);
	led6.setLED(LED6);
	led6.setFrequency(5);
	led6.setOffset(30);
	LedBlink led7(&status, 5);
	led7.setLED(LED7);
	led7.setFrequency(0.5);
	led7.setOffset(50);
	LedBlink led8(&status, 5);
	led8.setLED(LED8);
	led8.setFrequency(3);
	led8.setOffset(70);
	LedBlink led9(&status, 5);
	led9.setLED(LED9);
	led9.setFrequency(1);
	led9.setOffset(80);
	LedBlink led10(&status, 5);
	led10.setLED(LED10);
	led10.setFrequency(6);
	led10.setOffset(90);

	Task* taskarray[] = { &led3, &led4, &led5, &led6, &led7, &led8, &led9,
			&led10 };

	scheduler.start(taskarray, 8);
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

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	__SYSCFG_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
/**
 * @brief  Period elapsed callback in non blocking mode
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	// overrides default callback function
	if (htim->Instance == SCHEDULER_TIMER) {
		scheduler.timerIRQ();
	}

}
/* USER CODE END 4 */
//
//#ifdef USE_FULL_ASSERT
//
///**
//   * @brief Reports the name of the source file and the source line number
//   * where the assert_param error has occurred.
//   * @param file: pointer to the source file name
//   * @param line: assert_param error line source number
//   * @retval None
//   */
//void assert_failed(uint8_t* file, uint32_t line)
//{
//  /* USER CODE BEGIN 6 */
//  /* User can add his own implementation to report the file name and line number,
//    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
//  /* USER CODE END 6 */
//
//}
//
//#endif
/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
