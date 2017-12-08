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
Scheduler scheduler(&status, &htim2, &leds);

/* Input capture timer */
RCreceiver rcReceiver(&status, RC_RECEIVER_DEFAULT_PRIORITY, &htim4);

/* Sensor (Gyro, Accelerometer, Compass) management*/
MPU9150 mpu9150(&status, MPU9150_DEFAULT_PRIORITY, &hi2c1);

/* Pressure Sensor*/
BMP180 baro(&status, BMP180_DEFAULT_PRIORITY, &hi2c2);

/* Monitor Akku Voltage*/
AkkuMonitor akku(&status, AKKUMONITOR_DEFAULT_PRIORITY, &hadc1);

/* Motor Control */
PPMGenerator ppmgenerator(&status, PPMGENERATOR_DEFAULT_PRIORITY, &htim3,
        &status.motorSetpoint.x, &status.motorSetpoint.y,
        &status.motorSetpoint.z, &status.rcSignalThrottle);

/* EEPROM Configuration Management*/
ConfigReader configReader(&hi2c1);

/* PC communications */
USBHandler usb(&status, USB_DEFAULT_PRIORITY, &leds, USB_TRANSMIT_LED,
        &hUsbDeviceFS);

UARTBluetoothHandler uartBT(&status, UART_DEFAULT_PRIORITY, &leds,
UART_TRANSMIT_LED, &huart2);

QCcoms usbCom(&status, USB_DEFAULT_PRIORITY, &configReader, &usb, &flightLEDs);

QCcoms uartCom(&status, USB_DEFAULT_PRIORITY, &configReader, &uartBT,
        &flightLEDs);
/* GPS Receiver */
GPS gpsReceiver(&status, GPS_DEFAULT_PRIORITY, &huart1);

/* Sensor data fusion Filters*/
/* IMU to get orientation */
IMU imu(&status, IMU_DEFAULT_PRIRORITY);

/* Altimeter */
Altimeter altimeter(&status, ALTIMETER_DEFAULT_PRIRORITY);

/* PIDs low level */
PIDController pidAngleX(&status, PID_DEFAULT_PRIORITY,
        (float) SCHEDULER_INTERVALL_ms / 1000.0f, &status.angle.x,
        &status.rate.x, &status.angleSetpoint.x, &status.motorSetpoint.x,
        PID_XY_CONTROL_VALUE_GAIN,
        PID_LIMIT, PID_SUM_LIMIT, true);
PIDController pidAngleY(&status, PID_DEFAULT_PRIORITY,
        (float) SCHEDULER_INTERVALL_ms / 1000.0f, &status.angle.y,
        &status.rate.y, &status.angleSetpoint.y, &status.motorSetpoint.y,
        PID_XY_CONTROL_VALUE_GAIN,
        PID_LIMIT, PID_SUM_LIMIT, true);
PIDController pidRateZ(&status, PID_DEFAULT_PRIORITY,
        (float) SCHEDULER_INTERVALL_ms / 1000.0f, &status.rate.z, 0,
        &status.rcSignalYaw, &status.motorSetpoint.z, PID_Z_CONTROL_VALUE_GAIN,
        PID_LIMIT, PID_SUM_LIMIT, false);

/* Motion Control
 * includes PIDs for velocity and acceleration
 *
 * removed
 */

/* Feedback Tasks */
DiscoveryLEDs leds(&status, LEDs_DEFAULT_PRIORITY);
Buzzer beep1(&status, BUZZER_DEFAULT_PRIORITY, &htim15, &status.buzzerQueue1);
//Buzzer beep2(&status, BUZZER_DEFAULT_PRIORITY, &htim17, &status.buzzerQueue2);
FlightLED flightLEDs(&status, LEDs_DEFAULT_PRIORITY, &hspi2);
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* Main functions */
void FlightMode();
void ConfigMode();
void SoftwareReset();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {
    /* MCU Configuration----------------------------------------------------------*/
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize GPIO and USB and UART for bluetooth*/
    MX_GPIO_Init();
    MX_USB_DEVICE_Init();
    MX_USART2_UART_Init();

    usb.initialize();

    /* onboard leds*/
    leds.initialize();
    leds.on(ALL);

    /* Buzzer*/
    MX_TIM15_Init();
//	MX_TIM17_Init();
    beep1.initialize();
//	beep2.initialize();

    /* Short Delay to check all leds */
    HAL_Delay(10);

    /* Call Normal Flight Mode*/
    FlightMode();

    /* Infinite loop */
    while (1)
        ;

}

void FlightMode() {

    leds.off(ALL);
    leds.on(POWER_LED);
    leds.on(FLIGHT_LED);

    status.globalFlags.configMode = false;
    status.globalFlags.flightMode = true;

    /* init peripherals */
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_ADC1_Init();
    MX_USART1_UART_Init();
    MX_SPI2_Init();

    /* init Tasks */
    usbCom.initialize();
    uartCom.initialize();

    configReader.initialize(&status);

    mpu9150.initialize(MPU9150_GYRO_FULL_SCALE, MPU9150_ACCEL_FULL_SCALE);
    mpu9150.startReception();

    rcReceiver.initialize();
    ppmgenerator.initialize();
    akku.initialize();
    baro.initialize();
    gpsReceiver.initialize();

    imu.initialize();
    altimeter.initialize();

    /* Initialize PID for X Y and Z axis */
    pidAngleX.initialize(&status.pidSettingsAngleXY);
    pidAngleY.initialize(&status.pidSettingsAngleXY);
    pidRateZ.initialize(&status.pidSettingsRotationZ);

    /* blinking flight led, to indicate running cpu */
    leds.setFrequency(FLIGHT_LED, 1);

    /* activate flight LEDs*/
    flightLEDs.initialize();

    /* create tasks and start scheduler */
    Task* taskarray[] = { &mpu9150, &rcReceiver, &ppmgenerator, &imu,
            &altimeter, &pidAngleX, &pidAngleY, &pidRateZ, &gpsReceiver,
            &usbCom, &uartCom, &akku, &baro, &leds, &beep1, &flightLEDs };
    scheduler.start(taskarray, sizeof(taskarray) / 4);

    /* don't stop beliieeeving */
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_A4, 125);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_Gp4, 250);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_Gp4, 375);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_PAUSE, 125);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_Fp4, 125);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_A4, 125);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_Gp4, 250);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_Gp4, 375);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_PAUSE, 250);
//    HAL_Delay(2000);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_PAUSE, 500);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_A4, 125);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_Gp4, 125);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_A4, 125);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_B4, 125);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_Cp5, 250);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_B4, 63);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_Cp5, 62);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_Gp4, 250);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_Fp4, 125);
//    status.addToneToQueue(&status.buzzerQueue1, BUZZER_E4, 250);
//    HAL_Delay(2000);
    while (1) {
        if (status.globalFlags.resetRequested) {
            SoftwareReset();
        }
    }
}

void SoftwareReset() {
    /* No need to deinit Hardware reinitializing resets peripherals*/
    /* kill processes*/
    scheduler.kill();

    status.globalFlags.resetRequested = false;

    if (status.globalFlags.resetToConfig) {
        /* clear reset request */
        status.globalFlags.resetToConfig = false;
        /* goto config mode */
        ConfigMode();
    } else {
        /* clear reset request */
        status.globalFlags.resetToFlight = false;
        /* recall flight mode */
        FlightMode();
    }
}

void ConfigMode() {
    /* Enter Config Mode:
     *
     * Stop Sensor Functions
     * Disable Motors
     *
     * Restart scheduler only for usb task
     *
     */
    scheduler.reset();

    status.globalFlags.configMode = true;
    status.globalFlags.flightMode = false;

    /* kill all sensors
     * and switch off engine
     */
    mpu9150.kill();
    baro.kill();
    rcReceiver.kill();
    ppmgenerator.kill();

    leds.off(ALL);
    leds.on(POWER_LED);
    /* only blinking led to indicate running cpu
     * and comminication tasks needed
     * */
    leds.setFrequency(CONFIG_LED, 1);
    usbCom.initialize();
    uartCom.initialize();

    Task* tasks_config[] = { &usbCom, &uartCom, &leds, &beep1 };
    scheduler.start(tasks_config, sizeof(tasks_config) / 4);

    while (1) {
        if (status.globalFlags.resetRequested) {
            /* config finished */
            configReader.saveConfiguration(&status);
            SoftwareReset();
        }
    }
}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI
            | RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = 16;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks 
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB
            | RCC_PERIPHCLK_USART1 | RCC_PERIPHCLK_USART2 | RCC_PERIPHCLK_I2C1
            | RCC_PERIPHCLK_I2C2 | RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
    PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Enables the Clock Security System 
     */
    HAL_RCC_EnableCSS();

    /**Configure the Systick interrupt time 
     */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

    /**Configure the Systick 
     */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line) {
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while (1) {
    }
    /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
