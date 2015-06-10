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
            &status.pidXOut, &status.pidYOut);

/* EEPROM Configuration Management*/
ConfigReader configReader(&hi2c1);

/* PC communications */
usb_handler usb(&status, USB_DEFAULT_PRIORITY, &hUsbDeviceFS);

/* Sensor data fusion Filters*/
ComplementaryFilter compFilterX(&status, 0, &status.accelY, &status.accelZ,
            &status.rateX, &status.angleX, &status.filterCoefficientXY);
ComplementaryFilter compFilterY(&status, 0, &status.accelX, &status.accelZ,
            &status.rateY, &status.angleY, &status.filterCoefficientXY);
Compass compFilterNorth(&status, 0, &status.magnetY, &status.magnetX, &status.angleY,
            &status.angleX, &status.rateZ, &status.angleNorth,
            &status.filterCoefficientZ);

PIDController pidControllerX(&status, PID_DEFAULT_PRIORITY,
		SCHEDULER_INTERVALL_ms, &status.angleX, 0, &status.rcSignalNick, &status.pidXOut,
            0.15f, false);
PIDController pidControllerY( &status, PID_DEFAULT_PRIORITY,
		SCHEDULER_INTERVALL_ms, &status.angleY, 0, &status.rcSignalRoll, &status.pidYOut,
			0.15f, false);

DiscoveryLEDs leds(&status, LEDs_DEFAULT_PRIORITY);

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* Main functions */
void FlightMode();
void ConfigMode();
void Reset(uint8_t mode);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void) {
    /* MCU Configuration----------------------------------------------------------*/
    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize GPIO and USB*/
    MX_GPIO_Init();
    MX_USB_DEVICE_Init();
    usb.initialize(&configReader);

    /* onboard leds*/
    leds.initialize();
    leds.on(ALL);

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

    SET_FLAG(status.globalFlags, FLIGHT_MODE_FLAG);

    /* init peripherals */
    MX_DMA_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_ADC1_Init();

    /* init Tasks */
    configReader.loadConfiguration(&status);
    mpu9150.initialize(MPU9150_GYRO_FULL_SCALE, MPU9150_ACCEL_FULL_SCALE);
    mpu9150.startReception();
    rcReceiver.initialize();
    ppmgenerator.initialize();
    akku.initialize();
    baro.initialize();

    compFilterX.initialize();
    compFilterY.initialize();
    compFilterNorth.initialize();

    /* Initialize PID for X and Y axis */
    pidControllerX.initialize(&status.pXY, &status.iXY, &status.dXY);
    pidControllerY.initialize(&status.pXY, &status.iXY, &status.dXY);

    /* blinking flight led, to indicate running cpu */
    leds.setFrequency(FLIGHT_LED,1);

    /* create tasks and start scheduler */
    Task* taskarray[] = { &mpu9150, &rcReceiver, &ppmgenerator, &compFilterX,
                          &compFilterY, &compFilterNorth, &pidControllerX, &usb,
                          &akku, &baro, &leds};

    scheduler.start(taskarray, sizeof(taskarray) / 4);

    while (1) {
        if ((usb.usb_mode_request == USB_MODE_CONFIG)
                    || (usb.usb_mode_request == USB_MODE_SAVE_CONFIG)) {
            ConfigMode();
        } else if (usb.usb_mode_request == USB_MODE_RESET) {
            Reset(RESET_TO_FLIGHT);
        }
    }
}

void Reset(uint8_t mode) {
     /* No need to deinit Hardware reinitializing resets peripherals*/

    if (mode == RESET_TO_CONFIG) {
        usb.usb_mode_request = USB_MODE_CONFIG;
        ConfigMode();
    } else {
        usb.usb_mode_request = USB_MODE_NORMAL;
        /* kill processes*/
        scheduler.kill();

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
    scheduler.pause();

    SET_FLAG(status.globalFlags, CONFIG_MODE_FLAG);
    RESET_FLAG(status.globalFlags, FLIGHT_MODE_FLAG);

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
     * and usb Task needed
     * */
    leds.setFrequency(CONFIG_LED,1);
    usb.initialize(&configReader);

    Task* taskarray[] = { &usb, &leds };
    scheduler.start(taskarray, 2);

    while (1) {
        if ((usb.usb_mode_request == USB_MODE_LEAVE_CONFIG)
                    || (usb.usb_mode_request == USB_MODE_SAVE_CONFIG)) {
            /* config finished */
            configReader.saveConfiguration(&status);

            /* Reset System */
            Reset(RESET_TO_FLIGHT);
        } else if (usb.usb_mode_request == USB_MODE_RESET) {
            Reset(RESET_TO_CONFIG);
        }
    }
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

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB | RCC_PERIPHCLK_I2C1
                | RCC_PERIPHCLK_I2C2 | RCC_PERIPHCLK_ADC12;
    PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
    PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
    PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
    PeriphClkInit.USBClockSelection = RCC_USBPLLCLK_DIV1_5;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    HAL_RCC_EnableCSS();

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
