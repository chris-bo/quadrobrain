/**
  ******************************************************************************
  * File Name          : TIM.c
  * Date               : 21/03/2015 17:56:44
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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
#include "tim.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim17;

/* TIM2 init function */
/* scheduler timer */
void MX_TIM2_Init(void) {
	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim2.Instance = TIM2;
	htim2.Init.Prescaler = SCHEDULER_TIMER_PRESCALER;
	htim2.Init.CounterMode = TIM_COUNTERMODE_DOWN;
	htim2.Init.Period = SCHEDULER_TIMER_PERIOD;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&htim2);

//  sClockSourceConfig.Clock
//  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
//  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = PPM_TIMER_PRESCALER;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = PPM_TIMER_PERIOD;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  HAL_TIM_PWM_Init(&htim3);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);

  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

}

/* TIM4 init function */
/* rc receiver timer*/
void MX_TIM4_Init(void) {
	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_IC_InitTypeDef sConfigIC;

	/* Time base configuration
	 * Prescaler sets timer clock to 1 MHz -> timings in us
	 * */

	htim4.Instance = TIM4;
	htim4.Init.Prescaler = RC_RECEIVER_TIMER_PRESCALER;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = RC_RECEIVER_OverrunTime_us;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_IC_Init(&htim4);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig);

	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_3);

}


/* TIM15 init function */
void MX_TIM15_Init(void) {
TIM_MasterConfigTypeDef sMasterConfig;
TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
TIM_OC_InitTypeDef sConfigOC;

htim15.Instance = TIM15;
htim15.Init.Prescaler = BUZZER_PRESCALER;
htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
htim15.Init.Period = 2200;
htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
htim15.Init.RepetitionCounter = 0;
HAL_TIM_PWM_Init(&htim15);

sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig);

sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
sBreakDeadTimeConfig.DeadTime = 0;
sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
sBreakDeadTimeConfig.BreakFilter = 0;
sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig);

sConfigOC.OCMode = TIM_OCMODE_PWM1;
sConfigOC.Pulse = 0;
sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1);

HAL_TIM_PWM_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_2);

}

/* TIM17 init function */
void MX_TIM17_Init(void)
{
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim17.Instance = TIM17;
  htim17.Init.Prescaler = BUZZER_PRESCALER;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 2200;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim17);

  HAL_TIM_PWM_Init(&htim17);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim17, &sBreakDeadTimeConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim17, &sConfigOC, TIM_CHANNEL_1);

}


void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* Peripheral clock enable */
    __TIM2_CLK_ENABLE();

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspInit 0 */

  /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __TIM3_CLK_ENABLE();
  
    /**TIM3 GPIO Configuration    
    PC6     ------> TIM3_CH1
    PC7     ------> TIM3_CH2
    PC8     ------> TIM3_CH3
    PC9     ------> TIM3_CH4 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM3_MspInit 1 */

  /* USER CODE END TIM3_MspInit 1 */
  }   else if(htim_base->Instance==TIM17)
  {
  /* USER CODE BEGIN TIM17_MspInit 0 */

  /* USER CODE END TIM17_MspInit 0 */
    /* Peripheral clock enable */
    __TIM17_CLK_ENABLE();

    /**TIM17 GPIO Configuration
    PB9     ------> TIM17_CH1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM17;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM17_MspInit 1 */

  /* USER CODE END TIM17_MspInit 1 */
  }
}


void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* htim_pwm) {

GPIO_InitTypeDef GPIO_InitStruct;
if (htim_pwm->Instance == TIM3) {
    /* USER CODE BEGIN TIM3_MspInit 0 */

    /* USER CODE END TIM3_MspInit 0 */
    /* Peripheral clock enable */
    __TIM3_CLK_ENABLE();

    /**TIM3 GPIO Configuration
     PC6     ------> TIM3_CH1
     PC7     ------> TIM3_CH2
     PC8     ------> TIM3_CH3
     PC9     ------> TIM3_CH4
     */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM3_MspInit 1 */

    /* USER CODE END TIM3_MspInit 1 */
} else if (htim_pwm->Instance == TIM15) {
    /* USER CODE BEGIN TIM15_MspInit 0 */

    /* USER CODE END TIM15_MspInit 0 */
    /* Peripheral clock enable */
    __TIM15_CLK_ENABLE();

    /**TIM15 GPIO Configuration
     PF9     ------> TIM15_CH1
     PF10    ------> TIM15_CH2
     */
    GPIO_InitStruct.Pin = GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM15;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /* USER CODE BEGIN TIM15_MspInit 1 */

    /* USER CODE END TIM15_MspInit 1 */
}  else if(htim_pwm->Instance==TIM17)
{
/* USER CODE BEGIN TIM17_MspInit 0 */

/* USER CODE END TIM17_MspInit 0 */
  /* Peripheral clock enable */
  __TIM17_CLK_ENABLE();

  /**TIM17 GPIO Configuration
  PB9     ------> TIM17_CH1
  */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM17;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN TIM17_MspInit 1 */

/* USER CODE END TIM17_MspInit 1 */
}
}


void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* htim_ic)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim_ic->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspInit 0 */

  /* USER CODE END TIM4_MspInit 0 */
    /* Peripheral clock enable */
    __TIM4_CLK_ENABLE();
  
    /**TIM4 GPIO Configuration    
    PB8     ------> TIM4_CH3 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Peripheral interrupt init*/
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 1);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USER CODE BEGIN TIM4_MspInit 1 */

  /* USER CODE END TIM4_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base)
{

  if(htim_base->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM2_CLK_DISABLE();

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM2_IRQn);

  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(htim_base->Instance==TIM3)
  {
  /* USER CODE BEGIN TIM3_MspDeInit 0 */

  /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM3_CLK_DISABLE();
  
    /**TIM3 GPIO Configuration    
    PC6     ------> TIM3_CH1
    PC7     ------> TIM3_CH2
    PC8     ------> TIM3_CH3
    PC9     ------> TIM3_CH4 
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9);

  /* USER CODE BEGIN TIM3_MspDeInit 1 */

  /* USER CODE END TIM3_MspDeInit 1 */
  }
}


void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* htim_pwm) {

if (htim_pwm->Instance == TIM3) {
    /* USER CODE BEGIN TIM3_MspDeInit 0 */

    /* USER CODE END TIM3_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM3_CLK_DISABLE();

    /**TIM3 GPIO Configuration
     PC6     ------> TIM3_CH1
     PC7     ------> TIM3_CH2
     PC8     ------> TIM3_CH3
     PC9     ------> TIM3_CH4
     */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9);

    /* USER CODE BEGIN TIM3_MspDeInit 1 */

    /* USER CODE END TIM3_MspDeInit 1 */
} else if (htim_pwm->Instance == TIM15) {
    /* USER CODE BEGIN TIM15_MspDeInit 0 */

    /* USER CODE END TIM15_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM15_CLK_DISABLE();

    /**TIM15 GPIO Configuration
     PF9     ------> TIM15_CH1
     PF10     ------> TIM15_CH2
     */
    HAL_GPIO_DeInit(GPIOF, GPIO_PIN_9 | GPIO_PIN_10);

    /* USER CODE BEGIN TIM15_MspDeInit 1 */

    /* USER CODE END TIM15_MspDeInit 1 */
}
}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* htim_ic)
{

  if(htim_ic->Instance==TIM4)
  {
  /* USER CODE BEGIN TIM4_MspDeInit 0 */

  /* USER CODE END TIM4_MspDeInit 0 */
    /* Peripheral clock disable */
    __TIM4_CLK_DISABLE();
  
    /**TIM4 GPIO Configuration    
    PB8     ------> TIM4_CH3 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(TIM4_IRQn);

  /* USER CODE BEGIN TIM4_MspDeInit 1 */

  /* USER CODE END TIM4_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/