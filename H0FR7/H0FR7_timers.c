/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H0FR7_timers.c
 Description   : Peripheral timers setup source file.

 Required MCU resources :
 >> Timer 15 for milli-sec delay.
 >> Timer 16 for micro-sec delay.

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/*----------------------------------------------------------------------------*/
/* Configure Timers                                                           */
/*----------------------------------------------------------------------------*/

/* Variables ---------------------------------------------------------*/
//GPIO_InitTypeDef GPIO_InitStruct = {0};
TIM_HandleTypeDef htim16; /* micro-second delay counter */
TIM_HandleTypeDef htim15; /* milli-second delay counter */
TIM_HandleTypeDef htim14; /* MOSFET Gate Driver Timer */


void MX_TIM14_Init(void);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);


/*-----------------------------------------------------------*/
/*  Micro-seconds timebase init function - TIM14 (16-bit)
 */
void TIM_USEC_Init(void){

	  __TIM16_CLK_ENABLE();

	  htim16.Instance = TIM16;
	  htim16.Init.Prescaler = 47;
	  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim16.Init.Period = 0XFFFF;
	  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim16.Init.RepetitionCounter = 0;
	  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  HAL_TIM_Base_Init(&htim16);

	  HAL_TIM_Base_Start(&htim16);
}

/*-----------------------------------------------------------*/

/*  Milli-seconds timebase init function - TIM15 (16-bit)
 */
void TIM_MSEC_Init(void){

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* Peripheral clock enable */
	__TIM15_CLK_ENABLE();

	/* Peripheral configuration */
	htim15.Instance = TIM15;
	htim15.Init.Prescaler = 47999;
	htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim15.Init.Period = 0xFFFF;
	htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim15.Init.RepetitionCounter = 0;
	htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	HAL_TIM_Base_Init(&htim15);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim15,&sMasterConfig);

	HAL_TIM_Base_Start(&htim15);
}

/*-----------------------------------------------------------*/

/* TIM14 init function -- MOSFET PWM control */
void MX_TIM14_Init(void) {

  TIM_OC_InitTypeDef sConfigOC = {0};
  /* Peripheral clock enable */
  	__TIM14_CLK_ENABLE();

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = PWM_TIMER_CLOCK_ARR;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  HAL_TIM_Base_Init(SWITCHING_TIMER_HANDLE);

  HAL_TIM_PWM_Init(SWITCHING_TIMER_HANDLE);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(SWITCHING_TIMER_HANDLE, &sConfigOC,SWITCHING_TIM_CH);

  HAL_TIM_MspPostInit(SWITCHING_TIMER_HANDLE);
}
/*-----------------------------------------------------------*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	if(tim_baseHandle->Instance==TIM14) {
		/* Peripheral clock enable */

		__HAL_RCC_TIM14_CLK_ENABLE();

		/* TIM14 interrupt Init */
		HAL_NVIC_SetPriority(TIM14_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(TIM14_IRQn);
	}
	else if(tim_baseHandle->Instance==TIM15) {

		/** Initializes the peripherals clocks*/
		PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM15;
		PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLKSOURCE_PCLK1;
		HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

		/* TIM15 clock enable */
		__HAL_RCC_TIM15_CLK_ENABLE();

	}
}

/*-----------------------------------------------------------*/
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim) {

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if(htim->Instance==TIM14) {

      __HAL_RCC_GPIOB_CLK_ENABLE();
      /**TIM14 GPIO Configuration*/
      GPIO_InitStruct.Pin = SWITCHING_PIN;
      GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      GPIO_InitStruct.Alternate = GPIO_AF0_TIM14;
      HAL_GPIO_Init(SWITCHING_PORT, &GPIO_InitStruct);
    }
}
/*-----------------------------------------------------------*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{
	if(tim_baseHandle->Instance==TIM14) {
		/* Peripheral clock disable */
		__HAL_RCC_TIM14_CLK_DISABLE();

		/* TIM14 interrupt DeInit */
		HAL_NVIC_DisableIRQ(TIM14_IRQn);
	}
	if(tim_baseHandle->Instance == TIM15) {
		/* USER CODE BEGIN TIM15_MspDeInit 0 */

		/* USER CODE END TIM15_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM15_CLK_DISABLE();
		/* USER CODE BEGIN TIM15_MspDeInit 1 */

		/* USER CODE END TIM15_MspDeInit 1 */
	}
}

/*-----------------------------------------------------------*/
/* --- Load and start micro-second delay counter --- 
 */
void StartMicroDelay(uint16_t Delay){
	uint32_t t0 =0;
	
	portENTER_CRITICAL();
	
	if(Delay){
		t0 =htim16.Instance->CNT;
		
		while(htim16.Instance->CNT - t0 <= Delay){};
	}

	portEXIT_CRITICAL();
}

/*-----------------------------------------------------------*/

/* --- Load and start milli-second delay counter --- 
 */
void StartMilliDelay(uint16_t Delay){
	uint32_t t0 =0;
	
	portENTER_CRITICAL();
	
	if(Delay){
		t0 =htim15.Instance->CNT;
		
		while(htim15.Instance->CNT - t0 <= Delay){};
	}

	portEXIT_CRITICAL();
}
/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
