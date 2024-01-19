/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
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
TIM_HandleTypeDef htim17; /* MOSFET Gate Driver Timer */


extern void MX_TIM17_Init(void);
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

/* TIM17 init function -- MOSFET PWM control */
void MX_TIM17_Init(void)
{

	TIM_OC_InitTypeDef sConfigOC ={0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	htim17.Instance = TIM17;
	htim17.Init.Prescaler =0;
	htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim17.Init.Period = PWM_TIMER_CLOCK_ARR;
	htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim17.Init.RepetitionCounter =0;
	htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	HAL_TIM_Base_Init(&htim17);

	HAL_TIM_PWM_Init(&htim17);

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse =0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	HAL_TIM_PWM_ConfigChannel(&htim17,&sConfigOC,TIM_CHANNEL_1);

	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime =0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter =0;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	HAL_TIMEx_ConfigBreakDeadTime(&htim17,&sBreakDeadTimeConfig);

	HAL_TIM_MspPostInit(&htim17);
}

/*-----------------------------------------------------------*/
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{
	RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(tim_baseHandle->Instance==TIM17)
  {
  /* USER CODE BEGIN TIM17_MspInit 0 */

  /* USER CODE END TIM17_MspInit 0 */
    /* TIM17 clock enable */
    __HAL_RCC_TIM17_CLK_ENABLE();
  /* USER CODE BEGIN TIM17_MspInit 1 */

  /* USER CODE END TIM17_MspInit 1 */
  }

  else if(tim_baseHandle->Instance==TIM15)
  {
  /* USER CODE BEGIN TIM15_MspInit 0 */

  /* USER CODE END TIM15_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM15;
    PeriphClkInit.Tim15ClockSelection = RCC_TIM15CLKSOURCE_PCLK1;
    HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

    /* TIM15 clock enable */
    __HAL_RCC_TIM15_CLK_ENABLE();
  /* USER CODE BEGIN TIM15_MspInit 1 */

  /* USER CODE END TIM15_MspInit 1 */
  }
}

/*-----------------------------------------------------------*/
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

	  GPIO_InitTypeDef GPIO_InitStruct = {0};
	  if(timHandle->Instance==TIM17)
	  {
	  /* USER CODE BEGIN TIM17_MspPostInit 0 */

	  /* USER CODE END TIM17_MspPostInit 0 */

	    __HAL_RCC_GPIOD_CLK_ENABLE();
	    /**TIM17 GPIO Configuration
	    PD1     ------> TIM17_CH1
	    */
	    GPIO_InitStruct.Pin = GPIO_PIN_1;
	    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	    GPIO_InitStruct.Alternate = GPIO_AF2_TIM17;
	    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	  /* USER CODE BEGIN TIM17_MspPostInit 1 */

	  /* USER CODE END TIM17_MspPostInit 1 */
	  }
}

/*-----------------------------------------------------------*/
void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

	if(tim_baseHandle->Instance == TIM15){
		/* USER CODE BEGIN TIM15_MspDeInit 0 */

		/* USER CODE END TIM15_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM15_CLK_DISABLE();
		/* USER CODE BEGIN TIM15_MspDeInit 1 */

		/* USER CODE END TIM15_MspDeInit 1 */
	}

	else if(tim_baseHandle->Instance == TIM17){
		/* USER CODE BEGIN TIM17_MspDeInit 0 */

		/* USER CODE END TIM17_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_TIM17_CLK_DISABLE();
		/* USER CODE BEGIN TIM17_MspDeInit 1 */

		/* USER CODE END TIM17_MspDeInit 1 */
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
