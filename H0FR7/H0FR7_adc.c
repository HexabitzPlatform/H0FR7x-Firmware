/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
 All rights reserved

 File Name     : H0FR7_adc.c
 Description   : Source Code provides configuration of the ADC instances.

 */

/* Includes ------------------------------------------------------------------*/
#include "H0FR7_adc.h"

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void) {
	ADC_ChannelConfTypeDef sConfig = {0};

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)*/
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	HAL_ADC_Init(&hadc1);

	/** Configure Regular Channel*/
	sConfig.Channel = ADC_CHANNAL;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

//void HAL_MspInit(void) {

//	__HAL_RCC_SYSCFG_CLK_ENABLE();
//	__HAL_RCC_PWR_CLK_ENABLE();
//
//	/** Configure the internal voltage reference buffer voltage scale*/
//	HAL_SYSCFG_VREFBUF_VoltageScalingConfig(SYSCFG_VREFBUF_VOLTAGE_SCALE1);
//
//	/** Enable the Internal Voltage Reference buffer*/
//	HAL_SYSCFG_EnableVREFBUF();
//
//	/** Configure the internal voltage reference buffer high impedance mode*/
//	HAL_SYSCFG_VREFBUF_HighImpedanceConfig(SYSCFG_VREFBUF_HIGH_IMPEDANCE_DISABLE);
//
//	/** Disable the internal Pull-Up in Dead Battery pins of UCPD peripheral*/
//	HAL_SYSCFG_StrobeDBattpinsConfig(SYSCFG_CFGR1_UCPD1_STROBE | SYSCFG_CFGR1_UCPD2_STROBE);

//}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(hadc->Instance==ADC1) {

		/* Peripheral clock enable */
		__HAL_RCC_ADC_CLK_ENABLE();
		__HAL_RCC_GPIOA_CLK_ENABLE();

		/**ADC1 GPIO Configuration*/
		GPIO_InitStruct.Pin = ADC_PIN;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(ADC_PORT, &GPIO_InitStruct);
		/* ADC1 interrupt Init */
		HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
	}
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc) {

	if(hadc->Instance==ADC1) {

		/* Peripheral clock disable */
		__HAL_RCC_ADC_CLK_DISABLE();

		/**ADC1 GPIO Configuration*/
		HAL_GPIO_DeInit(ADC_PORT, ADC_PIN);
	}
}




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
