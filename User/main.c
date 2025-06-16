/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */

/* Includes ****************************************************************/
#include "BOS.h"

/* Private variables *******************************************************/
//float adcalue1 =0;
//float adcalue2 =0;
//float adcalue3 =0;
//float adcalue4 =0;


uint16_t dutyCycle = 0u;

float LLoadCurrent;


/* Private Function Prototypes *********************************************/




//static float Current(void){
//     float current = 0.0f;
//	 HAL_ADC_Start_IT(&hadc1);
//	 mVolt = (float) ((moving_avg * 0.6103515f) - (4.1943f));
//	 current = (mVolt /CURRENT_SENSE_GAIN);
////	 HAL_ADC_Stop(&hadc1);
//	 return current;
//}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
//{
////   adc_Counter++;
//	moving_avg = movingAverage(adcResult);
//
//
//}
/* Main Function ***********************************************************/
int main(void){

	/* Initialize Module &  BitzOS */





	Module_Init();

	/* Don't place your code here */
	for(;;){
	}
}

/***************************************************************************/
/* User Task */
void UserTask(void *argument){


//	  HAL_TIM_PWM_Start(SWITCH_CONTROL_TIM_HANDLE, SWITCH_CONTROL_TIM_CH);
//	  HAL_ADCEx_Calibration_Start(&hadc1);
//	  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)&adcResult, 1);
//	 ADCSelectPort(P2);
//	 ADCSelectPort(P3);

	/* put your code here, to run repeatedly. */
	while(1){

//		GetLoadCurrent (dutyCycle , &LLoadCurrent);


//		 TIM3->CCR4 = DutyCycle;
//		 myLoadCurrent = Current();

		OutputPWM(50);
		GetLoadCurrent(&LLoadCurrent);
//		HAL_Delay(6000);
//		OutputTurnOff();
//		HAL_Delay(6000);
//
//		HAL_Delay(6000);
//		OutputTurnOff();
//		HAL_Delay(6000);


//	ReadADCChannel(P2,"top",&adcalue1);
//	ReadADCChannel(P2,"bottom",&adcalue2);
//	ReadADCChannel(P3,"top",&adcalue3);
//	ReadADCChannel(P3,"bottom",&adcalue4);

	}
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
