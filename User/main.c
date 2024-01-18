/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

 /* Main function ------------------------------------------------------------*/

int main(void){


	Module_Init();    //Initialize Module &  BitzOS

	//Don't place your code here.
	for(;;){


	}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument){
//	HAL_TIM_Base_Start(&htim17);
//	HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);

//	TIM17->CCR1 = 800;

//	SetSwitchPWM(100);
	OutputOn(1000);
	while(1){



	}
}

/*-----------------------------------------------------------*/
