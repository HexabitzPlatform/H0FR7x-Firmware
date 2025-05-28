/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */

/* Includes ****************************************************************/
#include "BOS.h"

/* Private variables *******************************************************/

/* Private Function Prototypes *********************************************/

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
SwapUartPins(&huart6, REVERSED);
SwapUartPins(&huart5, REVERSED);
SwapUartPins(&huart3, REVERSED);
SwapUartPins(&huart2, REVERSED);
SwapUartPins(&huart1, REVERSED);
	/* put your code here, to run repeatedly. */
	while(1){

	}
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
