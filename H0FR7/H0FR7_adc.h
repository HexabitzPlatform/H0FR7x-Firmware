/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name     : H0FR7_adc.h
 Description   : Header file contains Peripheral adc setup.

 */

/* Define to prevent recursive inclusion ***********************************/
#ifndef H0BR4_adc_H
#define H0BR4_adc_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ****************************************************************/
#include "stm32g0xx_hal.h"

/* Exported Variables ******************************************************/

extern ADC_HandleTypeDef hadc1;

/* External function *******************************************************/
void MX_ADC1_Init(void);


#ifdef __cplusplus
}
#endif

#endif /* __ADC_H__ */

 /***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
