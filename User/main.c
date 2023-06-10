/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name     : main.c
 Description   : Main program body.
 */
/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include <stdio.h>
#include <string.h>

/* Private variables ---------------------------------------------------------*/

//volatile uint8_t Temp;
extern uint8_t setPointTempature;
extern uint8_t flagParamterTemp;


uint8_t flagCalibration =0;
int16_t diffTemp;

float adcValue;
float voltage_supply = 3300;
float voltageDividerR1 = 5000;
uint16_t currentTemperature=0;
uint16_t resistanceNTC=0;

// Constants for PID control
#define KP 5.86   // Proportional gain
#define KI 0.02   // Integral gain
#define KD 0.001  // Derivative gain

// Variables for PID control

float outputPWM = 0.0;      // PWM output
double integral = 0.0;    // Integral term
double previousError = 0.0;
float outpwm;


// Lookup table for temperature and corresponding resistance
const uint32_t temperatureValues[] = {15,17,20,23,25,27,30,33,35,40,42,44,46,48,50,51,52,53,54,55,56,57
  ,58,61,65,70,75,77,81,84,86,88,94,95,97,100,105,110,115,120,125,130,135,140,145,150,165
  ,170,175,180,185,190,195,200,205,210,215,220,223,225,233,235,237,240,244,245,248,249,250}; // Corresponding temperature values


const uint32_t resistanceValues[] = {92000,91000,75200,63100,59000,55000,53210,52000,50300,47000,44400,42300,36900,
  33300,29900,28100,27500,26700,25700,25300,24800,23800,21800,18900,14210,12300,10300,7750
  ,7550,7300,7100,6800,6400,6000,5930,5670,5510,5200,4612,3910,3201,2761,1991,1721,1512,1401,
  1330,1180,872,782,671,590,533,510,450,420,350,338,301,275,261,252,233,215,199,180,165,150,140}; // Corresponding resistance values




 uint32_t calculateResistance(uint32_t adc_value)
 {
 	uint32_t voltage_adc = adc_value * (voltage_supply / 4096); // تحويل القراءة من ADC إلى جهد
 	uint32_t resistance_ntc = voltage_adc * 5000 / (voltage_supply - voltage_adc); // مقاومة المستشعر بناءً على معادلة المقسم الجهد

     return resistance_ntc;
 }

 uint16_t calculateTemperature(uint32_t resistance_ntc)
 {

     uint32_t table_length = sizeof(resistanceValues) / sizeof(resistanceValues[0]);
     uint8_t i;
       for (i = 0; i < table_length; i++) {


     	      if (resistance_ntc <= resistanceValues[i] && resistance_ntc >= resistanceValues[i + 1]) {
     	    	  break;
     	      }}

     	    	  uint32_t resistanceDiff = resistanceValues[i + 1] - resistanceValues[i];
     	          uint32_t temperatureDiff = temperatureValues[i + 1] - temperatureValues[i];
     	          uint32_t resistanceRatio = temperatureDiff/resistanceDiff;
     	          currentTemperature = temperatureValues[i] + (resistanceRatio * (resistance_ntc-resistanceValues[i]));
     	          return currentTemperature;
     	       	  }

 // Function to calculate PID output
float calculatePID(uint16_t setpoint,uint16_t input) {
     float error = setpoint - input;
     integral = integral + error;
     float derivative = error - previousError;
     outputPWM = (KP * error) + (KI * integral) + (KD * derivative);
     previousError = error;

     if (outputPWM < 0.0) {
         outputPWM = 0.0;
     } else if (outputPWM > 100.0) {
         outputPWM = 100.0;
     }

     return outputPWM;
 }







/* Private function prototypes -----------------------------------------------*/


 /* Main function ------------------------------------------------------------*/

int main(void){

	Module_Init();    //Initialize Module &  BitzOS

	//Don't place your code here.
	for(;;){
		//  readPxMutex(2,Rx_Data,25,10,10);

	}
}

/*-----------------------------------------------------------*/

/* User Task */
void UserTask(void *argument){

	ADCSelectChannel(2,"top");



	while(1){

   while(flagParamterTemp!=1){};

			 while (setPointTempature>=currentTemperature){

				 ReadADCChannel(2,"top",&adcValue);
				 resistanceNTC =calculateResistance(adcValue);
				 currentTemperature = calculateTemperature(resistanceNTC);
				 HAL_Delay(500);
				 Output_PWM(calculatePID(setPointTempature,currentTemperature));



			messageParams[0] =currentTemperature;
		 SendMessageToModule(1,CODE_H18R1_ParamterTemp,1);
		 Delay_ms(20);


		 }

		 SendMessageToModule(1,CODE_H18R1_ResetToZero,0);

		 flagCalibration =1;

		 while (flagCalibration ==1){

		 if(setPointTempature==0)
		 {Output_PWM(0);
		 flagCalibration=0;
		 flagParamterTemp=0;
		 SendMessageToModule(1,CODE_H18R1_ResetToZero,0);
		 }
		 else
		 {

			 ReadADCChannel(2,"top",&adcValue);
			 resistanceNTC =calculateResistance(adcValue);
			 currentTemperature = calculateTemperature(resistanceNTC);
			 HAL_Delay(500);
			 Output_PWM(calculatePID(setPointTempature,currentTemperature));



	 messageParams[0] =currentTemperature;
     SendMessageToModule(1,CODE_H18R1_ParamterTemp,1);
	 Delay_ms(20);


		 }
		 }

	}
}

/*-----------------------------------------------------------*/
