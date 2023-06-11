/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved

 File Name     : H0FR7.c
 Description   : Source code for module H0FR7.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "H0FR7_inputs.h"
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;


/* Module exported parameters ------------------------------------------------*/
float H0FR6_Current = 0.0f;
#ifdef H0FR7
uint8_t startMeasurement = STOP_MEASUREMENT;
#endif
module_param_t modParam[NUM_MODULE_PARAMS] ={{.paramPtr =&H0FR6_Current, .paramFormat =FMT_FLOAT, .paramName ="current"}};
/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;




/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
void MX_TIM1_Init(void);
void Switch_Init(void);
extern ADC_HandleTypeDef hadc1;
TimerHandle_t xTimerSwitch = NULL;
TaskHandle_t MosfetHandle = NULL;
Switch_state_t Switch_state = STATE_OFF, Switch_Oldstate = STATE_ON;
uint8_t SwitchindMode = 0;
uint8_t stream_index = 0;
uint8_t mosfetPort, mosfetModule, mosfetState, mosfetMode;
uint32_t rawValues, mosfetPeriod, mosfetTimeout, t0, temp32;
float tempFloat, Switch_OldDC;
float mosfetBuffer = 0;
float Current = 0.0f;
float *ptrBuffer = &mosfetBuffer;
bool stopB = 0;
float mosfetCurrent __attribute__((section(".mySection")));
/* Private function prototypes -----------------------------------------------*/
void ExecuteMonitor(void);
void SwitchTimerCallback(TimerHandle_t xTimerSwitch);
Module_Status Set_Switch_PWM(uint32_t freq,float dutycycle);
void TIM1_Init(void);
void TIM1_DeInit(void);
static float Current_Calculation(void);
static void MosfetTask(void *argument);
static Module_Status SendMeasurementResult(uint8_t request,float value,uint8_t module,uint8_t port,float *buffer);
static void CheckForEnterKey(void);
static Module_Status GetStopCompletedStatus(uint32_t *pStopStatus);

/* Create CLI commands --------------------------------------------------------*/
portBASE_TYPE onCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE offCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE toggleCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE ledModeCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE pwmCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);

/* CLI command structure : on */
const CLI_Command_Definition_t onCommandDefinition =
		{ (const int8_t*) "on", /* The command string to type. */
				(const int8_t*) "on:\r\n Turn solid state Switch on with a timeout (ms) (1st par.). Use 'inf' to turn on constantly\r\n\r\n",
				onCommand, /* The function to run. */
				1 /* One parameter is expected. */
		};
/*-----------------------------------------------------------*/
/* CLI command structure : off */
const CLI_Command_Definition_t offCommandDefinition =
	{ (const int8_t*) "off", /* The command string to type. */
(const int8_t*) "off:\r\n Turn solid state Switch off\r\n\r\n", offCommand, /* The function to run. */
0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : toggle */
const CLI_Command_Definition_t toggleCommandDefinition = {
		(const int8_t*) "toggle", /* The command string to type. */
		(const int8_t*) "toggle:\r\n Toggle solid state Switch\r\n\r\n",
		toggleCommand, /* The function to run. */
		0 /* No parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : ledMode */
const CLI_Command_Definition_t ledModeCommandDefinition =
		{ (const int8_t*) "ledmode", /* The command string to type. */
				(const int8_t*) "ledMode:\r\n Set solid state Switch indicator LED mode ('on' or 'off') (1st par.)\r\n\r\n",
				ledModeCommand, /* The function to run. */
				1 /* One parameter is expected. */
		};
/*-----------------------------------------------------------*/
/* CLI command structure : pwm */
const CLI_Command_Definition_t pwmCommandDefinition =
		{ (const int8_t*) "pwm", /* The command string to type. */
				(const int8_t*) "pwm:\r\n Control the solid state Switch with pulse-width modulation (PWM) signal with a percentage duty cycle (0-100) (1st par.)\r\n\r\n",
				pwmCommand, /* The function to run. */
				1 /* One parameter is expected. */
		};
/*-----------------------------------------------------------*/


/* -----------------------------------------------------------------------
 |												 Private Functions	 														|
 -----------------------------------------------------------------------
 */

/**
 * @brief  System Clock Configuration
 *         The system Clock is configured as follow :
 *            System Clock source            = PLL (HSE)
 *            SYSCLK(Hz)                     = 48000000
 *            HCLK(Hz)                       = 48000000
 *            AHB Prescaler                  = 1
 *            APB1 Prescaler                 = 1
 *            HSE Frequency(Hz)              = 8000000
 *            PREDIV                         = 1
 *            PLLMUL                         = 6
 *            Flash Latency(WS)              = 1
 * @param  None
 * @retval None
 */
void SystemClock_Config(void){
	  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
	  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

	  /** Configure the main internal regulator output voltage
	  */
	  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
	  /** Initializes the RCC Oscillators according to the specified parameters
	  * in the RCC_OscInitTypeDef structure.
	  */
	  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
	  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	  RCC_OscInitStruct.PLL.PLLN = 12;
	  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
      HAL_RCC_OscConfig(&RCC_OscInitStruct);

	  /** Initializes the CPU, AHB and APB buses clocks
	  */
	  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
	                              |RCC_CLOCKTYPE_PCLK1;
	  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

	  /** Initializes the peripherals clocks
	  */
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
	  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2;
	  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
	  HAL_NVIC_SetPriority(SysTick_IRQn,0,0);

}

/*-----------------------------------------------------------*/


/* --- Save array topology and Command Snippets in Flash RO ---
 */
uint8_t SaveToRO(void){
	BOS_Status result =BOS_OK;
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint16_t add =8;
    uint16_t temp =0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};

	HAL_FLASH_Unlock();

	/* Erase RO area */
	FLASH_PageErase(FLASH_BANK_1,RO_START_ADDRESS);
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	FLASH_PageErase(FLASH_BANK_1,RO_MID_ADDRESS);
	//TOBECHECKED
	FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
	if(FlashStatus != HAL_OK){
		return pFlash.ErrorCode;
	}
	else{
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR,FLASH_CR_PER);
	}

	/* Save number of modules and myID */
	if(myID){
		temp =(uint16_t )(N << 8) + myID;
		//HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,RO_START_ADDRESS,temp);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,RO_START_ADDRESS,temp);
		//TOBECHECKED
		FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
		if(FlashStatus != HAL_OK){
			return pFlash.ErrorCode;
		}
		else{
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
		}

		/* Save topology */
		for(uint8_t i =1; i <= N; i++){
			for(uint8_t j =0; j <= MaxNumOfPorts; j++){
				if(array[i - 1][0]){
          	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,RO_START_ADDRESS + add,array[i - 1][j]);
				 //HALFWORD 	//TOBECHECKED

					FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(FlashStatus != HAL_OK){
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
						add +=8;
					}
				}
			}
		}
	}

	// Save Command Snippets
	int currentAdd = RO_MID_ADDRESS;
	for(uint8_t s =0; s < numOfRecordedSnippets; s++){
		if(snippets[s].cond.conditionType){
			snipBuffer[0] =0xFE;		// A marker to separate Snippets
			memcpy((uint32_t* )&snipBuffer[1],(uint8_t* )&snippets[s],sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j =0; j < (sizeof(snippet_t)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )&snipBuffer[j*8]);
				//HALFWORD
				//TOBECHECKED
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j =0; j < ((strlen(snippets[s].cmd) + 1)/4); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint64_t* )(snippets[s].cmd + j*4 ));
				//HALFWORD
				//TOBECHECKED
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=8;
				}
			}
		}
	}

	HAL_FLASH_Lock();

	return result;
}

/* --- Clear array topology in SRAM and Flash RO ---
 */
uint8_t ClearROtopology(void){
	// Clear the array
	memset(array,0,sizeof(array));
	N =1;
	myID =0;

	return SaveToRO();
}
/*-----------------------------------------------------------*/

/* --- Trigger ST factory bootloader update for a remote module.
 */
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport){

	uint8_t myOutport =0, lastModule =0;
	int8_t *pcOutputString;

	/* 1. Get route to destination module */
	myOutport =FindRoute(myID,dst);
	if(outport && dst == myID){ /* This is a 'via port' update and I'm the last module */
		myOutport =outport;
		lastModule =myID;
	}
	else if(outport == 0){ /* This is a remote update */
		if(NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = route[NumberOfHops(dst)-1]; /* previous module = route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if(src == myID){
		/* Obtain the address of the output buffer.  Note there is no mutual
		 exclusion on this buffer as it is assumed only one command console
		 interface will be used at any one time. */
		pcOutputString =FreeRTOS_CLIGetOutputBuffer();

		if(outport == 0)		// This is a remote module update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateMessage,dst);
		else
			// This is a 'via port' remote update
			sprintf((char* )pcOutputString,pcRemoteBootloaderUpdateViaPortMessage,dst,outport);

		strcat((char* )pcOutputString,pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport,(char* )pcOutputString,strlen((char* )pcOutputString),cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);


	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport,myID,myOutport,myID,BIDIRECTIONAL,0xFFFFFFFF,0xFFFFFFFF,false);
}

/*-----------------------------------------------------------*/

/* --- Setup a port for remote ST factory bootloader update:
 - Set baudrate to 57600
 - Enable even parity
 - Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port){
	UART_HandleTypeDef *huart =GetUart(port);

	huart->Init.BaudRate =57600;
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);
}

/* --- H0FR7 module initialization.
 */
void Module_Peripheral_Init(void){

	/* Array ports */
	//MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();

	MX_TIM1_Init();
//	Switch_Init();
	/* Create module special task (if needed) */
}

/*-----------------------------------------------------------*/
/* --- H0FR7 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift)
{
	Module_Status result = H0FR7_OK;

	switch (code)
	{

		default:
			result = H0FR7_ERR_UnknownMessage;
			break;
	}

	return result;
}

/* --- Get the port for a given UART.
 */
uint8_t GetPort(UART_HandleTypeDef *huart){

	if(huart->Instance == USART4)
		return P1;
	else if(huart->Instance == USART2)
		return P2;
	else if(huart->Instance == USART6)
		return P3;
	else if(huart->Instance == USART3)
		return P4;
	else if(huart->Instance == USART5)
		return P5;
	else if(huart->Instance == USART1)
		return P6;

	return 0;
}
/*------------------------------------------------------------*/


/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/


/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void){

	FreeRTOS_CLIRegisterCommand(&onCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&offCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&toggleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&ledModeCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&pwmCommandDefinition);

}

/*-----------------------------------------------------------*/


/* Module special task function (if needed) */
//void Module_Special_Task(void *argument){
//
//	/* Infinite loop */
//	uint8_t cases; // Test variable.
//	for(;;){
//		/*  */
//		switch(cases){
//
//
//			default:
//				osDelay(10);
//				break;
//		}
//
//		taskYIELD();
//	}
//
//}


/*-----------------------------------------------------------*/

void initialValue(void){

}


/*-----------------------------------------------------------*/

/*-----------------------------------------------------------*/

/* --- Set Switch PWM frequency and dutycycle ---*/
Module_Status Set_Switch_PWM(uint32_t freq, float dutycycle) {
	Module_Status result = H0FR7_OK;
	uint32_t ARR = 100;
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);


	/* PWM period */
	htim1.Instance->ARR = ARR - 1;

	/* PWM duty cycle */
	htim1.Instance->CCR3 = ((float) dutycycle / 100.0f) * ARR;

	return result;
}

/* -----------------------------------------------------------------------
 |								  APIs							          | 																 	|
/* -----------------------------------------------------------------------
 */
/* --- Turn on the solid state Switch ---
 */
Module_Status Output_on(uint32_t timeout) {
	Module_Status result = H0FR7_OK;
//	MX_TIM1_Init();

	Set_Switch_PWM(1600000, 100);


	/* Turn on */
//	HAL_GPIO_WritePin(_Switch_PORT, _Switch_PIN, GPIO_PIN_SET);

	/* Indicator LED */
	if (SwitchindMode)
		IND_ON();

	/* Timeout */
	if (timeout != portMAX_DELAY) {
		/* Stop the timer if it's already running */
		if (xTimerIsTimerActive(xTimerSwitch))
			xTimerStop(xTimerSwitch, 100);
		/* Update timer timeout - This also restarts the timer */
		xTimerChangePeriod(xTimerSwitch, pdMS_TO_TICKS(timeout), 100);
	}

	/* Update Switch state */
	Switch_state = STATE_ON;
	Switch_Oldstate = Switch_state;

	return result;
}

/*-----------------------------------------------------------*/
/* --- Turn off the solid state Switch ---
 */
Module_Status Output_off(void) {
	Module_Status result = H0FR7_OK;

	/* Turn off PWM and re-initialize GPIO if needed */
		HAL_TIM_PWM_Stop(&htim1, _Switch_TIM_CH);
		htim1.Instance->CCR3=0;
//	MX_TIM1_Init();
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

//	result = Set_Switch_PWM(1600000, 0);

//	/* Turn off */
//	HAL_GPIO_WritePin(_Switch_PORT, _Switch_PIN, GPIO_PIN_RESET);

	/* Indicator LED */
	if (SwitchindMode)
		IND_OFF();

	/* Update Switch state */
	Switch_state = STATE_OFF;

	return result;
}
/*-----------------------------------------------------------*/
/* --- Toggle the solid state Switch ---
 */
Module_Status Output_toggle(void) {
	Module_Status result = H0FR7_OK;

//		MX_TIM1_Init();
//		HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	if (Switch_state) {
		result = Output_off();
	} else {
		if (Switch_Oldstate == STATE_ON)
			result = Output_on(portMAX_DELAY);
		else if (Switch_Oldstate == STATE_PWM)
			result = Output_PWM(Switch_OldDC);
	}

	return result;
}

/*-----------------------------------------------------------*/
Module_Status Output_PWM(float dutyCycle) {
	Module_Status result = H0FR7_OK;

	if (dutyCycle < 0 || dutyCycle > 100)
		return H0FR7_ERR_WrongParams;

	/* Start the PWM */

	MX_TIM1_Init();
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

	result = Set_Switch_PWM(1600000, dutyCycle);


	if (result == H0FR7_OK) {
			Switch_OldDC = dutyCycle;
			/* Update Switch state */
			Switch_state = STATE_PWM;
			Switch_Oldstate = Switch_state;
			/* Indicator LED */
			if (SwitchindMode)
				IND_ON();
		}


	return result;
}


/* -----------------------------------------------------------------------
 |								Commands							      |
   -----------------------------------------------------------------------
 */
portBASE_TYPE onCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString) {
	Module_Status result = H0FR7_OK;

	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;
	uint32_t timeout = 0;
	static const int8_t *pcOKMessage =
			(int8_t*) "Solid state Switch is turned on with timeout %d ms\r\n";
	static const int8_t *pcOKMessageInf =
			(int8_t*) "Solid state Switch is turned on without timeout\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Obtain the 1st parameter string. */
	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
	1, /* Return the first parameter. */
	&xParameterStringLength1 /* Store the parameter string length. */
	);

	if (!strcmp((char*) pcParameterString1, "inf")
			|| !strcmp((char*) pcParameterString1, "INF"))
		timeout = portMAX_DELAY;
	else
		timeout = (uint32_t) atol((char*) pcParameterString1);

	result = Output_on(timeout);

	/* Respond to the command */
	if (result == H0FR7_OK) {
		if (timeout != portMAX_DELAY) {
			sprintf((char*) pcWriteBuffer, (char*) pcOKMessage, timeout);
		} else {
			strcpy((char*) pcWriteBuffer, (char*) pcOKMessageInf);
		}
	}

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

portBASE_TYPE offCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString) {
	Module_Status result = H0FR7_OK;

	static const int8_t *pcMessage =
			(int8_t*) "Solid state Switch is turned off\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	result = Output_off();

	/* Respond to the command */
	if (result == H0FR7_OK) {
		strcpy((char*) pcWriteBuffer, (char*) pcMessage);
	}

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

portBASE_TYPE toggleCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString) {
	Module_Status result = H0FR7_OK;

	static const int8_t *pcOK1Message =
			(int8_t*) "Solid state Switch is turned on\r\n";
	static const int8_t *pcOK0Message =
			(int8_t*) "Solid state Switch is turned off\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	result = Output_toggle();

	/* Respond to the command */
	if (result == H0FR7_OK) {
		if (Switch_state) {
			strcpy((char*) pcWriteBuffer, (char*) pcOK1Message);
		} else {
			strcpy((char*) pcWriteBuffer, (char*) pcOK0Message);
		}
	}

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

portBASE_TYPE ledModeCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString) {
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;

	static const int8_t *pcOK1Message =
			(int8_t*) "Solid state Switch indicator LED is enabled\r\n";
	static const int8_t *pcOK0Message =
			(int8_t*) "Solid state Switch indicator LED is disabled\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Obtain the 1st parameter string. */
	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
	1, /* Return the first parameter. */
	&xParameterStringLength1 /* Store the parameter string length. */
	);
	if (!strcmp((char*) pcParameterString1, "on")
			|| !strcmp((char*) pcParameterString1, "ON"))
		SwitchindMode = 1;
	else if (!strcmp((char*) pcParameterString1, "off")
			|| !strcmp((char*) pcParameterString1, "OFF"))
		SwitchindMode = 0;

	/* Respond to the command */
	if (SwitchindMode) {
		strcpy((char*) pcWriteBuffer, (char*) pcOK1Message);
	} else {
		strcpy((char*) pcWriteBuffer, (char*) pcOK0Message);
	}

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/
portBASE_TYPE pwmCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen,
		const int8_t *pcCommandString) {
	Module_Status result = H0FR7_OK;

	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;
	float dutycycle = 0;
	static const int8_t *pcOKMessage =
			(int8_t*) "Solid state Switch is pulse-width modulated with %.1f%% duty cycle\r\n";
	static const int8_t *pcWrongValue =
			(int8_t*) "Wrong duty cycle value. Acceptable range is 0 to 100\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Obtain the 1st parameter string. */
	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, /* The command string itself. */
	1, /* Return the first parameter. */
	&xParameterStringLength1 /* Store the parameter string length. */
	);

	dutycycle = (float) atof((char*) pcParameterString1);

	if (dutycycle < 0.0f || dutycycle > 100.0f)
		result = H0FR7_ERR_Wrong_Value;
	else
		result = Output_PWM(dutycycle);

	/* Respond to the command */
	if (result == H0FR7_OK) {
		sprintf((char*) pcWriteBuffer, (char*) pcOKMessage, dutycycle);
	} else if (result == H0FR7_ERR_Wrong_Value) {
		strcpy((char*) pcWriteBuffer, (char*) pcWrongValue);
	}

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
