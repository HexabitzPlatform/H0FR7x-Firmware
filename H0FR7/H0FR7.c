/*
 BitzOS (BOS) V0.2.7 - Copyright (C) 2017-2022 Hexabitz
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
#include "H0FR7_inputs.h"
#include "H0FR6_adc.h"
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
extern TIM_HandleTypeDef htim1;
void MX_TIM1_Init(void);
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
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);

/* Create CLI commands --------------------------------------------------------*/

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

	 HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

	  /** Initializes the peripherals clocks
	  */
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2;
	  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
	  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
	  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
	    PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLKSOURCE_PCLK1;
	  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
		__HAL_RCC_PWR_CLK_ENABLE();
		HAL_PWR_EnableBkUpAccess();
		__HAL_RCC_TIM1_CLK_ENABLE();


		HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

		HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

		__SYSCFG_CLK_ENABLE()
		;
	  HAL_NVIC_SetPriority(SysTick_IRQn,0,0);
	
}

/*-----------------------------------------------------------*/


/* --- Save array topology and Command Snippets in Flash RO --- 
 */
uint8_t SaveToRO(void){
	BOS_Status result =BOS_OK;
	HAL_StatusTypeDef FlashStatus =HAL_OK;
	uint16_t add =2, temp =0;
	uint8_t snipBuffer[sizeof(snippet_t) + 1] ={0};
	
	HAL_FLASH_Unlock();
	
	/* Erase RO area */
	FLASH_PageErase(FLASH_BANK_1,RO_START_ADDRESS);
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
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,              //HALFWORD
						//TOBECHECKED
					RO_START_ADDRESS + add,array[i - 1][j]);
					add +=2;
					FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
					if(FlashStatus != HAL_OK){
						return pFlash.ErrorCode;
					}
					else{
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
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
			memcpy((uint8_t* )&snipBuffer[1],(uint8_t* )&snippets[s],sizeof(snippet_t));
			// Copy the snippet struct buffer (20 x numOfRecordedSnippets). Note this is assuming sizeof(snippet_t) is even.
			for(uint8_t j =0; j < (sizeof(snippet_t) / 2); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint16_t* )&snipBuffer[j * 2]);
				//HALFWORD
				//TOBECHECKED
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=2;
				}
			}
			// Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped
			for(uint8_t j =0; j < ((strlen(snippets[s].cmd) + 1) / 2); j++){
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,currentAdd,*(uint16_t* )(snippets[s].cmd + j * 2));
				//HALFWORD
				//TOBECHECKED
				FlashStatus =FLASH_WaitForLastOperation((uint32_t ) HAL_FLASH_TIMEOUT_VALUE);
				if(FlashStatus != HAL_OK){
					return pFlash.ErrorCode;
				}
				else{
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR,FLASH_CR_PG);
					currentAdd +=2;
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
	__HAL_RCC_GPIOB_CLK_ENABLE();
	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();
	//MX_TIM1_Init();

	/* Create module special task (if needed) */
#ifdef H0FR7
	/* ADC init */
	MX_ADC1_Init();

	/* Create a Mosfet task */
	xTaskCreate(MosfetTask,(const char* ) "MosfetTask",(2*configMINIMAL_STACK_SIZE),NULL,osPriorityNormal - osPriorityIdle,&MosfetHandle);
#endif
	/* Create a timeout timer for Switch_on() API */
	xTimerSwitch =xTimerCreate("SwitchTimer",pdMS_TO_TICKS(1000),pdFALSE,(void* )1,SwitchTimerCallback);

	/* Switch GPIO */
	Switch_Init();

}

void initialValue(void)
{
	mosfetCurrent=0;
}

/*-----------------------------------------------------------*/
/* --- H0FR7 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code,uint8_t port,uint8_t src,uint8_t dst,uint8_t shift){
	Module_Status result =H0FRx_OK;


	switch(code){

		default:
			result =H0FRx_ERR_UnknownMessage;
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
	else if(huart->Instance == USART6)
		return P6;
	
	return 0;
}

/*-----------------------------------------------------------*/

/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void){

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



/*-----------------------------------------------------------*/
/* --- Switch timer callback ---*/
void SwitchTimerCallback(TimerHandle_t xTimerSwitch) {

	Output_off();

#ifdef H0FR7

	HAL_ADC_Stop(&hadc1);
	uint32_t tid = 0;

	/* close DMA stream */
	tid = (uint32_t) pvTimerGetTimerID(xTimerSwitch);
	if (TIMERID_TIMEOUT_MEASUREMENT == tid) {
		startMeasurement = STOP_MEASUREMENT;
		mosfetMode = REQ_IDLE;		// Stop the streaming task
	}
#endif

}
/*-----------------------------------------------------------*/

#if defined(H0FR6) || defined(H0FR7)
/* --- Set Switch PWM frequency and dutycycle ---*/
Module_Status Set_Switch_PWM(uint32_t freq, float dutycycle) {
	Module_Status result = H0FRx_OK;
	uint32_t ARR = PWM_TIMER_CLOCK / freq;

	if (Switch_state != STATE_PWM)
		MX_TIM1_Init();

	/* PWM period */
	htim1.Instance->ARR = ARR - 1;

	/* PWM duty cycle */
	htim1.Instance->CCR3 = ((float) dutycycle / 100.0f) * ARR;

	if (HAL_TIM_PWM_Start(&htim1, _Switch_TIM_CH) != HAL_OK)
		return H0FRx_ERROR;

	return result;
}
#endif
/*-----------------------------------------------------------*/

#ifdef H0FR7
/* --- ADC Calculation for the Current in H0FR7 (Mosfet)---*/
static float Current_Calculation(void) {
	ADC_ChannelConfTypeDef sConfig ={0};

	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_79CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1,&sConfig);
	Output_on(3000);
	Delay_ms(1000);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,10);
	rawValues =HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_NONE;
	sConfig.SamplingTime = ADC_SAMPLETIME_79CYCLES_5;
	HAL_ADC_ConfigChannel(&hadc1,&sConfig);
	return (rawValues * ADC_CONVERSION);
}
/*-----------------------------------------------------------*/

/* --- Stop ADC Calculation and Switch off Mosfet ---*/
static void mosfetStopMeasurement(void) {
	Output_off();
	HAL_ADC_Stop(&hadc1);
}

void TIM1_DeInit(void) {
	HAL_NVIC_DisableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
	HAL_TIM_Base_DeInit(&htim1);
	HAL_TIM_PWM_DeInit(&htim1);
	__TIM1_CLK_DISABLE();
}
/*-----------------------------------------------------------*/


/* --- Definition of Mosfet Prime Task ---*/
static void MosfetTask(void *argument) {

	uint32_t t0 = 0;
	while (1) {
		switch (mosfetMode) {
				case REQ_STREAM_PORT_CLI:
					t0 = HAL_GetTick();
					Current = Current_Calculation();
					SendMeasurementResult(mosfetMode, Current, 0, 0, NULL);
					while (HAL_GetTick() - t0 < (mosfetPeriod - 1) && !stopB) {
						taskYIELD();
					}
					break;

				case REQ_STREAM_VERBOSE_PORT_CLI:
					t0 = HAL_GetTick();
					Current = Current_Calculation();
					SendMeasurementResult(mosfetMode, Current, 0, 0, NULL);
					while (HAL_GetTick() - t0 < (mosfetPeriod - 1) && !stopB) {
						taskYIELD();
					}
					break;

				case REQ_STREAM_PORT:
					t0 = HAL_GetTick();
					Current = Current_Calculation();
					SendMeasurementResult(mosfetMode, Current, 0, PcPort, NULL);
					while (HAL_GetTick() - t0 < (mosfetPeriod - 1) && !stopB) {
						taskYIELD();
					}
					break;

				case REQ_STREAM_BUFFER:
					t0 = HAL_GetTick();
					Current = Current_Calculation();
					SendMeasurementResult(mosfetMode, Current, mosfetModule,
							0, ptrBuffer);
					while (HAL_GetTick() - t0 < (mosfetPeriod - 1) && !stopB) {
						taskYIELD();
					}
					break;

				case REQ_STOP:
					Stop_current_measurement();
					break;

				default:
					mosfetMode = REQ_STOP;
					break;
				}

				taskYIELD();
			}
}
/*-----------------------------------------------------------*/

/* --- Send measurement results --- */
static Module_Status SendMeasurementResult(uint8_t request, float value, uint8_t module,
		uint8_t port, float *Buffer) {

	Module_Status state = H0FRx_OK;
	int8_t *pcOutputString;
	static const int8_t *pcCurrentMsg = (int8_t*) "Current: %.2f\r\n";
	static const int8_t *pcCurrentVerboseMsg = (int8_t*) "%.2f\r\n";
	static const int8_t *pcOutTimeout = (int8_t*) "TIMEOUT\r\n";
	float message;
	static uint8_t temp[4];

	/* Get CLI output buffer */
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();


	message = value;

	// If measurement timeout occured
	if (mosfetState == REQ_TIMEOUT) {
		switch (request) {
				case REQ_SAMPLE_CLI:
				case REQ_STREAM_PORT_CLI:
					request = REQ_TIMEOUT_CLI;
					break;
				case REQ_SAMPLE_VERBOSE_CLI:
				case REQ_STREAM_VERBOSE_PORT_CLI:
					request = REQ_TIMEOUT_VERBOSE_CLI;
					break;
				case REQ_STREAM_BUFFER:
					request = REQ_TIMEOUT_BUFFER;
					break;
				default:
					break;
		}
	}

	// Send the value to appropriate outlet
	switch (mosfetMode) {
	case REQ_SAMPLE_CLI:
		case REQ_STREAM_PORT_CLI:
			sprintf((char*) pcOutputString, (char*) pcCurrentMsg, message);
			writePxMutex(PcPort, (char*) pcOutputString,
					strlen((char*) pcOutputString), cmd500ms, HAL_MAX_DELAY);
			CheckForEnterKey();
			break;

		case REQ_SAMPLE_VERBOSE_CLI:
		case REQ_STREAM_VERBOSE_PORT_CLI:

			sprintf((char*) pcOutputString, (char*) pcCurrentVerboseMsg, message);
			writePxMutex(PcPort, (char*) pcOutputString,
					strlen((char*) pcOutputString), cmd500ms, HAL_MAX_DELAY);
			CheckForEnterKey();
			break;

		case REQ_SAMPLE_PORT:
		case REQ_STREAM_PORT:

			if (module == myID) {
				temp[0] = *((__IO uint8_t*) (&message) + 3);
				temp[1] = *((__IO uint8_t*) (&message) + 2);
				temp[2] = *((__IO uint8_t*) (&message) + 1);
				temp[3] = *((__IO uint8_t*) (&message) + 0);
				writePxMutex(port, (char*) &temp, 4 * sizeof(uint8_t), 10, 10);
			} else {
				messageParams[0] = port;
				messageParams[1] = *((__IO uint8_t*) (&message) + 3);
				messageParams[2] = *((__IO uint8_t*) (&message) + 2);
				messageParams[3] = *((__IO uint8_t*) (&message) + 1);
				messageParams[4] = *((__IO uint8_t*) (&message) + 0);
				SendMessageToModule(module, CODE_PORT_FORWARD,
						sizeof(uint32_t) + 1);
			}

			break;

		case REQ_SAMPLE_BUFFER:
		case REQ_STREAM_BUFFER:
			memset(Buffer, 0, sizeof(float));
			memcpy(Buffer, &message, sizeof(float));
			break;



		case REQ_TIMEOUT_CLI:
			strcpy((char*) pcOutputString, (char*) pcOutTimeout);
			writePxMutex(PcPort, (char*) pcOutputString,
				strlen((char*) pcOutputString), cmd500ms, HAL_MAX_DELAY);
			CheckForEnterKey();
			break;

		case REQ_TIMEOUT_VERBOSE_CLI:
			sprintf((char*) pcOutputString, (char*) pcCurrentVerboseMsg, 0);
			writePxMutex(PcPort, (char*) pcOutputString,
				strlen((char*) pcOutputString), cmd500ms, HAL_MAX_DELAY);
			CheckForEnterKey();
			break;

		default:
			break;
	}

	return (state);
}
/*-----------------------------------------------------------*/

/* --- Check for CLI stop key --- */
static void CheckForEnterKey(void) {
	stopB = 0;
	// Look for ENTER key to stop the stream
	for (uint8_t chr = 0; chr < MSG_RX_BUF_SIZE; chr++) {
		if (UARTRxBuf[PcPort - 1][chr] == '\r') {
			UARTRxBuf[PcPort - 1][chr] = 0;
			mosfetMode = REQ_STOP;		// Stop the streaming task
			xTimerStop(xTimerSwitch, 0); // Stop any running timeout timer
			stopB = 1;
			break;
		}
	}
}
/*-----------------------------------------------------------*/
#endif
/* -----------------------------------------------------------------------
 |								  APIs							          | 																 	|
/* -----------------------------------------------------------------------
 */
/* --- Turn on the solid state Switch ---
 */
Module_Status Output_on(uint32_t timeout) {
	Module_Status result = H0FRx_OK;

#if defined(H0FR6) || defined(H0FR7)
	/* Turn off PWM and re-initialize GPIO if needed */
	if (Switch_state == STATE_PWM) {
		HAL_TIM_PWM_Stop(&htim1, _Switch_TIM_CH);
		TIM1_DeInit();
		Switch_Init();

		//Switch_Init();
	}
#endif

	/* Turn on */
	HAL_GPIO_WritePin(_Switch_PORT, _Switch_PIN, GPIO_PIN_SET);

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
	Module_Status result = H0FRx_OK;

#if defined(H0FR6) || defined(H0FR7)
	/* Turn off PWM and re-initialize GPIO if needed */
	if (Switch_state == STATE_PWM) {
		HAL_TIM_PWM_Stop(&htim1, _Switch_TIM_CH);
		TIM1_DeInit();
		Switch_Init();
	}
#endif

	/* Turn off */
	HAL_GPIO_WritePin(_Switch_PORT, _Switch_PIN, GPIO_PIN_RESET);

	/* Indicator LED */
	if (SwitchindMode)
		IND_OFF();

	/* Update Switch state */
	Switch_state = STATE_OFF;

	return result;
}

/*-----------------------------------------------------------*/
#if defined(H0FR6) || defined(H0FR7)
/* --- Turn-on Switch with pulse-width modulation (PWM) ---
 dutyCycle: PWM duty cycle in precentage (0 to 100)
 */
Module_Status Output_PWM(float dutyCycle) {
	Module_Status result = H0FRx_OK;

	if (dutyCycle < 0 || dutyCycle > 100)
		return H0FRx_ERR_Wrong_Value;

	/* Start the PWM */
	HAL_GPIO_WritePin(_Switch_PORT, _Switch_PIN, GPIO_PIN_SET);
	result = Set_Switch_PWM(Switch_PWM_DEF_FREQ, dutyCycle);

	if (result == H0FRx_OK) {
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
#endif

#ifdef H0FR7
/* --- Read the Current value with Analog Digital Converter (ADC) in H0FR7 ---
 */
float Sample_current_measurement(void) {
	float temp;
	mosfetMode = REQ_SAMPLE;
	startMeasurement = START_MEASUREMENT;

	if (mosfetState == REQ_TIMEOUT) {
		return 0;
	} else {
		temp = Current_Calculation();
		mosfetState = REQ_IDLE;
		return temp;
	}
}
/*-----------------------------------------------------------*/

/* --- Stream measurements continuously to a port --- */
float Stream_current_To_Port(uint8_t Port, uint8_t Module, uint32_t Period,
		uint32_t Timeout) {

	mosfetPort = Port;
	mosfetModule = Module;
	mosfetPeriod = Period;
	mosfetTimeout = Timeout;
	mosfetMode = REQ_STREAM_PORT;

	if ((mosfetTimeout > 0) && (mosfetTimeout < 0xFFFFFFFF)) {
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimerSwitch = xTimerCreate("mosfetTimer",
				pdMS_TO_TICKS(mosfetTimeout), pdFALSE,
				(void*) TIMERID_TIMEOUT_MEASUREMENT, SwitchTimerCallback);
		/* Start the timeout timer */
		xTimerStart(xTimerSwitch, portMAX_DELAY);
	}
	return (H0FRx_OK);
}
/*-----------------------------------------------------------*/

/* --- stream Current value to CLI
 */
float Stream_current_To_CLI(uint32_t Period, uint32_t Timeout) {

	mosfetPeriod = Period;
	mosfetTimeout = Timeout;
	mosfetMode = REQ_STREAM_PORT_CLI;

	if ((mosfetTimeout > 0) && (mosfetTimeout < 0xFFFFFFFF)) {
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimerSwitch = xTimerCreate("mosfetTimer",
				pdMS_TO_TICKS(mosfetTimeout), pdFALSE,
				(void*) TIMERID_TIMEOUT_MEASUREMENT, SwitchTimerCallback);
		/* Start the timeout timer */
		xTimerStart(xTimerSwitch, portMAX_DELAY);
	}
	if (mosfetTimeout > 0) {
		startMeasurement = START_MEASUREMENT;
	}

	return (H0FRx_OK);
}
/*-----------------------------------------------------------*/

/* --- stream Current value from to CLI
 */
float Stream_current_To_CLI_V(uint32_t Period, uint32_t Timeout) {

	mosfetPeriod = Period;
	mosfetTimeout = Timeout;
	mosfetMode = REQ_STREAM_VERBOSE_PORT_CLI;

	if ((mosfetTimeout > 0) && (mosfetTimeout < 0xFFFFFFFF)) {
		/* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimerSwitch = xTimerCreate("mosfetTimer",
				pdMS_TO_TICKS(mosfetTimeout), pdFALSE,
				(void*) TIMERID_TIMEOUT_MEASUREMENT, SwitchTimerCallback);
		/* Start the timeout timer */
		xTimerStart(xTimerSwitch, portMAX_DELAY);
	}
	if (mosfetTimeout > 0) {
		startMeasurement = START_MEASUREMENT;
	}
	return (H0FRx_OK);
}
/*-----------------------------------------------------------*/

float Stream_current_To_Buffer(float *Buffer, uint32_t Period, uint32_t Timeout)
{
	mosfetPeriod=Period;
	mosfetTimeout=Timeout;
	ptrBuffer=Buffer;
	mosfetMode=REQ_STREAM_BUFFER;

	if ((mosfetTimeout > 0) && (mosfetTimeout < 0xFFFFFFFF))
  {
	  /* start software timer which will create event timeout */
		/* Create a timeout timer */
		xTimerSwitch = xTimerCreate( "mosfetTimer", pdMS_TO_TICKS(mosfetTimeout), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, SwitchTimerCallback );
		/* Start the timeout timer */
		xTimerStart( xTimerSwitch, portMAX_DELAY );
	}

	return (H0FRx_OK);
}

/*-----------------------------------------------------------*/

/* --- Stop Current measurement --- */
Module_Status Stop_current_measurement(void) {

	Module_Status state = H0FRx_OK;
	mosfetMode = REQ_IDLE;
	startMeasurement = STOP_MEASUREMENT;
	xTimerStop(xTimerSwitch, 0);

	mosfetStopMeasurement();


	return state;
}
/*-----------------------------------------------------------*/

#endif

/* -----------------------------------------------------------------------
 |								Commands							      |
   -----------------------------------------------------------------------
 */



/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
