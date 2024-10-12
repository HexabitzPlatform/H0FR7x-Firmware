/*
 BitzOS (BOS) V0.3.6 - Copyright (C) 2017-2024 Hexabitz
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
#include "H0FR7_adc.h"

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
float H0FR7_Current = 0.0f;

uint8_t startMeasurement = STOP_MEASUREMENT;

module_param_t modParam[NUM_MODULE_PARAMS] = {{.paramPtr =&H0FR7_Current, .paramFormat =FMT_FLOAT, .paramName ="current"}};
/* Exported variables */
extern FLASH_ProcessTypeDef pFlash;
extern uint8_t numOfRecordedSnippets;

/* Private variables ---------------------------------------------------------*/
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim14;


/* FreeRTOS Task & Timer */
TimerHandle_t xTimerSwitch = NULL;
TaskHandle_t MosfetHandle = NULL;
 Switch_state_t Switch_state;
uint8_t SwitchindMode = 0;
uint8_t stream_index = 0;
uint8_t mosfetPort, mosfetModule, mosfetState, mosfetMode;
uint32_t rawValues, mosfetPeriod, mosfetTimeout, t0, temp32;
float tempFloat;
float mosfetBuffer = 0;
float Current = 0.0f;
float *ptrBuffer = &mosfetBuffer;
bool stopB = 0;
float mosfetCurrent __attribute__((section(".mySection")));
static float Current_Calculation(void);

/* Private function prototypes -----------------------------------------------*/
void ExecuteMonitor(void);
void SwitchTimerCallback(TimerHandle_t xTimerSwitch);
extern void MX_TIM14_Init(void);

static void MosfetTask(void *argument);
static Module_Status SendMeasurementResult(uint8_t request,float value,uint8_t module,uint8_t port,float *buffer);
static Module_Status GetStopCompletedStatus(uint32_t *pStopStatus);
static void CheckForEnterKey(void);


/* Create CLI commands --------------------------------------------------------*/
portBASE_TYPE onCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE offCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE toggleCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE ledModeCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE pwmCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE mosfetSampleCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE mosfetStreamCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
portBASE_TYPE mosfetStopCommand(int8_t *pcWriteBuffer,size_t xWriteBufferLen,const int8_t *pcCommandString);
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
/* CLI command structure : sample */
const CLI_Command_Definition_t mosfetSampleCommandDefinition =
		{ (const int8_t*) "sample", /* The command string to type. */
		(const int8_t*) "sample:\r\n Sample command to get the Current consumption in (Amp)\r\n\r\n",
		mosfetSampleCommand, /* The function to run. */
		0 /* Zero parameter is expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : stream */
const CLI_Command_Definition_t mosfetStreamCommandDefinition =
		{ (const int8_t*) "stream", /* The command string to type. */
		(const int8_t*) "stream:\r\nStream measurements to the CLI with this syntax:\n\r\tstream period(in ms) timeout(in ms)\n\r\tstream period timeout -v\t(for verbose output)\
			\n\rOr to a specific port in a specific module with this syntax:\r\n\tstream period timeout port(p1..px) module\n\rOr to internal buffer with this syntax:\r\n\tstream period timeout buffer.\t(Buffer here is a literal value and can be accessed in the CLI using module parameter: current)\r\n\r\n",
		mosfetStreamCommand, /* The function to run. */
		-1 /* Multiple parameters are expected. */
};
/*-----------------------------------------------------------*/
/* CLI command structure : stop */
const CLI_Command_Definition_t mosfetStopCommandDefinition = {
		(const int8_t*) "stop", /* The command string to type. */
		(const int8_t*) "stop:\r\nStop continuous or timed measurement\r\n\r\n",
		mosfetStopCommand, /* The function to run. */
		0 /* No parameters are expected. */
};

/* -----------------------------------------------------------------------
 |								 Private Functions	 					  |
 -------------------------------------------------------------------------
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
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
//	MX_USART4_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();
	/* Timer init */
	MX_TIM14_Init();
	/* ADC init */
	MX_ADC1_Init();

	 //Circulating DMA Channels ON All Module
	 for(int i=1;i<=NumOfPorts;i++){
		  if(GetUart(i)==&huart1)
				   { index_dma[i-1]=&(DMA1_Channel1->CNDTR); }
		  else if(GetUart(i)==&huart2)
				   { index_dma[i-1]=&(DMA1_Channel2->CNDTR); }
		  else if(GetUart(i)==&huart3)
				   { index_dma[i-1]=&(DMA1_Channel3->CNDTR); }
		  else if(GetUart(i)==&huart4)
				   { index_dma[i-1]=&(DMA1_Channel4->CNDTR); }
		  else if(GetUart(i)==&huart5)
				   { index_dma[i-1]=&(DMA1_Channel5->CNDTR); }
		  else if(GetUart(i)==&huart6)
				   { index_dma[i-1]=&(DMA1_Channel6->CNDTR); }
		}

	/* Create module special task (if needed) */

	/* Create a Mosfet task */
	xTaskCreate(MosfetTask,(const char* ) "MosfetTask",(2*configMINIMAL_STACK_SIZE),NULL,0,&MosfetHandle);

	/* start software timer which will create event timeout */
	xTimerSwitch = xTimerCreate( "mosfetTimer", pdMS_TO_TICKS(FREERTOS_TIMER_DEFAULT), pdFALSE, ( void * ) TIMERID_TIMEOUT_MEASUREMENT, SwitchTimerCallback );

}

/*-----------------------------------------------------------*/
void initialValue(void)
{
	mosfetCurrent=0;
}
/*-----------------------------------------------------------*/
/* --- H0FR7 message processing task.
 */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift)
{
	Module_Status result = H0FR7_OK;

	uint32_t period;
	uint32_t timeout;

			switch (code) {
			case CODE_H0FR7_ON:
				temp32 = cMessage[port - 1][shift] + ((uint32_t) cMessage[port - 1][1 + shift] << 8)+ ((uint32_t) cMessage[port - 1][2 + shift] << 16)+ ((uint32_t)cMessage[port - 1][3 + shift]<< 24);
				OutputOn(temp32);
				break;

			case CODE_H0FR7_OFF:
				OutputOff();
				break;

			case CODE_H0FR7_TOGGLE:
				OutputToggle();
				break;

			case CODE_H0FR7_PWM:
				tempFloat = (float)cMessage[port - 1][shift];
		//		tempFloat = (float) (((uint64_t) cMessage[port - 1][shift] )+ ((uint64_t) cMessage[port - 1][1 + shift] << 8)+ ((uint64_t) cMessage[port - 1][2 + shift] << 16)+ ((uint64_t) cMessage[port - 1][3 + shift] <<24));
				OutputPWM(tempFloat);
				break;
			case CODE_H0FR7_SAMPLE_PORT:
				SampleCurrentMeasurement(&Current);
				SendMeasurementResult(REQ_SAMPLE, Current, cMessage[port - 1][1+shift], cMessage[port - 1][shift], NULL);
				break;
			case CODE_H0FR7_STREAM_PORT:
				period = ((uint32_t) cMessage[port - 1][5 + shift] << 24)+ ((uint32_t) cMessage[port - 1][4 + shift] << 16)+ ((uint32_t) cMessage[port - 1][3 + shift] << 8)+ cMessage[port - 1][2 + shift];
				timeout = ((uint32_t) cMessage[port - 1][9 + shift] << 24)+ ((uint32_t) cMessage[port - 1][8 + shift] << 16)+ ((uint32_t) cMessage[port - 1][7 + shift] << 8)+ cMessage[port - 1][6 + shift];
				Stream_current_To_Port(cMessage[port - 1][shift], cMessage[port - 1][1 + shift], period, timeout);
				break;
			case CODE_H0FR7_STREAM_BUFFER:
				period = ((uint32_t) cMessage[port - 1][3 + shift] << 24)	+ ((uint32_t) cMessage[port - 1][2 + shift] << 16)+ ((uint32_t) cMessage[port - 1][1 + shift] << 8)+ cMessage[port - 1][shift];
				timeout = ((uint32_t) cMessage[port - 1][7 + shift] << 24)+ ((uint32_t) cMessage[port - 1][6 + shift] << 16)+ ((uint32_t) cMessage[port - 1][5 + shift] << 8)+ cMessage[port - 1][4 + shift];
				Stream_current_To_Buffer(&mosfetBuffer, period, timeout);
				break;
			case CODE_H0FR7_STOP_MEASUREMENT:
				Stop_current_measurement();
				break;

			default:
				result = H0FR7_ERR_UnknownMessage;
				break;
			}

	return result;
}

/*-----------------------------------------------------------*/
/* --- Get the port for a given UART.
 */
uint8_t GetPort(UART_HandleTypeDef *huart){
	if(huart->Instance == USART5)
		return P1;
	else if(huart->Instance == USART2)
		return P2;
	else if(huart->Instance == USART3)
		return P3;
	else if(huart->Instance == USART1)
		return P4;
	else if(huart->Instance == USART5)
		return P5;

	return 0;
}

/*------------------------------------------------------------*/
/* --- Register this module CLI Commands
 */
void RegisterModuleCLICommands(void){

	FreeRTOS_CLIRegisterCommand(&onCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&offCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&toggleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&ledModeCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&pwmCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&mosfetSampleCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&mosfetStreamCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&mosfetStopCommandDefinition);

}

/*-----------------------------------------------------------*/
/* --- Switch timer callback ---*/
void SwitchTimerCallback(TimerHandle_t xTimerSwitch) {

	OutputOff();

	HAL_ADC_Stop(&hadc1);
	uint32_t tid = 0;

	/* close DMA stream */
	tid = (uint32_t) pvTimerGetTimerID(xTimerSwitch);
	if (TIMERID_TIMEOUT_MEASUREMENT == tid) {
		startMeasurement = STOP_MEASUREMENT;
		mosfetMode = REQ_IDLE;		// Stop the streaming task
	}
}

/*-----------------------------------------------------------*/
/* --- ADC Calculation for the Current in H0FR7 (Mosfet)---*/
static float Current_Calculation(void) {
	float ADCconvertedData_mVolt;
	float ADCconvertedData_mAmper;
//	ADC_ChannelConfTypeDef sConfig ={0};
//	sConfig.Channel = ADC_CHANNEL_0;
//	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
//	sConfig.SamplingTime = ADC_SAMPLETIME_79CYCLES_5;
//	HAL_ADC_ConfigChannel(&hadc1,&sConfig);
	OutputOn(3000);
	Delay_ms(1000);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,10);
	rawValues =HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
//	sConfig.Channel = ADC_CHANNEL_0;
//	sConfig.Rank = ADC_RANK_NONE;
//	sConfig.SamplingTime = ADC_SAMPLETIME_79CYCLES_5;
//	HAL_ADC_ConfigChannel(&hadc1,&sConfig);
	ADCconvertedData_mVolt = (float)((rawValues) * (VREFANALOG_VOLTAGE) / DIGITAL_SCALE_12BITS);
	/*calculate the current from ADCconvertedData_mVolt(unit: mAmper) */
	ADCconvertedData_mAmper = (float)(ADCconvertedData_mVolt / IC_GAIN);
    return ADCconvertedData_mAmper;
}

/*-----------------------------------------------------------*/
/* --- Stop ADC Calculation and Switch off Mosfet ---*/
static void mosfetStopMeasurement(void) {
	OutputOff();
	HAL_ADC_Stop(&hadc1);
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
		t0 = HAL_GetTick();
				taskYIELD();
			}
}
/*-----------------------------------------------------------*/

/* --- Send measurement results --- */
static Module_Status SendMeasurementResult(uint8_t request, float value, uint8_t module,
		uint8_t port, float *Buffer) {

	Module_Status state = H0FR7_OK;
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

	    /* start software timer which will create event timeout */
	    /* Create a timeout timer */
		xTimerSwitch =xTimerCreate("SwitchTimer",pdMS_TO_TICKS(1000),pdFALSE,(void* )1,SwitchTimerCallback);
	    /* Start the timeout timer */
	    xTimerStart( xTimerSwitch, portMAX_DELAY );

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
/* Set Switch dutycycle */
Module_Status SetSwitchPWM(uint8_t dutycycle) {
	Module_Status result = H0FR7_OK;

	if(dutycycle >= 0 && dutycycle <= 100) {
	HAL_TIM_PWM_Start(SWITCHING_TIMER_HANDLE, SWITCHING_TIM_CH);
	TIM14->CCR1 = (PWM_TIMER_CLOCK_ARR * dutycycle) / 100;
	return result = H0FR7_OK;
	}
	else
	 return result = H0FR7_ERR_WrongParams;
}

/* -----------------------------------------------------------------------
 |								  APIs							          | 																 	|
/* -----------------------------------------------------------------------
 */
/* --- Turn on the solid state Switch ---
 */
Module_Status OutputOn(uint32_t timeout) {
	Module_Status result = H0FR7_OK;

	/* turn output on */
	SetSwitchPWM(DUTY_CYCLE_100_ON);

	/* Indicator LED */
	if (SwitchindMode)
		IND_ON();

	/* Timeout */
	if (timeout <= portMAX_DELAY) {
		/* Stop the timer if it's already running */
		if (xTimerIsTimerActive(xTimerSwitch))
			xTimerStop(xTimerSwitch, 100);
		/* Update timer timeout - This also restarts the timer */
		xTimerChangePeriod(xTimerSwitch, pdMS_TO_TICKS(timeout), 100);
	}

	/* Update Switch state */
	Switch_state = STATE_ON;
//	Switch_Oldstate = Switch_state;

	return result;
}

/*-----------------------------------------------------------*/
/* --- Turn off the solid state Switch ---
 */
Module_Status OutputOff(void) {
	Module_Status result = H0FR7_OK;

	/* Turn off PWM and re-initialize GPIO if needed */
	SetSwitchPWM(DUTY_CYCLE_0_OFF);

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
Module_Status OutputToggle(void) {
	Module_Status result = H0FR7_OK;

	if (Switch_state){
		OutputOff();
		result = STATE_OFF;}
	else{
		OutputOn(portMAX_DELAY);
		result = STATE_ON;}

	return result;
}

/*-----------------------------------------------------------*/
Module_Status OutputPWM(uint32_t dutyCycle) {
	Module_Status result = H0FR7_OK;

	if (dutyCycle < 0 || dutyCycle > 100)
		return H0FR7_ERR_WrongParams;

	/* Start the PWM */
	SetSwitchPWM(dutyCycle);

	/* Update Switch state */
	Switch_state =STATE_ON;
//	Switch_Oldstate = Switch_state;

	/* Indicator LED */
	if (SwitchindMode)
		IND_ON();

	return result;
}
/*-----------------------------------------------------------*/

/* @brief Reads the current value using the Analog Digital Converter (ADC).
 * @param[out] Current Pointer to a float where the measured current value will be stored (unit: mAmper)
 * @return Module_Status
 */
Module_Status SampleCurrentMeasurement(float* Current) {
	mosfetMode = REQ_SAMPLE;
	startMeasurement = START_MEASUREMENT;

	if (mosfetState == REQ_TIMEOUT) {
		*Current = 0;
	} else {
		*Current = Current_Calculation();
		mosfetState = REQ_IDLE;
		return H0FR7_OK;
	}
}

/*-----------------------------------------------------------*/
/* --- Stream measurements continuously to a port --- */
float Stream_current_To_Port(uint8_t Port, uint8_t Module, uint32_t Period,uint32_t Timeout) {
	float temp;
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
	temp = Current_Calculation();
	SendMeasurementResult(mosfetMode, temp, Module, Port, NULL);
	return (H0FR7_OK);
}
/*-----------------------------------------------------------*/

/* --- stream Current value to CLI
 */
float Stream_current_To_CLI(uint32_t Period, uint32_t Timeout) {
	float temp;
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
	temp = Current_Calculation();
	SendMeasurementResult(mosfetMode, temp, myID, PcPort, NULL);
	return (H0FR7_OK);
}
/*-----------------------------------------------------------*/

/* --- stream Current value from to CLI
 */
float Stream_current_To_CLI_V(uint32_t Period, uint32_t Timeout) {
	float temp;
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
	temp = Current_Calculation();
	SendMeasurementResult(mosfetMode, temp, myID, PcPort, NULL);
	return (H0FR7_OK);
}
/*-----------------------------------------------------------*/

float Stream_current_To_Buffer(float *Buffer, uint32_t Period, uint32_t Timeout)
{
	float temp;
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
	temp = Current_Calculation();
	SendMeasurementResult(mosfetMode, temp, 0, 0, Buffer);
	return (H0FR7_OK);
}

/*-----------------------------------------------------------*/

/* --- Stop Current measurement --- */
Module_Status Stop_current_measurement(void) {

	Module_Status state = H0FR7_OK;
	mosfetMode = REQ_IDLE;
	startMeasurement = STOP_MEASUREMENT;
	xTimerStop(xTimerSwitch, 0);

	mosfetStopMeasurement();

	return state;
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

	result = OutputOn(timeout);

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

	result = OutputOff();

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

	result = OutputToggle();

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
		result = OutputPWM(dutycycle);

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
portBASE_TYPE mosfetSampleCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	int8_t *pcParameterString1;
	portBASE_TYPE xParameterStringLength1 = 0;
	float Current = 0;
	static const int8_t *pcOKMessage = (int8_t*) "Current: %.3f Amp\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Obtain the value. */
	Current = Current_Calculation();
	mosfetCurrent = Current;

	/* Respond to the command */
	sprintf((char*) pcWriteBuffer, (char*) pcOKMessage, Current);

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

portBASE_TYPE mosfetStreamCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	static const int8_t *pcMessageBuffer =
			(int8_t*) "Streaming measurements to internal buffer. Access in the CLI using module parameter: current\n\r";
	static const int8_t *pcMessageModule =
			(int8_t*) "Streaming measurements to port P%d in module #%d\n\r";
	static const int8_t *pcMessageCLI =
			(int8_t*) "Streaming measurements to the CLI\n\n\r";
	static const int8_t *pcMessageError = (int8_t*) "Wrong parameter\r\n";
	static const int8_t *pcMessageStopMsg =
				(int8_t*) "Streaming stopped successfully\n\r";

	int8_t *pcParameterString1; /* period */
	int8_t *pcParameterString2; /* timeout */
	int8_t *pcParameterString3; /* port or buffer */
	int8_t *pcParameterString4; /* module */
	portBASE_TYPE xParameterStringLength1 = 0;
	portBASE_TYPE xParameterStringLength2 = 0;
	portBASE_TYPE xParameterStringLength3 = 0;
	portBASE_TYPE xParameterStringLength4 = 0;

	uint32_t Period = 0;
	uint32_t Timeout = 0;
	uint8_t Port = 0;
	uint8_t Module = 0;
	bool b;

	Module_Status result = H0FR7_OK;

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	/* Obtain the 1st parameter string: period */
	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1,
			&xParameterStringLength1);
	/* Obtain the 2nd parameter string: timeout */
	pcParameterString2 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 2,
			&xParameterStringLength2);
	/* Obtain the 3rd parameter string: port */
	pcParameterString3 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 3,
			&xParameterStringLength3);
	/* Obtain the 4th parameter string: module */
	pcParameterString4 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 4,
			&xParameterStringLength4);

	if (NULL != pcParameterString1) {
		Period = atoi((char*) pcParameterString1);
	} else {
		result = H0FR7_ERR_WrongParams;
	}
	if (NULL != pcParameterString2) {
		if (!strncmp((const char*) pcParameterString2, "inf", 3)) {
			Timeout = portMAX_DELAY;
		} else {
			Timeout = atoi((char*) pcParameterString2);
		}
	} else {
		result = H0FR7_ERR_WrongParams;
	}

	/* streaming data to internal buffer (module parameter) */
		if (NULL != pcParameterString3 && !strncmp((const char *)pcParameterString3, "buffer", 6))
		{
			strcpy(( char * ) pcWriteBuffer, ( char * ) pcMessageBuffer);

			Stream_current_To_Buffer(&mosfetBuffer, Period, Timeout);

			// Return right away here as we don't want to block the CLI
			return pdFALSE;
		}
	/* streaming data to port */
	else if (NULL != pcParameterString3 && NULL != pcParameterString4
			&& pcParameterString3[0] == 'p') {
		Port = (uint8_t) atol((char*) pcParameterString3 + 1);
		Module = atoi((char*) pcParameterString4);
		sprintf((char*) pcWriteBuffer, (char*) pcMessageModule, Port, Module);
		Stream_current_To_Port(Period, Timeout, Port, Module);
		// Return right away here as we don't want to block the CLI
		return pdFALSE;
	}
		/* Stream to the CLI */
			else if (NULL == pcParameterString3) {
				strcpy((char*) pcWriteBuffer, (char*) pcMessageCLI);
				writePxMutex(PcPort, (char*) pcWriteBuffer,
						strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
				Stream_current_To_CLI(Period, Timeout);

				/* Wait till the end of stream */
				while (startMeasurement != STOP_MEASUREMENT) {
					taskYIELD();
				}
			}

			else if (NULL != pcParameterString3
					&& !strncmp((const char*) pcParameterString3, "-v", 2)) {
				strcpy((char*) pcWriteBuffer, (char*) pcMessageCLI);
				writePxMutex(PcPort, (char*) pcWriteBuffer,
						strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
				Stream_current_To_CLI_V(Period, Timeout);

				/* Wait till the end of stream */
				while (startMeasurement != STOP_MEASUREMENT) {taskYIELD();}
			} else {
				result = H0FR7_ERR_WrongParams;
					}

	if (H0FR7_ERR_WrongParams == result) {
		strcpy((char*) pcWriteBuffer, (char*) pcMessageError);
	}

	Stop_current_measurement();

		if (stopB) {
			strcpy((char*) pcWriteBuffer, (char*) pcMessageStopMsg);
			writePxMutex(PcPort, (char*) pcWriteBuffer,
					strlen((char*) pcWriteBuffer), cmd50ms, HAL_MAX_DELAY);
			memset((char*) pcWriteBuffer, 0, strlen((char*) pcWriteBuffer));
			stopB = 0;
		} else {
			/* clean terminal output */
			memset((char*) pcWriteBuffer, 0, strlen((char*) pcWriteBuffer));
		}

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}
/*-----------------------------------------------------------*/

portBASE_TYPE mosfetStopCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status result = H0FR7_OK;
	static const int8_t *pcMessageOK =
			(int8_t*) "Streaming stopped successfully\r\n";
	static const int8_t *pcMessageError =
			(int8_t*) "Command failed! Please try again or reboot\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) pcCommandString;
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	result = Stop_current_measurement();

	if (H0FR7_OK == result) {
		strcpy((char*) pcWriteBuffer, (char*) pcMessageOK);
	} else {
		strcpy((char*) pcWriteBuffer, (char*) pcMessageError);
	}

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

portBASE_TYPE currentModParamCommand(int8_t *pcWriteBuffer,
		size_t xWriteBufferLen, const int8_t *pcCommandString) {
	static const int8_t *pcJoystickVerboseMsg = (int8_t*) "%.3f\r\n";

	/* Remove compile time warnings about unused parameters, and check the
	 write buffer is not NULL.  NOTE - for simplicity, this example assumes the
	 write buffer length is adequate, so does not check for buffer overflows. */
	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	sprintf((char*) pcWriteBuffer, (char*) pcJoystickVerboseMsg,
			mosfetBuffer);

	/* There is no more data to return after this single string, so return pdFALSE. */
	return pdFALSE;
}

/*-----------------------------------------------------------*/

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
