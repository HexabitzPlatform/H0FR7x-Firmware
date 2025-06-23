/*
 BitzOS (BOS) V0.4.0 - Copyright (C) 2017-2025 Hexabitz
 All rights reserved

 File Name  : H0FR7.c
 Description: H0FR7 power switch control main implementation.
 Components: UART ports, PWM timers, ADC input for current sensing.
 Functions: Output switching, PWM control, load current monitoring.
 */

/* Includes ****************************************************************/
#include "BOS.h"
#include "H0FR7_inputs.h"

/* Exported Typedef ********************************************************/
/* Define UART variables */
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart6;

TIM_HandleTypeDef htim3;

/* Private Variables *******************************************************/

uint16_t adcResult = 0u;
uint16_t adcBuffer[MOVING_AVG_SIZE] = { 0u }; // Circular buffer storing moving ADC samples
uint16_t sampleCount = 0u;                    // Number of samples collected so far
uint16_t bufferIndex = 0u;                    // Current index in the circular buffer
uint32_t sum = 0u;
uint32_t MovingAvg = 0u;
uint16_t ActualPWM_Frequency = 0;
float adcVoltagemV = 0.0f;
float LoadCurrent = 0.0f;

/* Global variables for sensor data used in ModuleParam */
float H0FR7_LoadCurrent = 0.0f;

/* Module Parameters */
ModuleParam_t ModuleParam[NUM_MODULE_PARAMS] = {
	{ .ParamPtr = &H0FR7_LoadCurrent,     .ParamFormat = FMT_FLOAT,   .ParamName = "current" },
};

/* Private Function Prototypes *********************************************/
void MX_TIM3_Init(void);
void Module_Peripheral_Init(void);
void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src, uint8_t dst, uint8_t inport, uint8_t outport);
uint8_t ClearROtopology(void);
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift);

/* Local Function Prototypes ***********************************************/
uint16_t MovingAverage(uint16_t adcNewValue);
Module_Status CalculateLoadCurrent(float *Current);
Module_Status SwitchControlPWM(uint8_t dutycycle,uint32_t freq);

/* Create CLI commands *****************************************************/
portBASE_TYPE CLI_Output_Turn_ONCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
portBASE_TYPE CLI_Output_Turn_OFFCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
portBASE_TYPE CLI_Output_PWMCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);
portBASE_TYPE CLI_Get_CurrentCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString);

/* CLI command structure ***************************************************/
/* CLI command structure : OutputTurnOn */
const CLI_Command_Definition_t CLI_Output_Turn_ONCommandDefinition = { (const int8_t*) "turn_on", /* The command string to type. */
(const int8_t*) "turn_on:\r\nTurn on the output by turning the switch fully ON \n\r", CLI_Output_Turn_ONCommand, /* The function to run. */
0 /* zero parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : OutputTurnOff */
const CLI_Command_Definition_t CLI_Output_Turn_OFFCommandDefinition = { (const int8_t*) "turn_off", /* The command string to type. */
(const int8_t*) "turn_off:\r\nTurn off the output by turning the switch fully OFF \r\n",
CLI_Output_Turn_OFFCommand, /* The function to run. */
0 /* zero parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : OutputPWM */
const CLI_Command_Definition_t CLI_Output_PWMCommandDefinition =
{ (const int8_t*) "turn_pwm", /* The command string to type. */
(const int8_t*) "turn_pwm:\r\nParameters required to execute a OutputPWM:\n\r 1)dutyCycle: PWM duty cycle in precentage (0 to 100)% \n\r 2)Freq:Desired PWM signal frequency in Hz. Must be > 0 and < 30000.\n\r",
CLI_Output_PWMCommand, /* The function to run. */
2 /* tow parameters are expected. */
};

/***************************************************************************/
/* CLI command structure : GetLoadCurrent */
const CLI_Command_Definition_t CLI_Get_CurrentCommandDefinition =
{ (const int8_t*) "get_current", /* The command string to type. */
(const int8_t*) "get_current:\r\nParameters required to execute a GetLoadCurrent:\n\r 1)dutyCycle: PWM duty cycle in precentage (0 to 100)% \n\r",
CLI_Get_CurrentCommand, /* The function to run. */
0 /* zero parameters are expected. */
};

/***************************************************************************/
/************************ Private function Definitions *********************/
/***************************************************************************/
/* @brief  System Clock Configuration
 *         This function configures the system clock as follows:
 *            - System Clock source            = PLL (HSE)
 *            - SYSCLK(Hz)                     = 64000000
 *            - HCLK(Hz)                       = 64000000
 *            - AHB Prescaler                  = 1
 *            - APB1 Prescaler                 = 1
 *            - HSE Frequency(Hz)              = 8000000
 *            - PLLM                           = 1
 *            - PLLN                           = 16
 *            - PLLP                           = 2
 *            - Flash Latency(WS)              = 2
 *            - Clock Source for UART1,UART2,UART3 = 16MHz (HSI)
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE; // Enable both HSI and HSE oscillators
	RCC_OscInitStruct.HSEState = RCC_HSE_ON; // Enable HSE (External High-Speed Oscillator)
	RCC_OscInitStruct.HSIState = RCC_HSI_ON; // Enable HSI (Internal High-Speed Oscillator)
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1; // No division on HSI
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT; // Default calibration value for HSI
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON; // Enable PLL
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE; // Set PLL source to HSE
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1; // Prescaler for PLL input
	RCC_OscInitStruct.PLL.PLLN = 16; // Multiplication factor for PLL
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2; // PLLP division factor
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2; // PLLQ division factor
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2; // PLLR division factor
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	/** Initializes the CPU, AHB and APB buses clocks */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK; // Select PLL as the system clock source
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1; // AHB Prescaler set to 1
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1; // APB1 Prescaler set to 1

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2); // Configure system clocks with flash latency of 2 WS
}

/***************************************************************************/
/* enable stop mode regarding only UART1 , UART2 , and UART3 */
BOS_Status EnableStopModebyUARTx(uint8_t port) {

	UART_WakeUpTypeDef WakeUpSelection;
	UART_HandleTypeDef *huart = GetUart(port);

	if ((huart->Instance == USART1) || (huart->Instance == USART2) || (huart->Instance == USART3)) {

		/* make sure that no UART transfer is on-going */
		while (__HAL_UART_GET_FLAG(huart, USART_ISR_BUSY) == SET)
			;

		/* make sure that UART is ready to receive */
		while (__HAL_UART_GET_FLAG(huart, USART_ISR_REACK) == RESET)
			;

		/* set the wake-up event:
		 * specify wake-up on start-bit detection */
		WakeUpSelection.WakeUpEvent = UART_WAKEUP_ON_STARTBIT;
		HAL_UARTEx_StopModeWakeUpSourceConfig(huart, WakeUpSelection);

		/* Enable the UART Wake UP from stop mode Interrupt */
		__HAL_UART_ENABLE_IT(huart, UART_IT_WUF);

		/* enable MCU wake-up by LPUART */
		HAL_UARTEx_EnableStopMode(huart);

		/* enter STOP mode */
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	} else
		return BOS_ERROR;

}

/***************************************************************************/
/* Enable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status EnableStandbyModebyWakeupPinx(WakeupPins_t wakeupPins) {

	/* Clear the WUF FLAG */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

	/* Enable the WAKEUP PIN */
	switch (wakeupPins) {

	case PA0_PIN:
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
		break;

	case PA2_PIN:
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
		break;

	case PB5_PIN:
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
		break;

	case PC13_PIN:
		HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
		break;

	case NRST_PIN:
		/* do no thing*/
		break;
	}

	/* Enable SRAM content retention in Standby mode */
	HAL_PWREx_EnableSRAMRetention();

	/* Finally enter the standby mode */
	HAL_PWR_EnterSTANDBYMode();

	return BOS_OK;
}

/***************************************************************************/
/* Disable standby mode regarding wake-up pins:
 * WKUP1: PA0  pin
 * WKUP4: PA2  pin
 * WKUP6: PB5  pin
 * WKUP2: PC13 pin
 * NRST pin
 *  */
BOS_Status DisableStandbyModeWakeupPinx(WakeupPins_t wakeupPins) {

	/* The standby wake-up is same as a system RESET:
	 * The entire code runs from the beginning just as if it was a RESET.
	 * The only difference between a reset and a STANDBY wake-up is that, when the MCU wakes-up,
	 * The SBF status flag in the PWR power control/status register (PWR_CSR) is set */
	if (__HAL_PWR_GET_FLAG(PWR_FLAG_SB) != RESET) {
		/* clear the flag */
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);

		/* Disable  Wake-up Pinx */
		switch (wakeupPins) {

		case PA0_PIN:
			HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1); /* PA0 */
			break;

		case PA2_PIN:
			HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN4); /* PA2 */
			break;

		case PB5_PIN:
			HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN6); /* PB5 */
			break;

		case PC13_PIN:
			HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN2); /* PC13 */
			break;

		case NRST_PIN:
			/* do no thing*/
			break;
		}

		IND_blink(1000);

	} else
		return BOS_OK;

}

/***************************************************************************/
/* Save Command Topology in Flash RO */
uint8_t SaveTopologyToRO(void) {

	HAL_StatusTypeDef flashStatus = HAL_OK;

	/* flashAdd is initialized with 8 because the first memory room in topology page
	 * is reserved for module's ID */
	uint16_t flashAdd = 8;
	uint16_t temp = 0;

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();

	/* Erase Topology page */
	FLASH_PageErase(FLASH_BANK_2, TOPOLOGY_PAGE_NUM);

	/* Wait for an Erase operation to complete */
	flashStatus = FLASH_WaitForLastOperation((uint32_t) HAL_FLASH_TIMEOUT_VALUE);

	if (flashStatus != HAL_OK) {
		/* return FLASH error code */
		return pFlash.ErrorCode;
	}

	else {
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	}

	/* Save module's ID and topology */
	if (myID) {

		/* Save module's ID */
		temp = (uint16_t) (N << 8) + myID;

		/* Save module's ID in Flash memory */
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, TOPOLOGY_START_ADDRESS, temp);

		/* Wait for a Write operation to complete */
		flashStatus = FLASH_WaitForLastOperation((uint32_t) HAL_FLASH_TIMEOUT_VALUE);

		if (flashStatus != HAL_OK) {
			/* return FLASH error code */
			return pFlash.ErrorCode;
		}

		else {
			/* If the program operation is completed, disable the PG Bit */
			CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
		}

		/* Save topology */
		for (uint8_t row = 1; row <= N; row++) {
			for (uint8_t column = 0; column <= MAX_NUM_OF_PORTS; column++) {
				/* Check the module serial number
				 * Note: there isn't a module has serial number 0
				 */
				if (Array[row - 1][0]) {
					/* Save each element in topology Array in Flash memory */
					HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, TOPOLOGY_START_ADDRESS + flashAdd,
							Array[row - 1][column]);
					/* Wait for a Write operation to complete */
					flashStatus = FLASH_WaitForLastOperation((uint32_t) HAL_FLASH_TIMEOUT_VALUE);
					if (flashStatus != HAL_OK) {
						/* return FLASH error code */
						return pFlash.ErrorCode;
					} else {
						/* If the program operation is completed, disable the PG Bit */
						CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
						/* update new flash memory address */
						flashAdd += 8;
					}
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Save Command Snippets in Flash RO */
uint8_t SaveSnippetsToRO(void) {
	HAL_StatusTypeDef FlashStatus = HAL_OK;
	uint8_t snipBuffer[sizeof(Snippet_t) + 1] = { 0 };

	/* Unlock the FLASH control register access */
	HAL_FLASH_Unlock();
	/* Erase Snippets page */
	FLASH_PageErase(FLASH_BANK_2, SNIPPETS_PAGE_NUM);
	/* Wait for an Erase operation to complete */
	FlashStatus = FLASH_WaitForLastOperation((uint32_t) HAL_FLASH_TIMEOUT_VALUE);

	if (FlashStatus != HAL_OK) {
		/* return FLASH error code */
		return pFlash.ErrorCode;
	} else {
		/* Operation is completed, disable the PER Bit */
		CLEAR_BIT(FLASH->CR, FLASH_CR_PER);
	}

	/* Save Command Snippets */
	int currentAdd = SNIPPETS_START_ADDRESS;
	for (uint8_t index = 0; index < NumOfRecordedSnippets; index++) {
		/* Check if Snippet condition is true or false */
		if (Snippets[index].Condition.ConditionType) {
			/* A marker to separate Snippets */
			snipBuffer[0] = 0xFE;
			memcpy((uint32_t*) &snipBuffer[1], (uint8_t*) &Snippets[index], sizeof(Snippet_t));
			/* Copy the snippet struct buffer (20 x NumOfRecordedSnippets). Note this is assuming sizeof(Snippet_t) is even */
			for (uint8_t j = 0; j < (sizeof(Snippet_t) / 4); j++) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAdd, *(uint64_t*) &snipBuffer[j * 8]);
				FlashStatus = FLASH_WaitForLastOperation((uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 8;
				}
			}
			/* Copy the snippet commands buffer. Always an even number. Note the string termination char might be skipped */
			for (uint8_t j = 0; j < ((strlen(Snippets[index].CMD) + 1) / 4); j++) {
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, currentAdd, *(uint64_t*) (Snippets[index].CMD + j * 4));
				FlashStatus = FLASH_WaitForLastOperation((uint32_t) HAL_FLASH_TIMEOUT_VALUE);
				if (FlashStatus != HAL_OK) {
					return pFlash.ErrorCode;
				} else {
					/* If the program operation is completed, disable the PG Bit */
					CLEAR_BIT(FLASH->CR, FLASH_CR_PG);
					currentAdd += 8;
				}
			}
		}
	}
	/* Lock the FLASH control register access */
	HAL_FLASH_Lock();
}

/***************************************************************************/
/* Clear Array topology in SRAM and Flash RO */
uint8_t ClearROtopology(void) {
	/* Clear the Array */
	memset(Array, 0, sizeof(Array));
	N = 1;
	myID = 0;

	return SaveTopologyToRO();
}

/***************************************************************************/
/* Trigger ST factory bootloader update for a remote module */
void RemoteBootloaderUpdate(uint8_t src, uint8_t dst, uint8_t inport, uint8_t outport) {

	uint8_t myOutport = 0, lastModule = 0;
	int8_t *pcOutputString;

	/* 1. Get Route to destination module */
	myOutport = FindRoute(myID, dst);
	if (outport && dst == myID) { /* This is a 'via port' update and I'm the last module */
		myOutport = outport;
		lastModule = myID;
	} else if (outport == 0) { /* This is a remote update */
		if (NumberOfHops(dst)== 1)
		lastModule = myID;
		else
		lastModule = Route[NumberOfHops(dst)-1]; /* previous module = Route[Number of hops - 1] */
	}

	/* 2. If this is the source of the message, show status on the CLI */
	if (src == myID) {
		/* Obtain the address of the output buffer.  Note there is no mutual
		 * exclusion on this buffer as it is assumed only one command console
		 * interface will be used at any one time. */
		pcOutputString = FreeRTOS_CLIGetOutputBuffer();

		if (outport == 0)		// This is a remote module update
			sprintf((char*) pcOutputString, pcRemoteBootloaderUpdateMessage, dst);
		else
			// This is a 'via port' remote update
			sprintf((char*) pcOutputString, pcRemoteBootloaderUpdateViaPortMessage, dst, outport);

		strcat((char*) pcOutputString, pcRemoteBootloaderUpdateWarningMessage);
		writePxITMutex(inport, (char*) pcOutputString, strlen((char*) pcOutputString), cmd50ms);
		Delay_ms(100);
	}

	/* 3. Setup my inport and outport for bootloader update */
	SetupPortForRemoteBootloaderUpdate(inport);
	SetupPortForRemoteBootloaderUpdate(myOutport);

	/* 5. Build a DMA stream between my inport and outport */
	StartScastDMAStream(inport, myID, myOutport, myID, BIDIRECTIONAL, 0xFFFFFFFF, 0xFFFFFFFF, false);
}

/***************************************************************************/
/* Setup a port for remote ST factory bootloader update:
 * Set baudrate to 57600
 * Enable even parity
 * Set datasize to 9 bits
 */
void SetupPortForRemoteBootloaderUpdate(uint8_t port) {

	UART_HandleTypeDef *huart = GetUart(port);
	HAL_UART_DeInit(huart);
	huart->Init.Parity = UART_PARITY_EVEN;
	huart->Init.WordLength = UART_WORDLENGTH_9B;
	HAL_UART_Init(huart);

	/* The CLI port RXNE interrupt might be disabled so enable here again to be sure */
	__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);

}

/***************************************************************************/
/* H0FR7 module initialization */
void Module_Peripheral_Init(void) {
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/* Array ports */
	MX_USART1_UART_Init();
	MX_USART2_UART_Init();
	MX_USART3_UART_Init();
	MX_USART5_UART_Init();
	MX_USART6_UART_Init();

	/* MOSFET Timer Init */
	MX_TIM3_Init();
	/* ADC Init */
	MX_ADC_Init();
	/* Start ADC1 in DMA mode to continuously read one value into adcResult */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &adcResult, 1);

	/* Circulating DMA Channels ON All Module */
	for (int i = 1; i <= NUM_OF_PORTS; i++) {
		if (GetUart(i) == &huart1) {
			dmaIndex[i - 1] = &(DMA1_Channel1->CNDTR);
		} else if (GetUart(i) == &huart2) {
			dmaIndex[i - 1] = &(DMA1_Channel2->CNDTR);
		} else if (GetUart(i) == &huart3) {
			dmaIndex[i - 1] = &(DMA1_Channel3->CNDTR);
		} else if (GetUart(i) == &huart4) {
			dmaIndex[i - 1] = &(DMA1_Channel4->CNDTR);
		} else if (GetUart(i) == &huart5) {
			dmaIndex[i - 1] = &(DMA1_Channel5->CNDTR);
		} else if (GetUart(i) == &huart6) {
			dmaIndex[i - 1] = &(DMA1_Channel6->CNDTR);
		}
	}

}

/***************************************************************************/
/* H0FR7 message processing task */
Module_Status Module_MessagingTask(uint16_t code, uint8_t port, uint8_t src, uint8_t dst, uint8_t shift) {
	Module_Status result = H0FR7_OK;
	uint8_t DutyCycle = 0;
    uint16_t Freq = 0;
	switch (code) {

	case CODE_H0FR7_ON:
		OutputTurnOn();
		break;

	case CODE_H0FR7_OFF:
		OutputTurnOff();
		break;

	case CODE_H0FR7_PWM:
		DutyCycle = (uint8_t) cMessage[port - 1][shift];
		Freq = (uint16_t) cMessage[port - 1][1 + shift] + ((uint16_t) cMessage[port - 1][2 + shift] << 8);
		OutputPWM(DutyCycle,Freq);
		break;

	default:
		result = H0FR7_ERR_UNKNOWNMESSAGE;
		break;
	}

	return result;
}
/***************************************************************************/
/* Get the port for a given UART */
uint8_t GetPort(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART6)
		return P1;
	else if (huart->Instance == USART2)
		return P2;
	else if (huart->Instance == USART3)
		return P3;
	else if (huart->Instance == USART1)
		return P4;
	else if (huart->Instance == USART5)
		return P5;
	else if (huart->Instance == USART4)
		return P6;

	return 0;
}

/***************************************************************************/
/* Register this module CLI Commands */
void RegisterModuleCLICommands(void) {
	FreeRTOS_CLIRegisterCommand(&CLI_Output_Turn_ONCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_Output_Turn_OFFCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_Output_PWMCommandDefinition);
	FreeRTOS_CLIRegisterCommand(&CLI_Get_CurrentCommandDefinition);

}

/***************************************************************************/
/* Samples a module parameter value based on parameter index.
 * paramIndex: Index of the parameter (1-based index).
 * value: Pointer to store the sampled float value.
 */
Module_Status GetModuleParameter(uint8_t paramIndex, float *value) {
	Module_Status status = H0FR7_OK;

	switch (paramIndex) {

	case 1: {

		float temp = 0.0f;
		status =  GetLoadCurrent(&temp);
		if (status == H0FR7_OK)
			*value = (float) temp;
		break;
	}

	/* Invalid parameter index */
	default:
		status = BOS_ERR_WrongParam;
		break;
	}

	return status;
}

/***************************************************************************/
/****************************** Local Functions ****************************/
/***************************************************************************/
/* Calculates a moving average of the converted ADC samples.
 * @adcNewValue: The latest ADC sample after conversion/scaling.
 */
uint16_t MovingAverage(uint16_t adcNewValue) {

	/* Remove oldest sample from sum if buffer is full */
	if (sampleCount >= MOVING_AVG_SIZE) {
		sum -= adcBuffer[bufferIndex];
	} else {
		sampleCount++;
	}

	/* Add new sample to buffer and update sum */
	adcBuffer[bufferIndex] = adcNewValue;
	sum += adcNewValue;

	/* Advance buffer index circularly */
	bufferIndex = (bufferIndex + 1) % MOVING_AVG_SIZE;

	/* Return average of samples collected so far */
	return (uint16_t) (sum / sampleCount);
}

/***************************************************************************/
/* Callback function called when ADC conversion is complete to updates the current reading using the moving average filter*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	MovingAvg = MovingAverage(adcResult);
}

/***************************************************************************/
/* Sets PWM duty cycle to control the switch (MOSFET).
 * dutycycle: The desired PWM duty cycle (0 to 100).
 */
Module_Status SwitchControlPWM(uint8_t dutycycle, uint32_t freq) {
	Module_Status status = H0FR7_OK;

	/* Get the timer base clock frequency (use appropriate function based on timer used) */
	uint32_t timerClock = HAL_RCC_GetPCLK1Freq();

	if ((freq < 0 || freq >= 30000))
		return H0FR7_ERR_WRONGPARAMS;

	/* Validate input parameters */
	if (dutycycle >= 0 && dutycycle <= 100) {
		uint32_t prescaler = 1;
		uint32_t arr = timerClock / freq;

		/* Check if ARR exceeds 16-bit limit */
		if (arr > 0xFFFF) {
			prescaler = (arr / 0xFFFF) + 1;
			arr = timerClock / (freq * prescaler);
		}

		/* Apply calculated prescaler and ARR values to timer registers */
		SWITCH_CONTROL_PSC = prescaler - 1;
		SWITCH_CONTROL_ARR = arr - 1;

		/* Calculate the CCR value based on desired duty cycle (0–100%) */
		SWITCH_CONTROL_CCR = ((float) dutycycle / 100.0f) * arr;

		/* Calculate and store actual PWM frequency */
		ActualPWM_Frequency = (float) timerClock / ((SWITCH_CONTROL_PSC + 1) * (SWITCH_CONTROL_ARR + 1));

		/* Start PWM signal generation */
		HAL_TIM_PWM_Start(SWITCH_CONTROL_TIM_HANDLE, SWITCH_CONTROL_TIM_CH);
	} else {

		status = H0FR7_ERR_WRONGPARAMS;
	}

	return status;
}
/***************************************************************************/
/* Calculates the load current using filtered ADC readings.
 * Current: Pointer to store the calculated current in mA. */
Module_Status CalculateLoadCurrent(float *Current) {

	Module_Status status = H0FR7_OK;

	if (Current == NULL)
		return H0FR7_ERR_WRONGPARAMS;

	/* Start ADC calibration and conversion */
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_IT(&hadc1);

	/* Convert the filtered ADC value to voltage in millivolts (applying step size and offset correction) */
	adcVoltagemV = (float) ((MovingAvg * ADC_STEP_MV) - (CURRENT_SENSE_OFFSET));

	/* Calculate the load current using the current sensing gain factor */
	*Current = (adcVoltagemV / CURRENT_SENSE_GAIN);

	/* Stop ADC */
	HAL_ADC_Stop_IT(&hadc1);

	return status;
}

/***************************************************************************/

/***************************************************************************/
/***************************** General Functions ***************************/
/***************************************************************************/
/* Turn on the output by setting the switch fully ON using 100% PWM duty cycle. */
Module_Status OutputTurnOn(void) {
	Module_Status status = H0FR7_OK;

	/* Set PWM to 100%. Since the output remains constantly HIGH at 100% duty cycle, the frequency parameter (second argument) has no practical effect. */
	SwitchControlPWM(PWM_DUTY_CYCLE_FULL,1);
	/* Measured the current flowing through the load and store it in Load Current. */
	CalculateLoadCurrent(&LoadCurrent);
	/* Apply a fixed offset to the measured current to improve accuracy */
	LoadCurrent = LoadCurrent + I_OFFSET;

	return status;
}

/***************************************************************************/
/* Turn off the output by setting the switch fully OFF using 0% PWM duty cycle. */
Module_Status OutputTurnOff(void) {
	Module_Status status = H0FR7_OK;

	/* Set PWM 0 % Since the output remains constantly LOW at 0% duty cycle, the frequency parameter (second argument) has no practical effect. */
	SwitchControlPWM(PWM_DUTY_CYCLE_OFF,1);
	/* Manually set the load current to zero, since the output is turned off and no current is expected. */
	LoadCurrent = 0;

	return status;
}

/***************************************************************************/
/* Set the PWM output to a specific duty cycle and frequency.
 * dutyCycle: Desired PWM dutycycle percentage (0–100).
 * Freq:Desired PWM signal frequency in Hz. Must be > 0 and < 30000.
 */
Module_Status OutputPWM(uint8_t DutyCycle, uint16_t Freq) {
	Module_Status status = H0FR7_OK;

	if (DutyCycle < 0 || DutyCycle > 100)
		return H0FR7_ERR_WRONGPARAMS;

	/* update the PWM output with the given duty cycle and frequency. */
	SwitchControlPWM(DutyCycle,Freq);
	/* Measure current through the load. */
	CalculateLoadCurrent(&LoadCurrent);

	if (DutyCycle >= 25) {
	/* If duty cycle is sufficiently high, apply an offset for better accuracy. */
		LoadCurrent = LoadCurrent + I_OFFSET;
	}

	return status;
}

/***************************************************************************/
/* Retrieves the current value from the global LoadCurrent variable.
 * CurrentOut: Pointer to store the calculated current in mA.
 */
Module_Status GetLoadCurrent(float *CurrentOut) {
	Module_Status status = H0FR7_OK;

	if (CurrentOut == NULL)
		return H0FR7_ERR_WRONGPARAMS;

	/* Return the global LoadCurrent value */
	*CurrentOut = LoadCurrent;

	return status;
}

/***************************************************************************/
/********************************* Commands ********************************/
/***************************************************************************/
portBASE_TYPE CLI_Output_Turn_ONCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H0FR7_OK;

	static const int8_t *pcOKMessage = (int8_t*) "The output has been successfully turned on\r\n";

	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	status = OutputTurnOn();

	/* Respond to the command */
	if (status == H0FR7_OK) {
		strcpy((char*) pcWriteBuffer, (char*) pcOKMessage);
	}

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/***************************************************************************/
portBASE_TYPE CLI_Output_Turn_OFFCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H0FR7_OK;

	static const int8_t *pcOKMessage = (int8_t*) "The output has been successfully turned off\r\n";

	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	status = OutputTurnOff();

	/* Respond to the command */
	if (status == H0FR7_OK) {
		strcpy((char*) pcWriteBuffer, (char*) pcOKMessage);
	}

	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/***************************************************************************/
portBASE_TYPE CLI_Output_PWMCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H0FR7_OK;

	uint8_t DutyCycle;
    uint16_t Freq;
	portBASE_TYPE xParameterStringLength1 = 0;
	portBASE_TYPE xParameterStringLength2 = 0;
	static int8_t *pcParameterString1;
	static int8_t *pcParameterString2;

	static const int8_t *pcOKMessage = (int8_t*) "The output is running PWM in duty cycle %d%% percent and a signal frequency of %d Hz\r\n";
	static const int8_t *pcWrongDutyCycleMessage = (int8_t*) "WrongDutyCycle!\n\r";

	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);

	pcParameterString1 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength1);
	DutyCycle = (uint8_t) atol((char*) pcParameterString1);

	pcParameterString2 = (int8_t*) FreeRTOS_CLIGetParameter(pcCommandString, 1, &xParameterStringLength2);
	Freq = (uint8_t) atol((char*) pcParameterString2);
	status = OutputPWM(DutyCycle,Freq);

	/* Respond to the command */
	if (status == H0FR7_OK) {
		sprintf((char*) pcWriteBuffer, (char*) pcOKMessage, DutyCycle ,Freq);
	} else if (status == H0FR7_ERR_WRONGDUTYCYCLE) {
		strcpy((char*) pcWriteBuffer, (char*) pcWrongDutyCycleMessage);
	}
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/***************************************************************************/
portBASE_TYPE CLI_Get_CurrentCommand(int8_t *pcWriteBuffer, size_t xWriteBufferLen, const int8_t *pcCommandString) {
	Module_Status status = H0FR7_OK;

	float LoadCurrent;

	static const int8_t *pcOKMessage = (int8_t*) "Load current: %.2f mA \r\n";
	static const int8_t *pcWrongDutyCycleMessage = (int8_t*) "WrongDutyCycle!\n\r";

	(void) xWriteBufferLen;
	configASSERT(pcWriteBuffer);
	for(uint8_t count = 0 ;count <255 ; count++){
		CalculateLoadCurrent(&LoadCurrent);
	}
	status = CalculateLoadCurrent(&LoadCurrent);
	LoadCurrent = LoadCurrent + I_OFFSET;
	/* Respond to the command */
	if (status == H0FR7_OK) {
		sprintf((char*) pcWriteBuffer, (char*) pcOKMessage, LoadCurrent);

	} else if (status == H0FR7_ERR_WRONGDUTYCYCLE) {
		strcpy((char*) pcWriteBuffer, (char*) pcWrongDutyCycleMessage);
	}
	/* There is no more data to return after this single string, so return
	 pdFALSE. */
	return pdFALSE;
}

/***************************************************************************/
/***************** (C) COPYRIGHT HEXABITZ ***** END OF FILE ****************/
