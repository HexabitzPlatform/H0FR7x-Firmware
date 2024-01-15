/*
 BitzOS (BOS) V0.2.9 - Copyright (C) 2017-2023 Hexabitz
 All rights reserved
 
 File Name     : H0FR7.h
 Description   : Header file for module H0FR7.
 	 	 	 	 (Description_of_module)

(Description of Special module peripheral configuration):
>>
>>
>>

 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef H0FR7_H
#define H0FR7_H

/* Includes ------------------------------------------------------------------*/
#include "BOS.h"
#include "H0FR7_MemoryMap.h"
#include "H0FR7_uart.h"
#include "H0FR7_gpio.h"
#include "H0FR7_dma.h"
#include "H0FR7_inputs.h"
#include "H0FR7_eeprom.h"
/* Exported definitions -------------------------------------------------------*/

#define	modulePN		_H0FR7


/* Port-related definitions */
#define	NumOfPorts			5

#define P_PROG 				P2						/* ST factory bootloader UART */

/* Define available ports */
#define _P1 
#define _P2 
#define _P3 
#define _P4 
#define _P5 


/* Define available USARTs */

#define _Usart2 1
#define _Usart3 1
#define _Usart4 1
#define _Usart5 1
#define _Usart6	1


/* Port-UART mapping */
#define P1uart &huart3
#define P2uart &huart1
#define P3uart &huart5
#define P4uart &huart6
#define P5uart &huart2

/* Port Definitions */
#define	USART1_TX_PIN		GPIO_PIN_10
#define	USART1_RX_PIN		GPIO_PIN_9
#define	USART1_TX_PORT		GPIOA
#define	USART1_RX_PORT		GPIOA
#define	USART1_AF			GPIO_AF1_USART1

#define	USART2_TX_PIN		GPIO_PIN_2
#define	USART2_RX_PIN		GPIO_PIN_3
#define	USART2_TX_PORT		GPIOA
#define	USART2_RX_PORT		GPIOA
#define	USART2_AF			GPIO_AF1_USART2

#define	USART3_TX_PIN		GPIO_PIN_10
#define	USART3_RX_PIN		GPIO_PIN_11
#define	USART3_TX_PORT		GPIOB
#define	USART3_RX_PORT		GPIOB
#define	USART3_AF			GPIO_AF4_USART3

#define	USART4_TX_PIN		GPIO_PIN_0
#define	USART4_RX_PIN		GPIO_PIN_1
#define	USART4_TX_PORT		GPIOA
#define	USART4_RX_PORT		GPIOA
#define	USART4_AF			GPIO_AF4_USART4

#define	USART5_TX_PIN		GPIO_PIN_3
#define	USART5_RX_PIN		GPIO_PIN_2
#define	USART5_TX_PORT		GPIOD
#define	USART5_RX_PORT		GPIOD
#define	USART5_AF			GPIO_AF3_USART5

#define	USART6_TX_PIN		GPIO_PIN_8
#define	USART6_RX_PIN		GPIO_PIN_9
#define	USART6_TX_PORT		GPIOB
#define	USART6_RX_PORT		GPIOB
#define	USART6_AF			GPIO_AF8_USART6


#define	_Switch_PIN						GPIO_PIN_6
#define	_Switch_PORT					GPIOB
#define _Switch_TIM_CH					TIM_CHANNEL_3
#define _Switch_GPIO_CLK()				__GPIOB_CLK_ENABLE();
#define PWM_TIMER_CLOCK					16000000
#define Switch_PWM_DEF_FREQ				10000
#define Switch_PWM_DEF_PERIOD			((float) (1/Switch_PWM_FREQ) )

#define ADC_CONVERSION 				  0.0058

#define MOSFET_DEFAULT_MAX_LOOP       2000

#define STOP_MEASUREMENT      		  0
#define START_MEASUREMENT     		  1


#define REQ_IDLE               		  0
#define REQ_SAMPLE_BUFFER		      1
#define REQ_SAMPLE_PORT				  2
#define REQ_SAMPLE_CLI                3
#define REQ_SAMPLE_VERBOSE_CLI		  4
#define REQ_STREAM_PORT_CLI           5
#define REQ_STREAM_VERBOSE_PORT_CLI   6
#define REQ_STREAM_PORT		          7
#define REQ_STREAM_BUFFER         	  8
#define REQ_TIMEOUT             	  9
#define REQ_MEASUREMENT_READY         10
#define REQ_TIMEOUT_CLI				  11
#define REQ_TIMEOUT_VERBOSE_CLI		  12
#define REQ_TIMEOUT_BUFFER			  13
#define REQ_STOP					  14
#define REQ_SAMPLE					  15

#define TIMERID_TIMEOUT_MEASUREMENT   0xFF

/* Macros define Mosfet running mode */
#define MOSFET_MODE_SINGLE            0x00
#define MOSFET_MODE_CONTINUOUS        0x01
#define MOSFET_MODE_CONTINUOUS_TIMED  0x02

/* Module-specific Definitions */

#define NUM_MODULE_PARAMS						7
#define STOP_MEASUREMENT_RANGING      0
#define START_MEASUREMENT_RANGING     1
/* Module EEPROM Variables */

// Module Addressing Space 500 - 599
#define _EE_MODULE							500		

/* Module_Status Type Definition */
typedef enum {
	H0FR7_OK = 0,
	H0FR7_ERR_UnknownMessage = 1,
	H0FR7_ERR_Wrong_Value = 2,
	H0FR7_ERR_Timeout,
	H0FR7_ERR_WrongParams,
	H0FR7_STOPED,
	H0FR7_ERROR = 255
} Module_Status;

/* Switch_state_t Type Definition */
typedef enum  {
	STATE_OFF,
	STATE_ON,
	STATE_PWM
} Switch_state_t;

/* Indicator LED */
#define _IND_LED_PORT			GPIOB
#define _IND_LED_PIN			GPIO_PIN_13

/* Export UART variables */
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart6;

/* Define UART Init prototypes */
extern void MX_USART1_UART_Init(void);
extern void MX_USART2_UART_Init(void);
extern void MX_USART3_UART_Init(void);
extern void MX_USART4_UART_Init(void);
extern void MX_USART5_UART_Init(void);
extern void MX_USART6_UART_Init(void);
extern void SystemClock_Config(void);
extern void ExecuteMonitor(void);

extern Switch_state_t Switch_State;
extern uint8_t SwitchindMode;
extern TIM_HandleTypeDef htim1;

void initialValue(void);

/* -----------------------------------------------------------------------
 |								  APIs							          |  																 	|
/* -----------------------------------------------------------------------
 */
extern Module_Status Output_on(uint32_t timeout);
extern Module_Status Output_off(void);
extern Module_Status Output_toggle(void);
extern Module_Status Output_PWM(float dutyCycle);
extern float Sample_current_measurement(void);
extern float Stream_current_To_Port(uint8_t Port, uint8_t Module, uint32_t Period, uint32_t Timeout);
extern float Stream_current_To_CLI_V(uint32_t Period, uint32_t Timeout);
extern float Stream_current_To_CLI(uint32_t Period, uint32_t Timeout);
extern float Stream_current_To_Buffer(float *Buffer, uint32_t Period, uint32_t Timeout);
extern Module_Status Stop_current_measurement(void);


void SetupPortForRemoteBootloaderUpdate(uint8_t port);
void remoteBootloaderUpdate(uint8_t src,uint8_t dst,uint8_t inport,uint8_t outport);

/* -----------------------------------------------------------------------
 |								Commands							      |															 	|
/* -----------------------------------------------------------------------
 */
extern const CLI_Command_Definition_t onCommandDefinition;
extern const CLI_Command_Definition_t offCommandDefinition;
extern const CLI_Command_Definition_t toggleCommandDefinition;
extern const CLI_Command_Definition_t ledModeCommandDefinition;
extern const CLI_Command_Definition_t pwmCommandDefinition;
extern const CLI_Command_Definition_t mosfetSampleCommandDefinition;
extern const CLI_Command_Definition_t mosfetStreamCommandDefinition;
extern const CLI_Command_Definition_t mosfetStopCommandDefinition ;
#endif /* H0FR7_H */

/************************ (C) COPYRIGHT HEXABITZ *****END OF FILE****/
