/*
 * stm32f411xx_usart_driver.h
 *
 *  Created on: 22-Jul-2023
 *      Author: admin
 */

#ifndef STM32F411XX_USART_DRIVER_H_
#define STM32F411XX_USART_DRIVER_H_

#include <stdint.h>
#include "stm32f411xx.h"


typedef struct
{
	uint8_t USART_Mode;	//USART_MODE
	uint32_t USART_Baud;	//USART_BAUD
	uint8_t USART_NoOfStopBits;	//USART_NO_OF_STOP_BITS
	uint8_t USART_WordLength;	//USART_WORDLENGTH
	uint8_t USART_ParityControl;	//USART_PARITY_CONTROL
	uint8_t USART_HardwareFlowControl;	//USART_HWFLOWCONTROL

} USART_Config_t;

typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t USART_Config;

	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;

} USART_Handle_t;


//Peripheral Clock Setup
void USART_PeripheralClockControl(USART_RegDef_t *pUSARTx, uint8_t EnDi);


//Init and De-init
void USART_Init(USART_Handle_t *pUSART_Handle);

void USART_DeInit(USART_Handle_t *pUSART_Handle);


//Data Send and Receive
void USART_SendData(USART_Handle_t *pUSART_Handle,uint8_t *pTxBuffer,uint32_t Len);

void USART_ReceiveData(USART_Handle_t *pUSART_Handle,uint8_t *pRxBuffer,uint32_t Len);


//Other Application API
void USART_PeripheralControl(USART_RegDef_t *pUSARTx,uint8_t EnDi);

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx,uint8_t StatusFlagName);

void USART_ClearFlag(USART_RegDef_t *pUSARTx,uint8_t StatusFlagName);

void USART_SetBaudRate(USART_RegDef_t *pUSARTx,uint32_t BaudRate);

uint32_t RCC_GetPCLK2Value(void);


/*	USART_MODE
	  */
#define USART_MODE_ONLY_TX 0
#define USART_MODE_ONLY_RX 1
#define USART_MODE_TxRx 2

// USART_STANDARD__BAUD_RATE
#define USART_STND_BAUD_1200 1200
#define USART_STND_BAUD_2400 2400
#define USART_STND_BAUD_9600 9600
#define USART_STND_BAUD_19200 19200
#define USART_STND_BAUD_38400 38400
#define USART_STND_BAUD_57600 57600
#define USART_STND_BAUD_115200 115200
#define USART_STND_BAUD_230400 230400
#define USART_STND_BAUD_460800 460800
#define USART_STND_BAUD_921600 921600
#define USART_STND_BAUD_2M 2000000
#define USART_STND_BAUD_3M 3000000

/*	USART_NO_OF_STOP_BITS
	USART_CR2_STOP1_0  */
#define USART_STOP_BITS_1 	 0	// 1 Stop Bit
#define USART_STOP_BITS_0_5  1	// 0.5 Stop Bit
#define USART_STOP_BITS_2 	 2	// 2 Stop Bits
#define USART_STOP_BITS_1_5  3	// 1.5 Stop Bit

/*	USART_WORDLENGTH
	USART_CR1_M  */
#define USART_WORDLEN_8_BITS 0
#define USART_WORDLEN_9_BITS 1

/*	USART_PARITY_CONTROL
	USART_CR1_PCE_10 --> 0 - Parity Disable, 1 - Parity Enable
	USART_CR1_PS_9	--> 0 - Even Parity, 1 - Odd Parity */

#define USART_PARITY_DISABLE 0
#define USART_PARITY_EN_EVEN 1
#define USART_PARITY_EN_ODD 2

/*	USART_HWFLOWCONTROL
	USART_CR3_RTSE
	USART_CR3_CTSE  */
#define USART_HW_FLOW_CTRL_NONE		0
#define USART_HW_FLOW_CTRL_CTS		1
#define USART_HW_FLOW_CTRL_RTS		2
#define USART_HW_FLOW_CTRL_CTS_RTS	3

//USART FLAGS
#define USART_FLAG_TXE (1<<USART_SR_TXE)
#define USART_FLAG_RXNE (1<<USART_SR_RXNE)
#define USART_FLAG_TC (1<<USART_SR_TC)

//APPLICATION STATES
#define USART_BUSY_IN_RX 1
#define USART_BUSY_IN_TX 2
#define USART_READY 0

#define USART_EVENT_TX_CMPLT 0
#define USART_EVENT_RX_CMPLT 1
#define USART_EVENT_IDLE 2
#define USART_EVENT_CTS 3
#define USART_EVENT_PE 4
#define USART_ERR_FE 5
#define USART_ERR_NE 6
#define USART_ERR_ORE 7

#define USART_


#endif /* STM32F411XX_USART_DRIVER_H_ */
