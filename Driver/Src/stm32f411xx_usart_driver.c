/*
 * stm32f411xx_usart_driver.c
 *
 *  Created on: 22-Jul-2023
 *      Author: admin
 */

#include <stdint.h>
#include "stm32f411xx_usart_driver.h"
#include "stm32f411xx_I2C_driver.h"

//Peripheral Clock Setup
void USART_PeripheralClockControl(USART_RegDef_t *pUSARTx, uint8_t EnDi)
{
	if(EnDi == ENABLE)
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_EN();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_EN();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_EN();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_EN();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_EN();
		}
		else if(pUSARTx == UART7)
		{
			UART7_PCLK_EN();
		}
		else if(pUSARTx == UART8)
		{
			UART8_PCLK_EN();
		}
	}

	else
	{
		if(pUSARTx == USART1)
		{
			USART1_PCLK_DI();
		}
		else if(pUSARTx == USART2)
		{
			USART2_PCLK_DI();
		}
		else if(pUSARTx == USART3)
		{
			USART3_PCLK_DI();
		}
		else if(pUSARTx == UART4)
		{
			UART4_PCLK_DI();
		}
		else if(pUSARTx == UART5)
		{
			UART5_PCLK_DI();
		}
		else if(pUSARTx == UART7)
		{
			UART7_PCLK_DI();
		}
		else if(pUSARTx == UART8)
		{
			UART8_PCLK_DI();
		}
	}
}

uint16_t AHB_Prescalr[8] = {2,4,8,16,64,128,256,512};
uint16_t APB2_Prescaler[4] = {2,4,8,16};

uint32_t RCC_GetPCLK2Value(void)
{
	uint32_t PCLK2, SystemClk;
	uint8_t ClkSrc, temp, AHB_PreS, APB2_PreS;

	ClkSrc = ( (RCC->CFGR >> 2) & 0x3 ) ;
	if(ClkSrc == 0)
	{
		SystemClk = 16000000;
	}
	else if(ClkSrc == 1)
	{
		SystemClk = 8000000;
	}
	else if(ClkSrc == 2)
	{
		SystemClk = RCC_GetPLLOutputClock();
	}
	//else if(ClkSrc == 3)  NotApplicable

	//AHB CLOCK
	temp = ( (RCC->CFGR >> 4) & 0xF ) ;
	if(temp<8)
	{
		AHB_PreS = 1;
	}
	else
	{
		AHB_PreS = AHB_Prescalr[temp-8];
	}

	//APB2 CLOCK
	temp = ( (RCC->CFGR >> 13) & 0x7 ) ;
	if(temp<0x04)
	{
		APB2_PreS = 1;
	}
	else
	{
		APB2_PreS = APB2_Prescaler[temp-4];
	}

	PCLK2 = (SystemClk/AHB_PreS) / APB2_PreS;
	return PCLK2;
}

//Init and De-init
void USART_Init(USART_Handle_t *pUSART_Handle)
{
	//Temporary Variable
	uint32_t TempReg=0;

	//Enable the clock
	USART_PeripheralClockControl(pUSART_Handle->pUSARTx, ENABLE);


	//Configuration of CR1
	//(1) Configure Tx and Rx Enable
	if(pUSART_Handle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement code to Enable the Receiver Bit Field
		TempReg |= (1<<USART_CR1_RE);
	}
	else if(pUSART_Handle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement code to Enable the Transmitter Bit Field
		TempReg |= (1<<USART_CR1_TE);
	}
	else if(pUSART_Handle->USART_Config.USART_Mode == USART_MODE_TxRx)
	{
		//Implement code to Enable BOTH the Transmitter and Receiver Bit Field
		TempReg |= ( (1<<USART_CR1_RE) | (1<<USART_CR1_TE) );
	}

	//(2) Configure Word Length
	TempReg |= pUSART_Handle->USART_Config.USART_WordLength << USART_CR1_M;

	//(3) Configure Parity Control Bit
	if(pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
		{
			//Implement code to Enable Parity Control
			TempReg |= (1<<USART_CR1_PCE);

			//Implement code to Enable EVEN Parity
			/*  By default USART_CR1_PS = 0 , So Not Required   */
		}
		else if(pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD)
		{
			//Implement code to Enable Parity Control
			TempReg |= (1<<USART_CR1_PCE);

			//Implement code to Enable ODD Parity
			TempReg |= (1<<USART_CR1_PS);
		}

	pUSART_Handle->pUSARTx->CR1 = TempReg;


	//Configuration of CR2
	TempReg=0;
	TempReg |= pUSART_Handle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP1_0;
	pUSART_Handle->pUSARTx->CR2 = TempReg;


	//Configuration of CR3
	TempReg=0;
	if(pUSART_Handle->USART_Config.USART_HardwareFlowControl == USART_HW_FLOW_CTRL_CTS)
		{
			//Implement code to Enable the CTS Flow Control
			TempReg |= (1<<USART_CR3_CTSE);
		}
		else if(pUSART_Handle->USART_Config.USART_HardwareFlowControl == USART_HW_FLOW_CTRL_RTS)
		{
			//Implement code to Enable the RTS Flow Control
			TempReg |= (1<<USART_CR3_RTSE);
		}
		else if(pUSART_Handle->USART_Config.USART_HardwareFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
		{
			//Implement code to Enable BOTH the CTS and RTS Flow Control
			TempReg |= ( (1<<USART_CR3_CTSE) | (1<<USART_CR3_RTSE) );
		}
	pUSART_Handle->pUSARTx->CR3 = TempReg;

	//Configuration of BRR
	USART_SetBaudRate(pUSART_Handle->pUSARTx,pUSART_Handle->USART_Config.USART_Baud);

}

void USART_DeInit(USART_Handle_t *pUSART_Handle)
{

}


//Data Send and Receive
void USART_SendData(USART_Handle_t *pUSART_Handle,uint8_t *pTxBuffer,uint32_t Len)
{
	uint16_t *pData;
	for(uint32_t i = 0;i<Len;i++)
	{
		//wait until TXE Flag is set
		while(! USART_GetFlagStatus(pUSART_Handle->pUSARTx,USART_FLAG_TXE));

		if(pUSART_Handle->USART_Config.USART_WordLength == USART_WORDLEN_9_BITS)
		{
			//if 9Bit Length - Load DR with 2 bytes and mask(set 0) remaining bits other that 0-9 bits
			pData = (uint16_t*) pTxBuffer;
			pUSART_Handle->pUSARTx->DR = (*pData & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSART_Handle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No Parity Bit is used in this Transfer, so 9 bits of user data
				// increment pTxBuffer Twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity Bit is used in this Transfer, so 8 bits of user data
				//9th bit will be replaced by Parity Bit by the Hardware
				// increment pTxBuffer once
				pTxBuffer++;
			}
		}
		else
		{
			//if 8Bit Length - Load DR with 1 byte Data Transfer
			pUSART_Handle->pUSARTx->DR = (*pTxBuffer & (uint8_t)0xFF);

			// increment pTxBuffer once
			pTxBuffer++;
		}



		//wait until Transmission is Complete (USART_SR_TC=1)
		while(! USART_GetFlagStatus(pUSART_Handle->pUSARTx,USART_FLAG_TC));
	}
}

void USART_ReceiveData(USART_Handle_t *pUSART_Handle,uint8_t *pRxBuffer,uint32_t Len)
{

}


//Other Application API
void USART_PeripheralControl(USART_RegDef_t *pUSARTx,uint8_t EnDi)
{
	if(EnDi == ENABLE)
	{
		pUSARTx->CR1 |= (1<<USART_CR1_UE);
	}
	else
	{
		pUSARTx->CR1 &= ~(1<<USART_CR1_UE);
	}
}

uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx,uint8_t StatusFlagName)
{
	if(pUSARTx->SR & StatusFlagName)
	{
		return SET;
	}
	else
		return RESET;
}

void USART_ClearFlag(USART_RegDef_t *pUSARTx,uint8_t StatusFlagName)
{
	pUSARTx->SR &= ~(StatusFlagName);
}

void USART_SetBaudRate(USART_RegDef_t *pUSARTx,uint32_t BaudRate)
{
	//variable to hold Temporary Register
	uint32_t tempreg = 0;

	//variable to hold APB Clock
	uint32_t PCLKx;
	uint32_t usartdiv;

	//variable to hold Mantissa and Fraction Values
	uint32_t M_Part, F_Part;

	//Get the Value of APB Clock
	if(pUSARTx == USART1 || pUSARTx == USART6)
	{
		//APB2 - USART1, USART6
		PCLKx = RCC_GetPCLK2Value();
	}
	else
	{
		//APB1 - USART2, USART3, UART4, UART5, UART7, UART8
		PCLKx = RCC_GetPCLK1Value();
	}


	//Check the Overflow OVER8 or OVER16 Configuration Bit (Sampling Rate - 8 or 16)
	if(pUSARTx->CR1 & (1<<USART_CR1_OVER8))
	{
		//OVER8 = 1,  over sampling by 8
		usartdiv = ( (25 * PCLKx) / (2 * BaudRate) );
	}
	else
	{
		//OVER8 = 0,  over sampling by 16
		usartdiv = ( (25 * PCLKx) / (4 * BaudRate) );
	}

	//Calculate the Mantissa Part
	M_Part = usartdiv/100;
	tempreg |= M_Part<<4;

	//Calculate the Fractional Part
	F_Part = (usartdiv - (M_Part * 100));
	if(pUSARTx->CR1 & (1<<USART_CR1_OVER8))
	{
		//OVER8 = 1,  over sampling by 8
		F_Part = ((( F_Part * 8) + 50) / 100) & ( (uint8_t)0x0F );
	}
	else
	{
		//OVER8 = 0,  over sampling by 16
		F_Part = ((( F_Part * 16) + 50) / 100) & ( (uint8_t)0x0F );
	}
	tempreg |= F_Part;

	pUSARTx->BRR = tempreg;

}
