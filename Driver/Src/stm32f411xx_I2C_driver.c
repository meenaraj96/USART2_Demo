/*
 * stm32f411xx_I2C_driver.c
 *
 *  Created on: 25-Jul-2023
 *      Author: admin
 */

#include "stm32f411xx_I2C_driver.h"
#include "stm32f411xx.h"
#include <stdint.h>

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddress);
static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddress);
static void I2C_ClearAddressFlag(I2C_Handle_t *pI2C_Handle);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_START);
}

static void I2C_ExecuteAddressPhaseWrite(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddress)
{
	SlaveAddress = SlaveAddress<<1;
	SlaveAddress &= ~(1); //WRITE Bit =0
	pI2Cx->DR = SlaveAddress;
}

static void I2C_ExecuteAddressPhaseRead(I2C_RegDef_t *pI2Cx,uint8_t SlaveAddress)
{
	SlaveAddress = SlaveAddress<<1;
	SlaveAddress |= (1); //READ Bit =1
	pI2Cx->DR = SlaveAddress;
}

static void I2C_ClearAddressFlag(I2C_Handle_t *pI2C_Handle)
{
	uint32_t dummy_read;
	//check for Device Mode
	if(pI2C_Handle->pI2Cx->SR2 & (1<<I2C_SR2_MSL))
	{
		//Device in MASTER Mode
		if(pI2C_Handle->TxRxState == I2C_BUSY_IN_RX)
		{
			if(pI2C_Handle->RxSize ==1)
			{
				//DISABLE the ACK
				I2C_ManageACKing(pI2C_Handle->pI2Cx,DISABLE);

				//clear ADDR FLAG (Read SR1, READ SR2)
				dummy_read= pI2C_Handle->pI2Cx->SR1;
				dummy_read= pI2C_Handle->pI2Cx->SR2;
				(void)dummy_read;
			}
		}
		else
		{
			//clear ADDR FLAG (Read SR1, READ SR2)
			dummy_read= pI2C_Handle->pI2Cx->SR1;
			dummy_read= pI2C_Handle->pI2Cx->SR2;
			(void)dummy_read;
		}
	}
	else
	{
		//Device in SLAVE Mode
		//clear ADDR FLAG (Read SR1, READ SR2)
		dummy_read= pI2C_Handle->pI2Cx->SR1;
		dummy_read= pI2C_Handle->pI2Cx->SR2;
		(void)dummy_read;
	}

}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
	pI2Cx->CR1 |= (1<<I2C_CR1_STOP);
}

void I2C_ManageACKing(I2C_RegDef_t *pI2Cx,uint8_t En_Di)
{
	if(En_Di == I2C_ACK_ENABLE)
	{
		//Enable the ACK
		pI2Cx->CR1 |= (1<<I2C_CR1_ACK);
	}
	else
	{
		//Disable the ACK
		pI2Cx->CR1 &= ~(1<<I2C_CR1_ACK);
	}
}

uint32_t RCC_GetPLLOutputClock()
{
	return 0;
}

//Peripheral Clock Setup
void I2C_PeripheralClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnDi)
{
	if(EnDi == ENABLE)
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_EN();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_EN();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_EN();
		}
	}
	else
	{
		if(pI2Cx == I2C1)
		{
			I2C1_PCLK_DI();
		}
		else if(pI2Cx == I2C2)
		{
			I2C2_PCLK_DI();
		}
		else if(pI2Cx == I2C3)
		{
			I2C3_PCLK_DI();
		}
	}
}

uint16_t AHB_Prescaler[8] = {2,4,8,16,64,128,256,512};
uint16_t APB1_Prescaler[4] = {2,4,8,16};

uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t PCLK1, SystemClk;
	uint8_t ClkSrc, temp, AHB_PreS, APB1_PreS;

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
		AHB_PreS = AHB_Prescaler[temp-8];
	}

	//APB1 CLOCK
	temp = ( (RCC->CFGR >> 10) & 0x7 ) ;
	if(temp<4)
	{
		APB1_PreS = 1;
	}
	else
	{
		APB1_PreS = APB1_Prescaler[temp-4];
	}

	PCLK1 = (SystemClk/AHB_PreS) / APB1_PreS;
	return PCLK1;
}


//Init and De-init
void I2C_Init(I2C_Handle_t *pI2C_Handle)
{
	uint32_t TempReg = 0;

	//Enable Clock for I2C Peripheral
	I2C_PeripheralClockControl(pI2C_Handle->pI2Cx,ENABLE);

	//Enable ACK Control Bit
	TempReg |= pI2C_Handle->I2C_Config.I2C_AckControl << 10;
	pI2C_Handle->pI2Cx->CR1 = TempReg;

	//Configure FREQ Field of CR2
	TempReg = 0;
	TempReg |= RCC_GetPCLK1Value()/1000000u;
	pI2C_Handle->pI2Cx->CR2 = (TempReg & 0x3F);

	//Program the device own Address
	TempReg = 0;
	TempReg |= pI2C_Handle->I2C_Config.I2C_DeviceAddress << 1;
	TempReg |= (1<<15);
	pI2C_Handle->pI2Cx->OAR1 = TempReg;

	//CCR Calculation
	uint16_t CCR_VALUE = 0;
	TempReg = 0;
	if(pI2C_Handle->I2C_Config.I2C_SCLSPEED <= I2C_SCL_SPEED_SM)
	{
		//Mode - Standard Mode
		CCR_VALUE = ( RCC_GetPCLK1Value() / (2*pI2C_Handle->I2C_Config.I2C_SCLSPEED) );
		TempReg |= (CCR_VALUE & 0xFFF);
	}
	else
	{
		//Mode - Fast Mode
		TempReg |= (1<<15);
		TempReg |= (pI2C_Handle->I2C_Config.I2C_FM_Duty_Cycle << 14);

		if(pI2C_Handle->I2C_Config.I2C_FM_Duty_Cycle == I2C_FM_DUTY_2)
		{
			CCR_VALUE = ( RCC_GetPCLK1Value() / (3*pI2C_Handle->I2C_Config.I2C_SCLSPEED) );
		}
		else
		{
			CCR_VALUE = ( RCC_GetPCLK1Value() / (25*pI2C_Handle->I2C_Config.I2C_SCLSPEED) );
		}

		TempReg |= (CCR_VALUE & 0xFFF);

	}
	pI2C_Handle->pI2Cx->CCR = TempReg;

	//TRISE Configuration
	if(pI2C_Handle->I2C_Config.I2C_SCLSPEED <= I2C_SCL_SPEED_SM)
	{
		//Mode - Standard Mode
		TempReg |= ( RCC_GetPCLK1Value() / 1000000U) + 1; //Incremented by 1 - Given in Reference Manual(page 871)
	}
	else
	{
		//Mode - Fast Mode
		TempReg |= ( (RCC_GetPCLK1Value() * 300) / 1000000U) + 1; //Incremented by 1 - Given in Reference Manual(page 871)
	}
	pI2C_Handle->pI2Cx->TRISE = (TempReg & 0x3F);

}

void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

}


void I2C_MasterSendData(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t SR)
{
	// (1) Generate START Condition
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

	// (2) Confirm Start Generation is completed by checking the SB Flag in SR1 Flag
	while(! I2C_GetFlagStatus(pI2C_Handle->pI2Cx,I2C_FLAG_SB) );

	// (3) send slave address R/W bit SET - totally 8 bits (WRITE)
	I2C_ExecuteAddressPhaseWrite(pI2C_Handle->pI2Cx,SlaveAddress);

	// (4) confirm address phase is completed by checking the ADDR Flag in SR1 Flag
	while(! I2C_GetFlagStatus(pI2C_Handle->pI2Cx,I2C_FLAG_ADDR) );

	// (5) CLEAR THE ADDR Flag
	I2C_ClearAddressFlag(pI2C_Handle);

	// (6) send the data till Len = 0
	while(Len>0)
	{
		while(! I2C_GetFlagStatus(pI2C_Handle->pI2Cx,I2C_FLAG_TXE) );
		pI2C_Handle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		Len--;
	}

	// (7) when Len = 0, wait for TXE=1 and BTF=1
	while(! I2C_GetFlagStatus(pI2C_Handle->pI2Cx,I2C_FLAG_TXE) );
	while(! I2C_GetFlagStatus(pI2C_Handle->pI2Cx,I2C_FLAG_BTF) );

	// (8) Generate a STOP Condition
	if(SR==I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);

}

void I2C_MasterReceiveData(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t SR)
{
	// (1) Generate START Condition
	I2C_GenerateStartCondition(pI2C_Handle->pI2Cx);

	// (2) Confirm Start Generation is completed by checking the SB Flag in SR1 Flag
	while(! I2C_GetFlagStatus(pI2C_Handle->pI2Cx,I2C_FLAG_SB) );

	// (3) send slave address R/W bit SET - totally 8 bits (READ)
	I2C_ExecuteAddressPhaseRead(pI2C_Handle->pI2Cx,SlaveAddress);

	// (4) confirm address phase is completed by checking the ADDR Flag in SR1 Flag
	while(! I2C_GetFlagStatus(pI2C_Handle->pI2Cx,I2C_FLAG_ADDR) );

	// (5) Procedure to read only 1 byte from slave
	if(Len == 1)
	{
		//CLEAR THE ADDR Flag
		I2C_ClearAddressFlag(pI2C_Handle);

		//Wait until RXNE is 1
		while(! I2C_GetFlagStatus(pI2C_Handle->pI2Cx,I2C_FLAG_RXNE) );

		//Generate a STOP Condition
		if(SR==I2C_DISABLE_SR)
		I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);

		//Read Data from Buffer
		*pRxBuffer = pI2C_Handle->pI2Cx->DR;

	}

	// (6) Procedure to read for multiple bytes from slave
	if(Len > 1)
	{
		//CLEAR THE ADDR Flag
		I2C_ClearAddressFlag(pI2C_Handle);

		//Read until Len becomes 0
		for(uint32_t i=Len;i>0;i--)
		{
			//Wait until RXNE is 1
			while(! I2C_GetFlagStatus(pI2C_Handle->pI2Cx,I2C_FLAG_RXNE) );

			//Last 2 Bytes Remaining
			if(i==2)
			{
				//DISABLE ACKing
				I2C_ManageACKing(pI2C_Handle->pI2Cx,I2C_ACK_DISABLE);

				//Generate a STOP Condition
				I2C_GenerateStopCondition(pI2C_Handle->pI2Cx);
			}
			//Read Data from Buffer
			*pRxBuffer = pI2C_Handle->pI2Cx->DR;
			pRxBuffer++;
		}
	}
	// (7) RE-ENABLE ACKing
	if(pI2C_Handle->I2C_Config.I2C_AckControl == I2C_ACK_ENABLE)
		I2C_ManageACKing(pI2C_Handle->pI2Cx,I2C_ACK_ENABLE);

}

//IRQ Configuration and ISR Handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber,uint8_t En_Di)
{

}

void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{

}


//Other Application API
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t En_Di)
{
	if(En_Di==ENABLE)
		pI2Cx->CR1 |= (1<<I2C_CR1_PE);
	else
		pI2Cx->CR1 |= (1<<I2C_CR1_PE);
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint8_t FlagName)
{
	if(pI2Cx->SR1 & FlagName)
		return FLAG_SET;
	return FLAG_RESET;
}

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle,uint8_t AppEvent)
{

}



// ()
// ()
// ()
// ()
// ()
// ()
// ()
// ()
// ()
// ()
// ()
// ()
