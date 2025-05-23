/*
 * stm32f411xx_I2C_driver.h
 *
 *  Created on: 10-Jul-2023
 *      Author: admin
 */

#ifndef STM32F411XX_I2C_DRIVER_H_
#define STM32F411XX_I2C_DRIVER_H_

#include "stm32f411xx.h"
#include <stdint.h>

typedef struct
{
	uint32_t I2C_SCLSPEED;
	uint8_t I2C_DeviceAddress;
	uint8_t I2C_AckControl;
	uint8_t I2C_FM_Duty_Cycle;

} I2C_Config_t;

typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;

	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint32_t RxSize;
	uint8_t TxRxState;
	uint8_t DevAddr;
	uint8_t Sr;

} I2C_Handle_t;

// I2C_SCL_SPEED
#define I2C_SCL_SPEED_SM 100000
#define I2C_SCL_SPEED_FM4K 400000
#define I2C_SCL_SPEED_FM2K 200000

#define I2C_ACK_ENABLE 1
#define I2C_ACK_DISABLE 0

//PG 870-RM (CCR)
#define I2C_FM_DUTY_2 0		//DUTY = 0 --> T low = 2 * T high
#define I2C_FM_DUTY_16_9 1	//DUTY = 1 --> T low = 1.7 * T high  (16/9 = 1.78)

//APPLICATION STATES

#define I2C_READY 0
#define I2C_BUSY_IN_RX 1
#define I2C_BUSY_IN_TX 2
//#define I2C_BUST_IN_RX 3

#define I2C_DISABLE_SR RESET
#define I2C_ENABLE_SR SET

#define I2C_FLAG_TXE (1<<I2C_SR1_TXE)
#define I2C_FLAG_RXNE (1<<I2C_SR1_RXNE)
#define I2C_FLAG_SB (1<<I2C_SR1_SB)
#define I2C_FLAG_OVR (1<<I2C_SR1_OVR)
#define I2C_FLAG_AF (1<<I2C_SR1_AF)
#define I2C_FLAG_ARL0 (1<<I2C_SR1_ARL0)
#define I2C_FLAG_BERR (1<<I2C_SR1_BERR)
#define I2C_FLAG_STOPF (1<<I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 (1<<I2C_SR1_ADD10)
#define I2C_FLAG_BTF (1<<I2C_SR1_BTF)
#define I2C_FLAG_ADDR (1<<I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT (1<<I2C_SR1_TIMEOUT)

//Peripheral Clock Setup
void I2C_PeripheralClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnDi);


//Init and De-init
void I2C_Init(I2C_Handle_t *pI2C_Handle);

void I2C_DeInit(I2C_RegDef_t *pI2Cx);


//Master Data SEND and RECEIVE
void I2C_MasterSendData(I2C_Handle_t *pI2C_Handle, uint8_t *pTxBuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t SR);

void I2C_MasterReceiveData(I2C_Handle_t *pI2C_Handle, uint8_t *pRxBuffer, uint32_t Len, uint8_t SlaveAddress, uint8_t SR);


//IRQ Configuration and ISR Handling
void I2C_IRQInterruptConfig(uint8_t IRQNumber,uint8_t En_Di);

void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);


//Other Application API
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx,uint8_t En_Di);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint8_t FlagName);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2C_Handle,uint8_t AppEvent);

void I2C_ManageACKing(I2C_RegDef_t *pI2Cx,uint8_t En_Di);

uint32_t RCC_GetPCLK1Value(void);

uint32_t RCC_GetPLLOutputClock();

#endif /* STM32F411XX_I2C_DRIVER_H_ */
