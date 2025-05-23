
#include <stdint.h>
#include <stdio.h>
#include "stm32f411xx.h"

#define GPIOAEN (1U<<0)
#define UART2EN (1U<<17)

#define SYS_FREQ 16000000
#define APB1_CLK SYS_FREQ
#define UART_BAUDRATE 115200

#define CR1_TE  (1U<<3)
#define CR1_UE  (1U<<13)
#define CR1_RE  (1U<<2)
#define SR_TXE  (1U<<7)
#define SR_RXNE  (1U<<5)

void uart2_rxtx_init(void);
void uart2_write(int ch);
char uart2_read(void);

static void uart_set_baudrate(USART_RegDef_t *USARTx,uint32_t PeriphClk,uint32_t Baudrate);
static uint16_t compute_uart_bd(uint32_t PeriphClk,uint32_t Baudrate);

int __io_putchar(int ch)
{
	uart2_write(ch);
	return ch;
}

int main(void)
{
	char key;
	uart2_rxtx_init();

	while(1)
	{
		key=uart2_read();
		if (key=='1')
		{
			printf("Read Data : %c is pressed \n \r",key);
		}
		else if (key=='2')
		{
			printf("Read Data : %c is pressed \n \r",key);
		}
		else if (key=='3')
		{
			printf("\n Write Data\n \r");
			uart2_write('A');
		}
		else
		{
			printf("Read Data : Other Key pressed \n \r");
		}

	}
}

void uart2_rxtx_init(void)
{
	RCC->AHB1ENR |=GPIOAEN;

	// UART2 - PA2  -> TRANSMIT - Alternate function mode(10)
	GPIOA->MODER &=~(1U <<4);
	GPIOA->MODER |= (1U <<5);
	// UART2 - PA3  -> RECEIVE - Alternate function mode(10)
	GPIOA->MODER &=~(1U <<6);
    GPIOA->MODER |= (1U <<7);

    ///PA2  -> ALTERNATE - AF7(0111)
	GPIOA->AFR[0] |=(1U<<8);
	GPIOA->AFR[0] |=(1U<<9);
	GPIOA->AFR[0] |=(1U<<10);
	GPIOA->AFR[0] &=~(1U<<11);

    ///PA3  -> ALTERNATE - AF7(0111)
	GPIOA->AFR[0] |=(1U<<12);
	GPIOA->AFR[0] |=(1U<<13);
	GPIOA->AFR[0] |=(1U<<14);
	GPIOA->AFR[0] &=~(1U<<15);

	RCC->APB1ENR |=UART2EN;

	uart_set_baudrate(USART2,APB1_CLK,UART_BAUDRATE); //139
	//139 - 1000 1011 => DIV_Mantissa = 8, DIV_Fraction = B(11)
	//When USART_CR1_OVER8=1, the DIV_Fraction bit is not considered and must be kept cleared.

	//TE=1: Transmitter is enabled, RE=1:Receiver is enabled and begins searching for a start bit
	USART2->CR1 =CR1_TE |CR1_RE;
	USART2->CR1 |=CR1_UE; //UE=1: USART enabled
}

char uart2_read(void)
{
	//An interrupt is generated if RXNEIE=1 in the USART_CR1 register.
	//It is cleared by a read to the USART_DR register.
	//The RXNE flag can also be cleared by writing a zero to it.
	//This clearing sequence is recommended only for multibuffer communication.

	//CR1_RXNEIE: RXNE interrupt enable. This bit is set and cleared by software.
	//CR1_RXNEIE=1: An USART interrupt is generated, whenever ORE=1 or RXNE=1 in the USART_SR register

	//RXNE: Read data register not empty - RXNE=1: Received data is ready to be read
	//the content of the Receive shift register has been transferred to the RDR(Receive data register) - pg 968
	//get the "received data" from DR Register

	//make sure the data register is not empty
	while(!(USART2->SR  & SR_RXNE)){}

	//Read from Receive data register(RDR)
	return USART2->DR;
}

void uart2_write(int ch)
{
	//while(!(*USART2_SR  & 0x0080)){}
	//*USART2_DR =(ch&0XFF) ;

	//interrupt is generated, if the TXEIE bit =1 in the USART_CR1 register
	//It is cleared by a write to the USART_DR register.

	//CR1_TXEIE: TXE interrupt enable. This bit is set and cleared by software
	//CR1_TXEIE=1: An USART interrupt is generated, whenever TXE=1 in the USART_SR register

	//TXE: Transmit data register empty - TXE=1: Data is transferred to the shift register
	//Content of the TDR(Transmit Data Register)register has been transferred into the Transmit shift register - pg 968
	//so keep the "data to transmit" in DR Register

	//Make sure the transmit data register(TDR) is empty
	while(!(USART2->SR  & SR_TXE)){}

	//write to transmit data register(TDR)
	USART2->DR =(ch&0XFF) ; //1.X=X
}

static void uart_set_baudrate(USART_RegDef_t *USARTx,uint32_t PeriphClk,uint32_t Baudrate)
{
	USARTx->BRR = compute_uart_bd(PeriphClk,Baudrate); //139.38889
}

static uint16_t compute_uart_bd(uint32_t PeriphClk,uint32_t Baudrate)
{
	return ((PeriphClk+(Baudrate/2U))/Baudrate);  //139.38889
}
