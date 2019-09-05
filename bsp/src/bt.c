/*
 * bt.c
 *
 *  Created on: 3 sept. 2019
 *      Author: cyril
 */
#include "bt.h"

void BSP_BT_Init() {

	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA9 and PA10 as Alternate function
	GPIOA->MODER &= ~(GPIO_MODER_MODER9_Msk | GPIO_MODER_MODER10_Msk);
	GPIOA->MODER |= (0x02 << GPIO_MODER_MODER9_Pos)
			| (0x02 << GPIO_MODER_MODER10_Pos);

	// Set PA9 and PA10 to AF1 (USART1)
	GPIOA->AFR[1] &= ~(0x00000FF0);
	GPIOA->AFR[1] |= (0x00000110);

	// Enable USART1 clock
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	// Clear USART1 configuration (reset state)
	// 8-bit, 1 start, 1 stop, CTS/RTS disabled
	USART1->CR1 = 0x00000000;
	USART1->CR2 = 0x00000000;
	USART1->CR3 = 0x00000000;

	// Select PCLK (APB1) as clock source
	// PCLK -> 48 MHz
	RCC->CFGR3 &= ~RCC_CFGR3_USART1SW_Msk;

	// Baud Rate = 115200
	// With OVER8=0 and Fck=48MHz, USARTDIV =   48E6/115200 = 416.6666
	// BRR = 417 -> Baud Rate = 115107.9137 -> 0.08% error
	//
	// With OVER8=1 and Fck=48MHz, USARTDIV = 2*48E6/115200 = 833.3333
	// BRR = 833 -> Baud Rate = 115246.0984 -> 0.04% error (better)

	USART1->CR1 |= USART_CR1_OVER8;
	USART1->BRR = 833;

	// Enable both Transmitter and Receiver
	USART1->CR1 |= USART_CR1_TE | USART_CR1_RE;

	// Enable USART1
	USART1->CR1 |= USART_CR1_UE;

	// Enable interrupt on RXNE event
	USART1->CR1 |= USART_CR1_RXNEIE;
}

/*
 * BSP_PutChar()
 * Send a character throw Usart
 */
void BSP_PutChar(USART_TypeDef* USARTx, uint8_t ch) {
	while ((USARTx->ISR & USART_ISR_TC) != USART_ISR_TC)
		;
	USARTx->TDR = ch;
}

/*
 * BSP_PutString()
 * Send a string throw Usart
 */
void BSP_PutString(USART_TypeDef* USARTx, uint8_t * str) {
	while (*str != 0) {
		BSP_PutChar(USARTx, *str);
		str++;
	}
}

/*
 * BSP_PutNumber()
 * Send a number throw Usart
 */


void BSP_PutNumber(USART_TypeDef* USARTx, uint32_t x) {
	char value[10]; //a temp array to hold results of conversion
	int i = 0; //loop index

	do {
		value[i++] = (char) (x % 10) + '0'; //convert integer to character
		x /= 10;
	} while (x);

	while (i) //send data
	{
		BSP_PutChar(USARTx, value[--i]);
	}
}
