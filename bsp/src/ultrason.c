/*
 * ultrason.c
 *
 *  Created on: 5 sept. 2019
 *      Author: cyril
 */
#include "ultrason.h"

void BSP_ULTRASONIC_ADC_INIT()
{
	//set timer to trig ADC conversion
	RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;

	TIM15->CR1 = 0x0000;
	TIM15->CR2 = 0x0000;

	TIM15->PSC = (uint16_t) 48 -1;

	TIM15->ARR = (uint16_t) 1000 -1;

	TIM15->CR1 |= TIM_CR1_ARPE;

	TIM15->CR2 |= (0x02 <<TIM_CR2_MMS_Pos);

	TIM15->CR1 |= TIM_CR1_CEN;

	// Enable GPIOC clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// Configure pin PC1 as analog
	GPIOC->MODER &= ~(GPIO_MODER_MODER1_Msk);
	GPIOC->MODER |= (0x03 <<GPIO_MODER_MODER1_Pos);

	// Enable ADC clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// Reset ADC configuration
	ADC1->CR 	= 0x00000000;
	ADC1->CFGR1  = 0x00000000;
	ADC1->CFGR2  = 0x00000000;
	ADC1->CHSELR = 0x00000000;

	// Enable 12-bit continuous conversion mode
	//ADC1->CFGR1 |= (0x00 <<ADC_CFGR1_RES_Pos) | ADC_CFGR1_CONT;

	//Trig ADC with TIM15
	ADC1->CFGR1 |= 0x04 << ADC_CFGR1_EXTSEL_Pos | ADC_CFGR1_EXTEN;

	// Select PCLK/2 as ADC clock
	ADC1->CFGR2 |= (0x01 <<ADC_CFGR2_CKMODE_Pos);

	// Set sampling time to 28.5 ADC clock cycles
	ADC1->SMPR = 0x03;

	ADC1->IER |= ADC_IER_EOCIE;

	// Select channel 11
	ADC1->CHSELR |= ADC_CHSELR_CHSEL11;

	// Enable ADC
	ADC1->CR |= ADC_CR_ADEN;

	// Start conversion
	ADC1->CR |= ADC_CR_ADSTART;
}

/*	Non utilisé
void BSP_UTLRASONIC_PWM_INIT()
{
	// Enable GPIOB clock
	RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

	// Configure PB4 as Alternate function
	GPIOB->MODER &= ~(GPIO_MODER_MODER4_Msk);
	GPIOB->MODER |=  (0x02 <<GPIO_MODER_MODER4_Pos);

	// Set PB4 to AF1 (TIM3_CH1)
	GPIOB->AFR[0] &= ~(0x000F0000);
	GPIOB->AFR[0] |=  (0x00010000);

	// Enable TIM3 clock
	RCC -> APB1ENR |= RCC_APB1ENR_TIM3EN;

	// Reset TIM3 configuration
	TIM3->CR1  = 0x0000;
	TIM3->CR2  = 0x0000;
	TIM3->CCER = 0x0000;

	// Set TIM3 prescaler
	// Fck = 48MHz -> /48000 = 1KHz counting frequency
	TIM3->PSC = (uint16_t) 48 -1;

	// Set Auto-Reload to maximum value
	TIM3->ARR = (uint16_t) 0xFFFF;

	// Setup Input Capture
	TIM3->CCMR1 = 0x0000;
	TIM3->CCMR2 = 0x0000;

	// Channel 1 input on TI1
	TIM3->CCMR1 |= (0x01 <<TIM_CCMR1_CC1S_Pos);

	// Channel 2 input also on TI1
	TIM3->CCMR1 |= (0x02 <<TIM_CCMR1_CC2S_Pos);

	// Filter with N=8
	TIM3->CCMR1 |= (0x03 <<TIM_CCMR1_IC1F_Pos) | (0x03 <<TIM_CCMR1_IC2F_Pos);

	// Select rising edge for channel 1
	TIM3->CCER |= (0x00 <<TIM_CCER_CC1NP_Pos) | (0x00 <<TIM_CCER_CC1P_Pos);

	// Select falling edge for channel 2
	TIM3->CCER |= (0x00 <<TIM_CCER_CC2NP_Pos) | (0x01 <<TIM_CCER_CC2P_Pos);

	// Enable capture on channel 1 & channel 2
	TIM3->CCER |= (0x01 <<TIM_CCER_CC1E_Pos) | (0x01 <<TIM_CCER_CC2E_Pos);

	// Choose Channel 1 as trigger input
	TIM3->SMCR |= (0x05 <<TIM_SMCR_TS_Pos);

	// Slave mode -> Resets counter when trigger occurs
	TIM3->SMCR |= (0x4 <<TIM_SMCR_SMS_Pos);

	// Enable TIM3
	TIM3->CR1 |= TIM_CR1_CEN;
}*/
