/**
  ******************************************************************************
  * @file    Templates/Src/stm32f0xx_it.c 
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
#include "stm32f0xx.h"
#include "bsp.h"


/** @addtogroup STM32F0xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  // HAL_IncTick();
}

/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/

extern uint8_t	console_rx_byte;
extern uint8_t	console_rx_irq;

void USART1_IRQHandler()
{
	// Test for RXNE pending interrupt
	if ((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
	{
		// RXNE flags automatically clears when reading RDR.

		// Store incoming byte
		console_rx_byte = USART1->RDR;
	console_rx_irq = 1;
	}
}

extern uint8_t timebase_irq;
extern uint8_t oc_inter;

void TIM16_IRQHandler()
{
	// Test for TIM6 update pending interrupt

	if ((TIM16->SR & TIM_SR_UIF) == TIM_SR_UIF)
	{
		//clear flag
		TIM16->SR &= ~TIM_SR_UIF;

		//timebase_irq = 1;
		TIM1->CR1 |= TIM_CR1_CEN;
		BSP_LED_On();
	}
	if ((TIM16->SR & TIM_SR_CC1IF) == TIM_SR_CC1IF)
	{
		//clear flag
		TIM16->SR &= ~TIM_SR_CC1IF;

		TIM1->CR1 &= ~TIM_CR1_CEN;
		BSP_LED_Off();
	}
}

extern uint16_t a;

void ADC1_COMP_IRQHandler (void){

	if ((ADC1->ISR & ADC_ISR_EOC) == ADC_ISR_EOC)
		{
			ADC1->ISR |= ADC_ISR_EOC;
			a=ADC1->DR - 50;
			if(a<310){
				TIM16->CCR1 = 200;
				TIM16->ARR = 210;
			}else if(a>1000){
				TIM16->CCR1 = 0;
				TIM16->ARR = 210;
			}
			else{
				TIM16->CCR1 = 200;
				TIM16->ARR = a;
			}
		}

}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @}
  */ 

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
