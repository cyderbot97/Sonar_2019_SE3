/*
 * bt.h
 *
 *  Created on: 3 sept. 2019
 *      Author: cyril
 */

#ifndef BSP_INC_BT_H_
#define BSP_INC_BT_H_

#include "stm32f0xx.h"


void BSP_BT_Init(void);
void BSP_PutChar(USART_TypeDef* USARTx, uint8_t ch);
void BSP_PutString(USART_TypeDef* USARTx, uint8_t * str);
void BSP_PutNumber(USART_TypeDef* USARTx, uint32_t x);




#endif /* BSP_INC_BT_H_ */
