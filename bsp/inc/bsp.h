#ifndef BSP_INC_BSP_H_
#define BSP_INC_BSP_H_

#include "stm32f0xx.h"


void	BSP_LED_Init			(void);
void 	BSP_LED_Toggle			(void);

void 	BSP_TIMER_Timebase_Init	(void);

void 	BSP_BUZZER_INIT			(void);
void 	BSP_TIMER_PWM_Init		(void);
void 	BSP_UTLRASONIC_PWM_INIT	(void);
void 	BSP_ULTRASONIC_ADC_INIT	(void);

void 	BSP_NVIC_Init 			(void);

void	BSP_PB_Init				(void);

void	BSP_Console_Init		(void);



#endif /* BSP_INC_BSP_H_ */
