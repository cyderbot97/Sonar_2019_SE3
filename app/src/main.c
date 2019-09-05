/*
 * main.c
 *
 *  Created on:
 *      Author: cyril
 */

#include "stm32f0xx.h"
#include "main.h"
#include "bsp.h"
#include "delay.h"
#include "bt.h"
#include "i2c.h"
#include "ultrason.h"



// Global functions declaration
extern int my_printf(const char *format, ...);
extern int my_sprintf(char *out, const char *format, ...);

// Global variables
uint8_t console_rx_byte;
uint8_t console_rx_irq = 0;


/*
 * Local Static Functions
 */
static uint8_t SystemClock_Config	(void);
uint16_t conversion(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max);


uint8_t		  rx_dma_irq = 0;
uint16_t	temp;
uint16_t a,i;
char dist_OLED[5];

// Main program

int main()
{
	SystemClock_Config();

	BSP_LED_Init();

	BSP_BT_Init();

	BSP_BUZZER_INIT();

	BSP_TIMER_PWM_Init();

	BSP_ULTRASONIC_ADC_INIT();

	BSP_NVIC_Init();

	BSP_I2C1_Init();

	BSP_OLED_Init();

	BSP_Console_Init();
	my_printf("Console Ready!\r\n");
	/*
	while(1)
	{
	  // Report TIM3 status (CNT, CCR1 and CCR2 registers)
	  my_printf("adc = %05d\r", a);

	  // Wait for 100ms
	  BSP_DELAY_ms(1000);

	  if (console_rx_byte == 1) {
	  BSP_PutNumber(USART1, a);
	  }
	}*/
	while(1)
	{
		sprintf(dist_OLED,"%d",conversion(a,300,4095,15,400));
		BSP_OLED_setXY(1,2);
		BSP_OLED_SendStr("Distance");
		BSP_OLED_setXY(4,2);
		BSP_OLED_SendStr(dist_OLED);
		if( conversion(a,300,4095,15,400) < 100 )
		{
			BSP_OLED_SendStr(" ");
		}
		BSP_OLED_setXY(4,6);
		BSP_OLED_SendStr("cm");
		my_printf("adc = %05d\r", conversion(a,300,4095,15,400));
		BSP_PutNumber(USART1,conversion(a,300,4095,15,400));
		//BSP_PutNumber(USART1,0x0A);
		BSP_DELAY_ms(500);
	}
}

/*
 * 	Clock configuration for the Nucleo STM32F072RB board
 * 	HSE input Bypass Mode 			-> 8MHz
 * 	SYSCLK, AHB, APB1 				-> 48MHz
 *  PA8 as MCO with /16 prescaler 	-> 3MHz
 */
uint16_t conversion(uint16_t x, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


static uint8_t SystemClock_Config()
{
	uint32_t	status;
	uint32_t	timeout;

	// Start HSE in Bypass Mode
	RCC->CR |= RCC_CR_HSEBYP;
	RCC->CR |= RCC_CR_HSEON;

	// Wait until HSE is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_HSERDY_Msk;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (1);	// HSE error


	// Select HSE as PLL input source
	RCC->CFGR &= ~RCC_CFGR_PLLSRC_Msk;
	RCC->CFGR |= (0x02 <<RCC_CFGR_PLLSRC_Pos);

	// Set PLL PREDIV to /1
	RCC->CFGR2 = 0x00000000;

	// Set PLL MUL to x6
	RCC->CFGR &= ~RCC_CFGR_PLLMUL_Msk;
	RCC->CFGR |= (0x04 <<RCC_CFGR_PLLMUL_Pos);

	// Enable the main PLL
	RCC-> CR |= RCC_CR_PLLON;

	// Wait until PLL is ready
	timeout = 1000;

	do
	{
		status = RCC->CR & RCC_CR_PLLRDY_Msk;
		timeout--;
	} while ((status == 0) && (timeout > 0));

	if (timeout == 0) return (2);	// PLL error


	// Set AHB prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_HPRE_Msk;
	RCC->CFGR |= RCC_CFGR_HPRE_DIV1;

	//Set APB1 prescaler to /1
	RCC->CFGR &= ~RCC_CFGR_PPRE_Msk;
	RCC->CFGR |= RCC_CFGR_PPRE_DIV1;

	// Enable FLASH Prefetch Buffer and set Flash Latency (required for high speed)
	FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

	// Select the main PLL as system clock source
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	// Wait until PLL becomes main switch input
	timeout = 1000;

	do
	{
		status = (RCC->CFGR & RCC_CFGR_SWS_Msk);
		timeout--;
	} while ((status != RCC_CFGR_SWS_PLL) && (timeout > 0));

	if (timeout == 0) return (3);	// SW error


	// Set MCO source as SYSCLK (48MHz)
	RCC->CFGR &= ~RCC_CFGR_MCO_Msk;
	RCC->CFGR |=  RCC_CFGR_MCOSEL_SYSCLK;

	// Set MCO prescaler to /16 -> 3MHz
	RCC->CFGR &= ~RCC_CFGR_MCOPRE_Msk;
	RCC->CFGR |=  RCC_CFGR_MCOPRE_DIV16;

	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA8 as Alternate function
	GPIOA->MODER &= ~GPIO_MODER_MODER8_Msk;
	GPIOA->MODER |= (0x02 <<GPIO_MODER_MODER8_Pos);

	// Set to AF0 (MCO output)
	GPIOA->AFR[1] &= ~(0x0000000F);
	GPIOA->AFR[1] |=  (0x00000000);

	// Update SystemCoreClock global variable
	SystemCoreClockUpdate();
	return (0);
}
