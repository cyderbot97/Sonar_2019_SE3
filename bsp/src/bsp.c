#include "bsp.h"

void BSP_NVIC_Init()
{
	// Set priority level 0 for TIM16 interrupt
	NVIC_SetPriority(TIM16_IRQn, 0);

	// Enable TIM16 interrupts
	NVIC_EnableIRQ(TIM16_IRQn);

	// Set priority level 1 for ADC1 interrupt
	NVIC_SetPriority(ADC1_IRQn, 1);

	// Enable ADC1 interrupts
	NVIC_EnableIRQ(ADC1_IRQn);

	// Set priority level 2 for USART1 interrupt
	NVIC_SetPriority(USART1_IRQn, 2);

	// Enable USART1 interrupts
	NVIC_EnableIRQ(USART1_IRQn);
}

void BSP_BUZZER_INIT()
{

	RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;

	TIM16->CR1 = 0x0000;
	TIM16->CR2 = 0x0000;

	TIM16->PSC = (uint16_t) 48000 -1;

	TIM16->ARR = (uint16_t) 500 -1;

	TIM16->CCMR1 |= (0x06 <<TIM_CCMR1_OC1M_Pos);// | TIM_CCMR1_OC1PE;

	TIM16->CCR1 = 200;

	TIM16->CR1 |= TIM_CR1_ARPE;

	TIM16->DIER |= TIM_DIER_UIE;
	TIM16->DIER |= TIM_DIER_CC1IE;


	TIM16->CCER |= TIM_CCER_CC1E;
	TIM16->CR1 |= TIM_CR1_CEN;

}

/*
 * Set up PWM output for buzzer
 */

void BSP_TIMER_PWM_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA8 as Alternate Function
	GPIOA->MODER &= ~GPIO_MODER_MODER8_Msk;
	GPIOA->MODER |=  (0x02 <<GPIO_MODER_MODER8_Pos);

	// Set PA8 to AF2 (TIM1)
	GPIOA->AFR[1] &= ~(0x0000000F);
	GPIOA->AFR[1] |=  (0x00000002);

	// Enable TIM1 clock
	RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;

	// Reset TIM1 configuration
	TIM1->CR1  = 0x0000;
	TIM1->CR2  = 0x0000;
	TIM1->CCER = 0x0000;

	// Set TIM1 prescaler
	// Fck = 48MHz -> /48 = 1MHz counting frequency (1µs resolution)
	TIM1->PSC = (uint16_t) 48 -1;

	// Set Auto-Reload to period = 500ms
	TIM1->ARR = (uint16_t) 500;

	// Enable Auto-Reload Preload register
	TIM1->CR1 |= TIM_CR1_ARPE;

	// Setup Input Capture
	TIM1->CCMR1 = 0x0000;
	TIM1->CCMR2 = 0x0000;

	// Setup PWM mode 1 output
	TIM1->CCMR1 |= (0x06 <<TIM_CCMR1_OC1M_Pos) | TIM_CCMR1_OC1PE;

	// Set default PWM values
	TIM1->CCR1 = 250;

	// Enable Outputs
	TIM1->CCER |= TIM_CCER_CC1E;

	// Enable Main output
	TIM1->BDTR |= TIM_BDTR_MOE;

	// Enable TIM1
	TIM1->CR1 |= TIM_CR1_CEN;
}

void BSP_Console_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA2 and PA3 as Alternate function
	GPIOA->MODER &= ~(GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk);
	GPIOA->MODER |=  (0x02 <<GPIO_MODER_MODER2_Pos) | (0x02 <<GPIO_MODER_MODER3_Pos);

	// Set PA2 and PA3 to AF1 (USART2)
	GPIOA->AFR[0] &= ~(0x0000FF00);
	GPIOA->AFR[0] |=  (0x00001100);

	// Enable USART2 clock
	RCC -> APB1ENR |= RCC_APB1ENR_USART2EN;

	// Clear USART2 configuration (reset state)
	// 8-bit, 1 start, 1 stop, CTS/RTS disabled
	USART2->CR1 = 0x00000000;
	USART2->CR2 = 0x00000000;
	USART2->CR3 = 0x00000000;

	// Select PCLK (APB1) as clock source
	// PCLK -> 48 MHz
	RCC->CFGR3 &= ~RCC_CFGR3_USART2SW_Msk;

	// Baud Rate = 115200
	// With OVER8=0 and Fck=48MHz, USARTDIV =   48E6/115200 = 416.6666
	// BRR = 417 -> Actual BaudRate = 115107.9137 -> 0.08% error
	//
	// With OVER8=1 and Fck=48MHz, USARTDIV = 2*48E6/115200 = 833.3333
	// BRR = 833 -> Actual BaudRate = 115246.0984 -> 0.04% error (better choice)

	USART2->CR1 |= USART_CR1_OVER8;
	USART2->BRR = 833;

	// Enable both Transmitter and Receiver
	USART2->CR1 |= USART_CR1_TE | USART_CR1_RE;

	// Enable USART2
	USART2->CR1 |= USART_CR1_UE;
}
void BSP_LED_Init()
{
	// Enable GPIOA clock
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

	// Configure PA5 as output
	GPIOA->MODER &= ~GPIO_MODER_MODER5_Msk;
	GPIOA->MODER |= (0x01 <<GPIO_MODER_MODER5_Pos);

	// Configure PA5 as Push-Pull output
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5;

	// Configure PA5 as High-Speed Output
	GPIOA->OSPEEDR &= ~GPIO_OSPEEDR_OSPEEDR5_Msk;
	GPIOA->OSPEEDR |= (0x03 <<GPIO_OSPEEDR_OSPEEDR5_Pos);

	// Disable PA5 Pull-up/Pull-down
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPDR5_Msk;

	// Set Initial State OFF
	GPIOA->BSRR = GPIO_BSRR_BR_5;
}

void BSP_LED_Toggle()
{
	GPIOA->ODR ^= GPIO_ODR_5;
}

/*
 * BSP_PB_Init()
 * Initialize Push-Button pin (PC13) as input without Pull-up/Pull-down
 */

void BSP_PB_Init()
{
	// Enable GPIOC clock
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;

	// Configure PC13 as input
	GPIOC->MODER &= ~GPIO_MODER_MODER13_Msk;
	GPIOC->MODER |= (0x00 <<GPIO_MODER_MODER13_Pos);

	// Disable PC13 Pull-up/Pull-down
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR13_Msk;
}

void BSP_TIMER_Timebase_Init()
{
	// Enable TIM6 clock
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

	// Reset TIM6 configuration
	TIM6->CR1 = 0x0000;
	TIM6->CR2 = 0x0000;

	// Set TIM6 prescaler
	// Fck = 48MHz -> /48000 = 1KHz counting frequency
	TIM6->PSC = (uint16_t) 48000 -1;

	// Set TIM6 auto-reload register for 1s
	TIM6->ARR = (uint16_t) 1000 -1;

	// Enable auto-reload preload
	TIM6->CR1 |= TIM_CR1_ARPE;

	// Start TIM6 counter
	TIM6->CR1 |= TIM_CR1_CEN;
}

void BSP_LED_On()
{
	GPIOA->BSRR |= GPIO_BSRR_BS_5;
}

/*
 * BSP_LED_Off()
 * Turn OFF LED on PA5
 */

void BSP_LED_Off()
{
	GPIOA->BSRR |= GPIO_BSRR_BR_5;
}

