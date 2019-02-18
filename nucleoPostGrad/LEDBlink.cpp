#include <stm32l4xx.h>

#ifdef __cplusplus
extern "C"
#endif


#define DELAY 500
void TIM2_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
volatile bool buttonInterrupt = false;
volatile bool timer2RolledOver = false;


void configureGpioPorts(void);
void turn_on_all_leds(void);
void turn_off_all_leds(void);

void delayMs(int);
void delay1Hz(void);
void initTim2(void);
void initUserSw(void);

typedef enum
{
	NUCLEO_SW,
	SHIELD_SW1,
	SHIELD_SW2
} SWITCHES;
void initSwitch(SWITCHES);
void initUsart(void);
void initPbInterrupt(void);

void USART2_Write(uint8_t ch);

typedef enum
{
	FORWARD,
	BACK
}DIRECTIONS;
typedef enum
{
	IDLE_STATE,
	READ_ADC,
	LED2_ON_STATE,
	LED2_OFF_STATE,
	LED3_ON_STATE,
	LED3_OFF_STATE,
	LED4_ON_STATE,
	LED4_OFF_STATE,
	LED5_ON_STATE,
	LED5_OFF_STATE,
	LED6_ON_STATE,
	LED6_OFF_STATE,
	LED7_ON_STATE,
	LED7_OFF_STATE,
	LED8_ON_STATE,
	LED8_OFF_STATE,
	LED9_ON_STATE,
	LED9_OFF_STATE,
} STATES;

int main(void)
{
	STATES next_state = IDLE_STATE;
	DIRECTIONS direction = FORWARD;
	uint8_t sw1_pressed = 0;
	uint8_t sw1_pressed_prev = 0;
	uint8_t sw2_pressed = 0;
	uint8_t sw2_pressed_prev = 0;
	uint8_t inverted_cylon = 0;

	__disable_irq();
	/* Configure the clocks - using MSI as SYSCLK @16MHz */
	RCC->CR 		 	&= 	0xFFFFFF07;  //Clear ~MSIRANGE bits and MSIRGSEL bit
	RCC->CR 		 	|= 	0x00000089;  //Set MSI to 16MHz and MSIRGSEL bit

	RCC->AHB2ENR 	|= 	0x00000001;  //Enable PA5 clocks - for on-board LED
	
	/* Enable PA0 for output */
	GPIOA->MODER 	&= 	0xFFFFF3FF;  //Clear GPIOA[5] MODER bits
	GPIOA->MODER 	|= 	0x00000400;  //Enable GPIOA[5] for output
	
	/* Configure Timer 2
	Assumes 16MHz system clock, rolls over
	at 1Hz intervals */
	//initPbInterrupt();
	initUserSw();
	initUsart();
	configureGpioPorts();
	initTim2();
	
	
	__enable_irq();
	while (1)
	{
	
		//Check switch  state - consider debounce later
		if (!(GPIOA->IDR & 0x00000020))
		{
			sw1_pressed = 1;
			USART2_Write('5');
		}
		else sw1_pressed = 0;

		if (!(GPIOA->IDR & 0x00000040))
		{
			sw2_pressed = 1;
			USART2_Write('6');
		}
		else sw2_pressed = 0;
		
		if (timer2RolledOver == true)
		{

			
			USART2_Write('S');
			USART2_Write('e');
			USART2_Write('a');
			USART2_Write('n');
			USART2_Write('\n');
			timer2RolledOver = false;

			//Check for a change in the state of a switch
			if(sw1_pressed_prev != sw1_pressed)
				next_state = IDLE_STATE;

			if (sw2_pressed_prev != sw2_pressed)
				next_state = IDLE_STATE;


			sw1_pressed_prev = sw1_pressed;
			sw2_pressed_prev = sw2_pressed;

			switch (next_state) {

			case IDLE_STATE:	
				next_state = LED2_ON_STATE;
				if (sw1_pressed) {
					// inverted cyclon response
				   inverted_cylon =  sw2_pressed ? 0 : 1;
					if (sw2_pressed)
						turn_off_all_leds();
					else
						turn_on_all_leds();
				}
				else
					next_state = READ_ADC;

				break;
			case READ_ADC: turn_off_all_leds(); break;
								
			case LED2_ON_STATE: 
				next_state = LED2_OFF_STATE;
				GPIOA->BSRR |= inverted_cylon ? 0x04000000 : 0x00000400;  	//Set the GPIO
				break;
			case LED2_OFF_STATE: 
				next_state =  LED3_ON_STATE;
				GPIOA->BSRR |= inverted_cylon ? 0x00000400 : 0x04000000;  	//Set the GPIO
				direction = FORWARD; 
				break;
			case LED3_ON_STATE: 
				next_state = LED3_OFF_STATE;
				GPIOB->BSRR |= inverted_cylon ? 0x00080000: 0x00000008;  	//Set the GPIO
				break;
			case LED3_OFF_STATE: 
				next_state = direction == BACK ? LED2_ON_STATE : LED4_ON_STATE;
				GPIOB->BSRR |= inverted_cylon  ? 0x00000008 : 0x00080000;    //Clear the GPIO
				break;
			case LED4_ON_STATE: 
				next_state = LED4_OFF_STATE;
				GPIOB->BSRR |= inverted_cylon ? 0x00200000 : 0x00000020;  	//Set the GPIO
				break;
			case LED4_OFF_STATE: 
				next_state = direction == BACK ? LED3_ON_STATE : LED5_ON_STATE;
				GPIOB->BSRR |= inverted_cylon ? 0x00000020 : 0x00200000;    //Clear the GPIO
				break;
			case LED5_ON_STATE: 
				next_state = LED5_OFF_STATE;
				GPIOB->BSRR |= inverted_cylon ? 0x00100000 : 0x00000010;  	//Set the GPIO
				break;
			case LED5_OFF_STATE: 
				next_state = direction == BACK ? LED4_ON_STATE : LED6_ON_STATE;
				GPIOB->BSRR |= inverted_cylon ? 0x00000010 : 0x00100000;    //Clear the GPIO
				break;
			case LED6_ON_STATE: 
				next_state = LED6_OFF_STATE;
				GPIOB->BSRR |= inverted_cylon ? 0x04000000 : 0x00000400;  	//Set the GPIO
				break;
			case LED6_OFF_STATE: 
				next_state = direction == BACK ? LED5_ON_STATE : LED7_ON_STATE;
				GPIOB->BSRR |= inverted_cylon ? 0x00000400 : 0x04000000;    //Clear the GPIO
				break;
			case LED7_ON_STATE: 
				next_state = LED7_OFF_STATE;
				GPIOA->BSRR |= inverted_cylon ? 0x01000000 : 0x00000100;  	//Set the GPIO
				break;
			case LED7_OFF_STATE: 
				next_state = direction == BACK ? LED6_ON_STATE : LED8_ON_STATE;
				GPIOA->BSRR |= inverted_cylon ? 0x00000100 : 0x01000000;    //Clear the GPIO
				break;
			case LED8_ON_STATE: 
				next_state = LED8_OFF_STATE;
				GPIOC->BSRR |= inverted_cylon ? 0x00020000 : 0x00000002;  	//Set the GPIO
				break;
			case LED8_OFF_STATE: 
				next_state = direction == BACK ? LED7_ON_STATE : LED9_ON_STATE;
				GPIOC->BSRR |= inverted_cylon ? 0x00000002 : 0x00020000;    //Clear the GPIO
				break;
			case LED9_ON_STATE: 
				next_state = LED9_OFF_STATE;
				direction = BACK;
				GPIOC->BSRR |= inverted_cylon ? 0x00010000 : 0x00000001;  	//Set the GPIO
				break;
			case LED9_OFF_STATE: 
				next_state = direction == BACK ? LED8_ON_STATE : LED9_ON_STATE;
				GPIOC->BSRR |= inverted_cylon ? 0x00000001 : 0x00010000;    //Clear the GPIO
				direction = BACK;
				break;
				
			}
		}
	}
}

// delay in mS, off a sysTick - assumes HSI @ 16MHz

void delayMs(int n) {
	uint16_t i;
	/* Configure SysTick 
	Let sysclk (MSI) = 16MHz, 1/sysclk = 62.5nS  - for 1mS delay
	need X * 62.5nS = 0.001, so X= 16000 needs to be the reload value */
	
	SysTick->LOAD = 0x3E80 - 1; /*16000 -1 */
	SysTick->VAL = 0x00000000;  /*Clear the current value register */
	SysTick->CTRL = 0x5;        /*Enable internal clocks source and enable systick */
	for (i = 0;i < n;i++) {
		while ((SysTick->CTRL & 0x10000) == 0) /* wait for reload */
		{}
	}
	SysTick->CTRL = 0; 					//Stop the timer
}

void initTim2() {
	/* Interrupt for Timer 2 is #28
	 * 1/16MHz = 62.5nS -  * 16000 = 1mS
	 * Futher divide this by 1000, in the reload value
	 * to give a 1Hz rollover
	 *
	 * To rollover at 1/4Sec - make reload value 1000/4
	 **/

	RCC->APB1ENR1	|= 0x1;   		//Enable timer 2
	TIM2->PSC 		= 16000 - 1;	//Prescalar value - divide 16MHz by 16000
	TIM2->ARR 		= (1000) / 4 - 1;	//Reload value
	TIM2->CR1		= 1;			//enable timer
	TIM2->DIER      |= 1;			//enable interrupt
	NVIC_EnableIRQ(TIM2_IRQn);		//Enable the interrupt in the NVIC
}


void 	delay1Hz() {
	/* Use TIM2 to generate a 1Hz delay
	   Assumes 16MHz sysclk, divide this down by 1600
	   in the prescalar, and then set the reload value for
	   10000 to give the 1Hz - 1/16MHz = 62.5ns
	1600 * 62.5nS = 0.0001S = 100uS, *10000 = 1S
	*/
	TIM2->CNT 		= 0; 				//Clear the count register
	TIM2->CR1 		= 1;  				//Start the counter
	while(!(TIM2->SR & 1)) {}    		//wait for rollover
	TIM2->SR &= ~1; 					//Clear UIF bit
	TIM2->CR1 		= 0;  				//Stop the counter
}



void initUserSw() {
	/* Configure switch on the Nucleo board*/
	RCC->AHB2ENR 	|= 0x4; 			//ENABLE GPIOC port clk
	GPIOC->MODER 	&= 0xF3FFFFFF; 		//Clear GPIOC[13] for input mode

	/* Configure the 2 switches on the Arduino shield
	   The Pushbutton Switches on the board are connected as
		S1: D12 � PA6
		S2: D13 � PA5
		Switches are to ground - so need to enable pull-up's on them
		2-bits per port-bit, 0x1 is pull-up
	*/
	RCC->AHB2ENR 	|= 0x1; 			//ENABLE GPIOA port clk
	GPIOA->MODER 	&= 0xFFFFC3FF; 		//Clear GPIOA[6] & GPIOA[5] for input mode
	GPIOA->PUPDR	&= 0xFFFFC3FF;      //clear the PA5/6 pupdx bits
	GPIOA->PUPDR	|= 0x00001400;      //Enable the pull-ups
}

void initUsart() {
	/* USART2 
		Leave prescalars at defaults - no divide from system clock
		system clock is set to 16Mhz
		PA3 - Rx (D0)
		PA2 - Tx (D1)
		RCC_APB1ENR1[17] -usart2 enable (APB1 clk enable)
		SysClk(MSI) -> AHB PRES (1,2,,,512) -> APB1 PRESC (1,2..16)
		RCC_CFG - contains the pre-scalers 
			AHB ->RCC_CFG[7:4]0,2,4...512]
			APB1->RCC_CFG[31:11]0,2,4,8,16]
		BAUD-RATE - USART_BRR - TxRXBaud = (2 * sysClk)/USART_DIV)
			let BR=115200 - oversampling by 16
			USART_DIV = (2 * 16MHz)/115200 => 277.777 = 0x115
			USART_BRR[15:4] = Mantissa (0x115)
			USART_BRR[3:0] = Fraction = .777 * 16 = 12.44 - use 12 = 0B
			USART_BRR = 0x115B
		AF's PA2 - USART2 TX ->AF7, PA3 USART2 RX AF7
     */

	RCC->AHB2ENR 	|= RCC_AHB2RSTR_GPIOARST_Msk;	//Enable the clock for GPIOA
	RCC->APB1ENR1	|= RCC_APB1ENR1_USART2EN_Msk;	//Enable the USART2 clock
	GPIOA->MODER 	&= 0xFFFFFF0F; 					//Clear PA2/3 bits
	GPIOA->MODER 	|= 0x000000A0; 					//Set PA2/3 for AF mode
	GPIOA->AFR[0]   &= 0xFFFF00FF;					//Clear the AF bits for 2&3
	GPIOA->AFR[0]   |= 0x00007700;					//Set both AF modes to AF7, for bits 2/3
	//USART2->BRR		|= 0x115B;						//Set BAUD Rate to 115200 with UartClk at 16MHz
	USART2->BRR		&= 0x0000;						//Set BAUD Rate to 115200 with UartClk at 16MHz
	USART2->BRR		|= 0x008B;						//Set BAUD Rate to 115200 with UartClk at 16MHz
	USART2->CR1     &= 0xEFFF6FFE;					//Clear the M1,OVER8,M0 bits, set 1 start-bit, 8-data bits n stop-bits, keep UE low
	USART2->CR2     &= 0xFFFFC000;					//Clear the stop-bits to give 1 stop-bit (default anyway)
	USART2->CR1     |= 0x00000009;					//Enable the TX and the uart

}

void initPbInterrupt() {
	
	RCC->APB2ENR        |= 1; 			//Enable SYSCFG clk (for GPIO interrupt enables)
	SYSCFG->EXTICR[3] 	&= ~0x00F0; 	//CLEAR_BIT the EXTI[13] bits
	SYSCFG->EXTICR[3]   |= 0x20; 		//Enable GPIOC for EXTI[13]
	EXTI->IMR1 			|= 0x2000; 	//Unmask EXTI13
	EXTI->FTSR1 		|= 0x2000;  //Enable falling edge triggered interrupts (pushbutton high to low on push)
	NVIC_EnableIRQ(EXTI15_10_IRQn); //Enable EXTI15-to-10 interrupts
}
	 

void turn_on_all_leds(void)
{
/*
	LED2 - D2 - PA10
	LED3 - D3 - PB3
	LED4 - D4 - PB5
	LED5 - D5 - PB4
	LED6 - D6 - PB10
	LED7 - D7 - PA8
	LED8 - A5 - PC1
	LED9 - A4 - PC0
 */

	GPIOA->BSRR |= 0x00000500;  	//Set LED2/7
	GPIOB->BSRR |= 0x00000438;  	//Set LED3/4/5/6
	GPIOC->BSRR |= 0x00000003;  	//Set LED8/9

}

void turn_off_all_leds(void)
{
/*
	LED2 - D2 - PA10
	LED3 - D3 - PB3
	LED4 - D4 - PB5
	LED5 - D5 - PB4
	LED6 - D6 - PB10
	LED7 - D7 - PA8
	LED8 - A5 - PC1
	LED9 - A4 - PC0
 */


	GPIOA->BSRR |= 0x05000000;  	//Set LED2/7
	GPIOB->BSRR |= 0x04380000;  	//Set LED3/4/5/6
	GPIOC->BSRR |= 0x00030000;   	//Set LED8/9

}

void configureGpioPorts()
{
	/*
	The LEDs are connected as follows:

	LED0, LED1 not connected
	LED2 - D2 - PA10
	LED3 - D3 - PB3
	LED4 - D4 - PB5
	LED5 - D5 - PB4
	LED6 - D6 - PB10
	LED7 - D7 - PA8
	LED8 - A5 - PC1
	LED9 - A4 - PC0
	*/

	RCC->AHB2ENR 	|= 	0x00000007;  //Enable PA,B,C clocks
	
	/* Enable PA8, PA10 for output */
	GPIOA->MODER 	&= 	0xFFCCFFFF;  //Clear GPIOA[10,8] MODER bits
	GPIOA->MODER 	|= 	0x00110000;  //Enable GPIOA[10,8] MODER  bits for output
	/* Enable PA3,PB4,PB5,PB10 for output */
	GPIOB->MODER 	&= 	0xFFCFF03F;  //Clear GPIOB[10,5,4,3] MODER bits
	GPIOB->MODER 	|= 	0x00100540;  //Enable GPIOB[10,5,4,3] MODER bits for output
	/* Enable PC0, PC1 for output */
	GPIOC->MODER 	&= 	0xFFFFFFF0;  //Clear GPIOC[1,0] MODER bits
	GPIOC->MODER 	|= 	0x00000005;  //Enable GPIOC[1,0] MODER  bits for output

}

void USART2_Write(uint8_t ch) {
	
	while (!(USART2->ISR & 0x0080)) {}
	USART2->TDR = ch;
}

/* Handle PC13 (bushbutton) interrupt */
void EXTI15_10_IRQHandler(void) {
	buttonInterrupt = true;
	EXTI->PR1 = 0x2000;
}

void TIM2_IRQHandler(void)
{
	TIM2->SR = 0;
	timer2RolledOver = true;
}
