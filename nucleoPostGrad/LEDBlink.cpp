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

void delayMs(int);
void delay1Hz(void);
void initTim2(void);
void initUserSw(void);
void initPbInterrupt(void);


typedef enum
{
	FORWARD,
	BACK
}DIRECTIONS;
typedef enum
{
	IDLE_STATE,
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
	__disable_irq();
	/* Configure the clocks - using MSI as SYSCLK @16MHz */
	RCC->CR 		 	&= 	0xFFFFFF07;  //Clear ~MSIRANGE bits and MSIRGSEL bit
	RCC->CR 		 	|= 	0x00000088;  //Set MSI to 16MHz and MSIRGSEL bit

	RCC->AHB2ENR 	|= 	0x00000001;  //Enable PA5 clocks - for on-board LED
	
	/* Enable PA0 for output */
	GPIOA->MODER 	&= 	0xFFFFF3FF;  //Clear GPIOA[5] MODER bits
	GPIOA->MODER 	|= 	0x00000400;  //Enable GPIOA[5] for output
	
	/* Configure Timer 2
	Assumes 16MHz system clock, rolls over
	at 1Hz intervals */
	//initPbInterrupt();
	initUserSw();
	configureGpioPorts();
	initTim2();
	
	
  __enable_irq();
	while (1)
	{
		
		if (timer2RolledOver == true)
		{
			timer2RolledOver = false;
			switch (next_state) {

			case IDLE_STATE:	
				next_state = LED2_ON_STATE;
				break;
								
			case LED2_ON_STATE: 
				next_state = LED2_OFF_STATE;
				GPIOA->ODR |= 0x00000400;  	//Set the GPIO
				break;
			case LED2_OFF_STATE: 
				next_state =  LED3_ON_STATE;
				GPIOA->BRR |= 0x00000400;  	//Set the GPIO
				direction = FORWARD; 
				break;
			case LED3_ON_STATE: 
				next_state = LED3_OFF_STATE;
				GPIOB->BSRR |= 0x00000008;  	//Set the GPIO
				break;
			case LED3_OFF_STATE: 
				next_state = direction == BACK ? LED2_ON_STATE : LED4_ON_STATE;
				GPIOB->BSRR |= 0x00080000;    //Clear the GPIO
				break;
			case LED4_ON_STATE: 
				next_state = LED4_OFF_STATE;
				GPIOB->BSRR |= 0x00000020;  	//Set the GPIO
				break;
			case LED4_OFF_STATE: 
				next_state = direction == BACK ? LED3_ON_STATE : LED5_ON_STATE;
				GPIOB->BSRR |= 0x00200000;    //Clear the GPIO
				break;
			case LED5_ON_STATE: 
				next_state = LED5_OFF_STATE;
				GPIOB->BSRR |= 0x00000010;  	//Set the GPIO
				break;
			case LED5_OFF_STATE: 
				next_state = direction == BACK ? LED4_ON_STATE : LED6_ON_STATE;
				GPIOB->BSRR |= 0x00100000;    //Clear the GPIO
				break;
			case LED6_ON_STATE: 
				next_state = LED6_OFF_STATE;
				GPIOB->BSRR |= 0x00000400;  	//Set the GPIO
				break;
			case LED6_OFF_STATE: 
				next_state = direction == BACK ? LED5_ON_STATE : LED7_ON_STATE;
				GPIOB->BSRR |= 0x04000000;    //Clear the GPIO
				break;
			case LED7_ON_STATE: 
				next_state = LED7_OFF_STATE;
				GPIOA->BSRR |= 0x00000100;  	//Set the GPIO
				break;
			case LED7_OFF_STATE: 
				next_state = direction == BACK ? LED6_ON_STATE : LED8_ON_STATE;
				GPIOA->BSRR |= 0x01000000;    //Clear the GPIO
				break;
			case LED8_ON_STATE: 
				next_state = LED8_OFF_STATE;
				GPIOC->BSRR |= 0x00000002;  	//Set the GPIO
				break;
			case LED8_OFF_STATE: 
				next_state = direction == BACK ? LED7_ON_STATE : LED9_ON_STATE;
				GPIOC->BSRR |= 0x00020000;    //Clear the GPIO
				break;
			case LED9_ON_STATE: 
				next_state = LED9_OFF_STATE;
				direction = BACK;
				GPIOC->BSRR |= 0x00000001;  	//Set the GPIO
				break;
			case LED9_OFF_STATE: 
				next_state = direction == BACK ? LED8_ON_STATE: LED9_ON_STATE;
				GPIOC->BSRR |= 0x00010000;    //Clear the GPIO
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
	/* Interrupt for Timer 2 is #28*/
	RCC->APB1ENR1	|= 0x1;   		//Enable timer 2
	TIM2->PSC 		= 16000 - 1;	//Prescalar value - divide 16MHz by 16000
	TIM2->ARR 		= 1000 - 1;		//Reload value
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
	RCC->AHB2ENR 	|= 0x4; 			//ENABLE GPIOC port clk
	GPIOC->MODER 	&= 0xF3FFFFFF; 		//Clear GPIOC[13] for input mode
}

void initPbInterrupt() {
	
	RCC->APB2ENR        |= 1; 			//Enable SYSCFG clk (for GPIO interrupt enables)
	SYSCFG->EXTICR[3] 	&= ~0x00F0; 	//CLEAR_BIT the EXTI[13] bits
	SYSCFG->EXTICR[3]   |= 0x20; 		//Enable GPIOC for EXTI[13]
	EXTI->IMR1 			|= 0x2000; 	//Unmask EXTI13
	EXTI->FTSR1 		|= 0x2000;  //Enable falling edge triggered interrupts (pushbutton high to low on push)
	NVIC_EnableIRQ(EXTI15_10_IRQn); //Enable EXTI15-to-10 interrupts
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
