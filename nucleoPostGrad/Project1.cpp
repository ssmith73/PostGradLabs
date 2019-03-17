#include <stm32l4xx.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#ifdef __cplusplus
//extern "C"
#endif

#define VREF 3.3
extern "C" void TIM2_IRQHandler();
extern "C" void ADC1_2_IRQHandler();
extern "C" void USART2_IRQHandler();
void configureGpioPorts(void);
void write_string(char *);

void delayMs(int);
void delay1Hz(void);
void initTim2(void);
void initUserSw(void);

void led_control(uint8_t);
void initAdc(void);

typedef enum
{
	NUCLEO_SW,
	SHIELD_SW1,
	SHIELD_SW2
} SWITCHES;

typedef enum {
	NORMAL,
	INVERSE
}DISPLAYMODES;

typedef enum
{
	FORWARD,
	BACK
}DIRECTIONS;
typedef enum
{
	IDLE_STATE,
	CHECK_FOR_NEW_CHAR,
	SINGLE_ADC_VALUE,
	OUTPUT_PERIOD_555,
	OUTPUT_HIGH_TIME_555,
	OUTPUT_LOW_TIME_555,
	OUTPUT_FREQ_555,
	CONTINUOUS_ADC_VALUES,
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

void initSwitch(SWITCHES);
void initUsart(void);
void USART2_Write();

void initPbInterrupt(void); //not used

// Globals for ISR's
int volatile adcResult = 0;
volatile char rxChar;
volatile bool buttonInterrupt = false;
volatile bool adcComplete = false;
volatile bool newChar = false;
volatile bool timer2RolledOver = false;
char *str;

int main(void)
{
	
	STATES next_state = IDLE_STATE;
	DIRECTIONS direction = FORWARD;
	uint8_t sw1_pressed = 0;
	uint8_t sw1_pressed_prev = 0;
	uint8_t inverted_cylon = 0;
	bool returnVAdc = true;
	bool continuousAdc = false;
	float VAdc = 0;

	__disable_irq();
	/* Configure the system clock - using MSI as SYSCLK @16MHz */
	RCC->CR 		&= 	0xFFFFFF07;        //Clear ~MSIRANGE bits and MSIRGSEL bit
	RCC->CR 		|= 	0x00000089;        //Set MSI to 16MHz and MSIRGSEL bit

	//Enable PA5 clocks - for on-board LED (Nucleo)
	RCC->AHB2ENR 	|= 	0x00000001;		   //Enable GPIO port A clk        
	GPIOA->MODER 	&= 	0xFFFFF3FF;        //Clear  GPIOA[5] MODER bits
	GPIOA->MODER 	|= 	0x00000400;        //Enable GPIOA[5] for output
	
	initUserSw();							//Enable SW1
	initUsart();
	configureGpioPorts();
	initTim2();
	initAdc();

	DISPLAYMODES display_mode = NORMAL;	
	str = (char *) malloc(25);
	strcpy(str, "Program Start\n");

	__enable_irq();
	while (1)
	{

		/* iF there is data in the string buffer - send it out on the UART*/

		if(*str != '\0')
			NVIC_EnableIRQ(USART2_IRQn);
		else
			NVIC_DisableIRQ(USART2_IRQn);
		
		if (timer2RolledOver == true)
		{
			timer2RolledOver = false;

			/* Basic state machine to handle the LED toggling
			   Operates at 0.25Hz, using a timer interrupt
			 */

			switch (next_state) {

			case IDLE_STATE:	
				next_state = CHECK_FOR_NEW_CHAR;
				break;
			case CHECK_FOR_NEW_CHAR: {
				//check for a new character
				if(USART2->ISR & 0x00000020)
				{
					rxChar = USART2->RDR;
					str[0] = rxChar;
					str[1] = '\0';
				}
				else
					USART2->ICR |= 0x00000008; //Clear the overrun error flag

				switch (rxChar)
				{
					case 'a': case 'A': { next_state = READ_ADC; returnVAdc = false; break; }
					case 'v': case 'V': { next_state = READ_ADC; returnVAdc = true; break; }
					case 'c': case 'C': { next_state = READ_ADC; continuousAdc = true; break; }
					case 'e': case 'E': { next_state = READ_ADC; continuousAdc = false ; break; }
					case 's': {next_state = IDLE_STATE;strcpy(str, "You pressed 's'\n");break;}
					default: {
						break;
					}
				}
				
				break;
				
			}

			case READ_ADC: {
					char dataPtr[50];

					next_state = IDLE_STATE;
				
					if(adcComplete == true)
					{
						if (returnVAdc == true)
							sprintf(dataPtr, "ADC-result %f\n", VAdc = (VREF / (4096 - 1))*adcResult);
						else
							sprintf(dataPtr, "ADC-result %d\n", adcResult);
						
						strcpy(str, dataPtr);
						adcComplete = false;
					}
					else
						ADC1->CR |= 0x00000004;            //Convst

				break;
				}
			}
		}
	}
}


void delayMs(int n) {
	// delay in mS, off a sysTick - assumes HSI @ 16MHz
	uint16_t i;
	/* Configure SysTick 
	Let sysclk (MSI) = 16MHz, 2/sysclk = 62.5nS  - for 1mS delay
	need X * 62.5nS = 0.001, so X= 16000 needs to be the reload value */
	
	SysTick->LOAD = 0x3E80 - 1; /*16000 -1 */
	SysTick->VAL = 0x00000000; /*Clear the current value register */
	SysTick->CTRL = 0x5; /*Enable internal clocks source and enable systick */
	for (i = 0; i < n; i++) {
		while ((SysTick->CTRL & 0x10000) == 0) /* wait for reload */
		{}
	}
	SysTick->CTRL = 0;       					//Stop the timer
}

void initTim2() {
	/* Interrupt for Timer 2 is #28
	 * 1/16MHz = 62.5nS -  * 16000 = 1mS
	 * Futher divide this by 1000, in the reload value
	 * to give a 1Hz rollover
	 *
	 * To rollover at 1/4Sec - make reload value 1000/4
	 * but asked to move LED's at 0.25s - flash on/off
	 * should be half this - so /8
	 * Priority of interrupt may have to be changed to give
	 * this interrupt a higher priority to ensure delays actually work
	 **/

	RCC->APB1ENR1	|= 0x1;        		//Enable timer 2
	TIM2->PSC 		= 16000 - 1;      	//Prescalar value - divide 16MHz by 16000
	TIM2->ARR 		= (1000 / 8) - 1;  	//Reload value
	TIM2->CR1		= 1;      			//enable timer
	TIM2->DIER      |= 1;      			//enable interrupt
	NVIC_EnableIRQ(TIM2_IRQn);      	//Enable the interrupt in the NVIC
}


void 	delay1Hz() {
	/* Use TIM2 to generate a 1Hz delay
	   Assumes 16MHz sysclk, divide this down by 1600
	   in the prescalar, and then set the reload value for
	   10000 to give the 1Hz - 1/16MHz = 62.5ns
	   1600 * 62.5nS = 0.0001S = 100uS, *10000 = 1S
	   Not actually using this for this lab, only 
	   written to test the timers, interrupt is a better way
	*/
	TIM2->CNT 		= 0;       	//Clear the count register
	TIM2->CR1 		= 1;        //Start the counter
	while(!(TIM2->SR & 1)) {}   //wait for rollover
	TIM2->SR &= ~1;       		//Clear UIF bit
	TIM2->CR1 		= 0;        //Stop the counter
}


void initUserSw() {
	/* Configure switch on the Nucleo board*/
	RCC->AHB2ENR 	|= 0x4;       		//ENABLE GPIOC port clk
	GPIOC->MODER 	&= 0xF3FFFFFF;      //Clear GPIOC[13] for input mode

	/* Configure the 2 switches on the Arduino shield
	   The Pushbutton Switches on the board are connected as
		S1: D12 � PA6
		S2: D13 � PA5
		Switches are to ground - so need to enable pull-up's on them
		2-bits per port-bit, 0x1 is pull-up
		S2 won't actually be used as it's conflicting with the 
		LED on the Nucleo board - also connected to PA5
	*/
	RCC->AHB2ENR 	|= 0x1;       		//ENABLE GPIOA port clk
	GPIOA->MODER 	&= 0xFFFFCFFF;      //Clear GPIOA[6] for input mode
	GPIOA->PUPDR	&= 0xFFFFCFFF;      //clear the PA6 pupdx bits
	GPIOA->PUPDR	|= 0x00001000;      //Enable the pull-up on PA6
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
			USART_DIV = 16MHz/115200 => 138.88 - say 139 = 0x8B
			USART_BRR = 0x8B
		AF's PA2 - USART2 TX ->AF7, PA3 USART2 RX AF7
	 */

	RCC->AHB2ENR 	|= RCC_AHB2RSTR_GPIOARST_Msk;	//Enable the clock for GPIOA
	RCC->APB1ENR1	|= RCC_APB1ENR1_USART2EN_Msk;  	//Enable the USART2 clock
	GPIOA->MODER 	&= 0xFFFFFF0F;       			//Clear PA2/3 bits
	GPIOA->MODER 	|= 0x000000A0;       			//Set PA2/3 for AF mode
	GPIOA->AFR[0]   &= 0xFFFF00FF;      			//Clear the AF bits for 2&3
	GPIOA->AFR[0]   |= 0x00007700;      			//Set both AF modes to AF7, for bits 2/3
	USART2->BRR		&= 0x0000;      				//clear BAUD Rate bits 
	USART2->BRR		|= 0x008B;      				//Set BAUD Rate to 115200 with UartClk at 16MHz
	USART2->CR1     &= 0xEFFF6FFE;      			//Clear the M1,OVER8,M0 bits, set 1 start-bit, 8-data bits n stop-bits, keep UE low
	USART2->CR2     &= 0xFFFFC000;      			//Clear the stop-bits to give 1 stop-bit (default anyway)
	USART2->CR1     |= 0x0000004D;      			//Enable Transmit-complete interrupt, RX/TX and the uart itself


}

void initPbInterrupt() {
	
	RCC->APB2ENR        |= 1;       	//Enable SYSCFG clk (for GPIO interrupt enables)
	SYSCFG->EXTICR[3] 	&= ~0x00F0;     //CLEAR_BIT the EXTI[13] bits
	SYSCFG->EXTICR[3]   |= 0x20;       	//Enable GPIOC for EXTI[13]
	EXTI->IMR1 			|= 0x2000;      //Unmask EXTI13
	EXTI->FTSR1 		|= 0x2000;      //Enable falling edge triggered interrupts (pushbutton high to low on push)
	NVIC_EnableIRQ(EXTI15_10_IRQn);     //Enable EXTI15-to-10 interrupts
}
	 
void initAdc(void)
{

	 /* ADC input is on PA4 - Page 73 of datasheet, 
	  *   alternate-function of PA4 -> ADC12_IN9
	  * Controlled with an interrupt
	  * Need to enable EOC interrupt - ADC_IER[2]
	  * 
	  *   alternate-function of PA0 -> ADC12_IN5
		 //ADC1->SMPR1  |= 0x00008000;         //Add a little more sampling time to channel 5 (6.5 ADC clk cycles)
		 //GPIOA->ASCR	|= 0x00000001;         //Connect analog switch to GPIOA[0]
		 //GPIOA->MODER	|= 0x00000003;         //Set A0 for analog input mode  - actually reset to analog input mode
		 //ADC1->SQR1	|= 0x00000140;         //Set for a sequence of 1 conversion on CH5
	  */
	 RCC->AHB2ENR	|= 0x00000001;         //Enable GPIOA CLK
	 RCC->CCIPR		|= 0x30000000;         //Select SYSCLK as ADC clk source
	 RCC->AHB2ENR	|= 0x00002000;         //Enable the ADC clock
	  
	 ADC1->CR		&= 0xDFFFFFFF;         //Take ADC out of deep power down
	 delayMs(1); 						   //Allow 1mS  - only needs Tadcvreg_stup - 20uS (datasheet p178)
	 GPIOA->ASCR	|= 0x00000010;         //Connect analog switch to GPIOA[4]
	 GPIOA->MODER	|= 0x00000300;         //Set A4 for analog input mode  - actually reset to analog input mode
	 ADC1->CR		|= 0x10000000;         //Enable ADC1 votage regulator
	 //ADC123_COMMON->CCR |= 0x00400000;     //Enable Vrefint

	 ADC1->IER		|= 0x00000004;         //Enable ADC1 EOC interrupt
	 NVIC_EnableIRQ(ADC1_2_IRQn);			   //Enable interrupts on ADC1

	 ADC1->SMPR1    |= 0x08008000;         //Add a little more sampling time to channel 9 (6.5 ADC clk cycles)
	 ADC1->ISR		|= 0x00000001;         //Clear the ADRDY bit in the ADCx_ISR register by writing �1�.
	 ADC1->SQR1		|= 0x00000240;         //Set for a sequence of 1 conversion on CH9 _01001_00_0000
	 ADC1->CR		|= 0x00000001;         //Enable ADC1
	 
}

void led_control(uint8_t mask)
{
	/*
	  Each bit of mask represents an led
	  8'b11001100 will turn on 4 led's 

		LED2 - D2 - PA10
		LED3 - D3 - PB3
		LED4 - D4 - PB5
		LED5 - D5 - PB4
		LED6 - D6 - PB10
		LED7 - D7 - PA8
		LED8 - A5 - PC1
		LED9 - A4 - PC0
	 */
	switch(mask)
	{
	case 0x01: {
			GPIOA->BSRR |= 0x00000400;  		//Set LED2
			break;
		}
	case 0x03: {
			GPIOA->BSRR |= 0x00000400;  		//Set LED2
			GPIOB->BSRR |= 0x00000008;      	//Set LED3
			break;
		}
	case 0x07: {
			GPIOA->BSRR |= 0x00000400;  		//Set LED2
			GPIOB->BSRR |= 0x00000028;      	//Set LED3/4
			break;
		}
	case 0x0F: {
			GPIOA->BSRR |= 0x00000400;  		//Set LED2
			GPIOB->BSRR |= 0x00000038;      	//Set LED3/4/5
			break;
		}
	case 0x1F: {
			GPIOA->BSRR |= 0x00000400;  		//Set LED2
			GPIOB->BSRR |= 0x00000438;      	//Set LED3/4/5/6
			break;
		}
	case 0x3F: {
			GPIOA->BSRR |= 0x00000500;  		//Set LED2/7
			GPIOB->BSRR |= 0x00000438;      	//Set LED3/4/5/6
			break;
		}
	case 0x7F: {
			GPIOA->BSRR |= 0x00000500;  		//Set LED2/7
			GPIOB->BSRR |= 0x00000438;      	//Set LED3/4/5/6
			GPIOC->BSRR |= 0x00000002;       	//Set LED8
			break;
		}
	case 0xFF: {
			GPIOA->BSRR |= 0x00000500;  		//Set LED2/7
			GPIOB->BSRR |= 0x00000438;      	//Set LED3/4/5/6
			GPIOC->BSRR |= 0x00000003;      	//Set LED8/9
			break;
		}
	default: {
			 //all off
		GPIOA->BSRR |= 0x05000000;      	//Clear LED2/7
		GPIOB->BSRR |= 0x04380000;      	//Clear LED3/4/5/6
		GPIOC->BSRR |= 0x00030000;      	//Clear LED8/9
		break;
		}
	}

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

	RCC->AHB2ENR 	|= 	0x00000007;        //Enable PA,B,C clocks
	
	/* Enable PA8, PA10 for output */
	GPIOA->MODER 	&= 	0xFFCCFFFF;        //Clear GPIOA[10,8] MODER bits
	GPIOA->MODER 	|= 	0x00110000;        //Enable GPIOA[10,8] MODER  bits for output
	/* Enable PA3,PB4,PB5,PB10 for output */
	GPIOB->MODER 	&= 	0xFFCFF03F;        //Clear GPIOB[10,5,4,3] MODER bits
	GPIOB->MODER 	|= 	0x00100540;        //Enable GPIOB[10,5,4,3] MODER bits for output
	/* Enable PC0, PC1 for output */
	GPIOC->MODER 	&= 	0xFFFFFFF0;        //Clear GPIOC[1,0] MODER bits
	GPIOC->MODER 	|= 	0x00000005;        //Enable GPIOC[1,0] MODER  bits for output

}

void write_string(char *str)
{
	/***** Simple pointer manipulation to 
		   write characters in a string to the UART *****/
	NVIC_EnableIRQ(USART2_IRQn);					//Enable interrupts for this usart
	while (*str++)
		//USART2_Write(*str++);
	NVIC_DisableIRQ(USART2_IRQn);					//Enable interrupts for this usart
		
}
void USART2_Write() {
	/**** Wait for USART to clear and send a single character *****/
//	while (!(USART2->ISR & 0x0080)) {}
	//USART2->TDR = ch;
}

void EXTI15_10_IRQHandler(void) {
/**** Handle PC13 (bushbutton) interrupt 
 **** Not used in this lab - just a test ****/
	buttonInterrupt = true;
	EXTI->PR1 = 0x2000;   //clear pending interrupt
}

void TIM2_IRQHandler(void)
{
	/**** When timer 2 rolls over, set a flag *****/
	//Should pending bit be reset?
	TIM2->SR = 0;
	timer2RolledOver = true;
}

void ADC1_2_IRQHandler(void) {
	adcComplete = true;
	adcResult = ADC1->DR;
}
void USART2_IRQHandler(void) {
	USART2->ICR |= 0x00000040; //Clear the overrun error flag
	USART2->TDR = *str++;
}

