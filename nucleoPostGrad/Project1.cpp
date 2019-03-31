#include <stm32l4xx.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#ifdef __cplusplus
//extern "C"
#endif

#define VREF 3.3
#define MASTER_PERIOD 62.5e-9
extern "C" void TIM1_CC_IRQHandler();
extern "C" void TIM2_IRQHandler();
extern "C" void TIM3_IRQHandler();
extern "C" void ADC1_2_IRQHandler();
extern "C" void USART2_IRQHandler();
void configureGpioPorts(void);

void delayMs(int);
void delay1Hz(void);


void initUsart(void);
void initPbInterrupt(void);   //not used
void initTim1(void);
void initTim2(void);
void initTim3(void);
void initUserSw(void);
void initAdc(void);
void toggleLed(void);
void initSysTick(void);

// Globals for ISR's
int volatile adcResult = 0;
volatile bool buttonInterrupt = false;
volatile bool adcComplete = false;
volatile bool timer3RolledOver = false;
volatile bool timer2RolledOver = false;
volatile uint16_t compareData[2];
volatile uint8_t activeEdge = 0;
volatile bool dataReady = false;
char *str;
char *buffer;

int main(void)
{
	
	bool strDone = true;
	bool returnVAdc = true;
	bool continuousAdc = false;
	bool continuous555 = false;
	bool doConversion = false;
	float VAdc = 0;
	uint16_t period;
	double frequency;
	float periodInNs;
	char rxChar = '?';
	char lastRxChar = '?';

	__disable_irq();
	/* Configure the system clock - using MSI as SYSCLK @16MHz */
	RCC->CR 		&= 	0xFFFFFF07;          //Clear ~MSIRANGE bits and MSIRGSEL bit
	RCC->CR 		|= 	0x00000089;          //Set MSI to 16MHz and MSIRGSEL bit

	//Enable PA5 clocks - for on-board LED (Nucleo)
	RCC->AHB2ENR 	|= 	0x00000001;  		   //Enable GPIO port A clk        
	GPIOA->MODER 	&= 	0xFFFFF3FF;          //Clear  GPIOA[5] MODER bits
	GPIOA->MODER 	|= 	0x00000400;          //Enable GPIOA[5] for output
	
	//initUserSw();  							//Enable SW1
	initUsart();
	configureGpioPorts();
	initTim1();
	initTim2();
	initTim3();
	initAdc();

	buffer = (char *) malloc(25);
	str = (char *) malloc(25);
	char dataPtr[150];
	bool firstCaptureDone = false;
	strcpy(str, "\n\nProgram Start\n\n");

	__enable_irq();
	while (1)
	{
			
		lastRxChar = rxChar;
		//Capture ADC data when it's ready - display it at 1mS intervals
		if(dataReady)
		{
			if (compareData[1] < compareData[0])
				period = (0xFFFF - compareData[0]) + compareData[1];
			else
				period = compareData[1] - compareData[0];   //in clkCycles
			frequency = 1.0 / (period* MASTER_PERIOD);   //make this a constant - or read from RCC register?
			periodInNs = period * MASTER_PERIOD;  //in clkCycles
			periodInNs *= 1e6;  //convert to uS
			if(firstCaptureDone == false)
				firstCaptureDone = true;
			else
				dataReady = false;
		}
					
		//Change the timer3 reload value according to the 
		//ADC value - just using some threshold values
		//Both ARR and CNT can be changed on the fly
		if(timer3RolledOver)
		{
			toggleLed();
			timer3RolledOver = false;
			NVIC_DisableIRQ(TIM3_IRQn);
			if (adcResult < 500) {
				TIM3->CNT = 0;
				TIM3->ARR = 50;
			}
			else if (adcResult < 1500) {
				TIM3->CNT = 0;
				TIM3->ARR = 150;
			}
			else if (adcResult < 2500) {
				TIM3->CNT = 0;
				TIM3->ARR = 550;
			}
			else if (adcResult < 3500) {
				TIM3->CNT = 0;
				TIM3->ARR = 1000;
			}
			else {
				TIM3->CNT = 0;
				TIM3->ARR = 2000;
			}
			NVIC_EnableIRQ(TIM3_IRQn);

		}

		/* iF there is data in the string buffer
		 * - send it out on the USART2*/
		if (*str != '\0')
		{
			NVIC_EnableIRQ(USART2_IRQn);
			continue;
		}
		else
		{
			NVIC_DisableIRQ(USART2_IRQn);
		}


		/*
		   Check for a new conversion completion on the ADC
		   Load buffer with string value - either voltage or ADC-code
		   Conversions are started in the main case-statement 
		     monitoring changes on the USART Rx line
		 */
		if (adcComplete == true)
		{
			*str = '\0';
			if (returnVAdc == true)
				sprintf(dataPtr, "ADC-result %fV\n", VAdc = (VREF / (4096 - 1))*adcResult);
			else
				sprintf(dataPtr, "ADC-result 0x%x\n", adcResult);
	
			strcpy(buffer, dataPtr);
			adcComplete = false;
			if (continuousAdc == false)
				rxChar = '?';

			str = buffer;
		}

		
		//Main key-pressed loop - operates at 1mS intervals
		else if(timer2RolledOver == true)
		{
			timer2RolledOver = false;

			//Clear out the buffer
			*buffer = '\0';
			//check for a new character
			if(USART2->ISR & 0x00000020)
				rxChar = USART2->RDR;
			else
				USART2->ICR |= 0x00000008;   //Clear the overrun error flag

			switch(rxChar)
			{

				/* ‘T’ or ‘t’: Report the 555 Timer Output Period in microseconds.
				*  ‘H’ or ‘h’: Report the 555 Timer Output High Time of its period in microseconds.
				*  ‘L’ or ‘l’: Report the 555 Timer Output Low Time of its period in microseconds.
				*  ‘F’ or ‘f’: Report the 555 Timer Output Frequency in kHz 
				*  ‘W’ or ‘w’: Display the 555 Timer Output Frequency in kHz Continuously at 1 second intervals.
				*  ‘Q’ or ‘q’: Stop displaying the 555 Timer Output Frequency Continuously
				*  ‘A’ or ‘a’: Report the ADC0 conversion result. This is the ADC value.
				*  ‘V’ or ‘v’: Report the ADC0 conversion result in Volts. You must convert the 
				*      ADC value to Volts. You may use floating point arithmetic for this calculation.
				*  ‘C’ or ‘c’: Display the ADC Voltage Continuously at 1 second intervals.
				*  ‘E’ or ‘e’: Stop displaying ADC Voltage Continuously.
				* Display an ‘Invalid Character’ Message for any other character 
				* entered (except CR(0x0d) or LF(0x0a). Continuous modes are mutually exclusive: 
				* display a message telling * the user to halt current continuous mode if 
				* another command is received in a continuous mode.
				*/

			case 'a': case 'A': case 'v': case 'V': 
			case 'c': case 'C': case 'e': case 'E':
				{ 
					
					//don't allow false exit of continuous 
					if (continuous555 == true)
					{
						rxChar = 'w';
						sprintf(str, "To exit continuous mode - enter 'q' or 'Q'\n");
						continue;
					}

					//start ADC conversion
					if (rxChar == 'c' || rxChar == 'C')
					{
						returnVAdc = true; //only voltage returned in continuous mode
						continuousAdc = true;
						ADC1->CR |= 0x00000004;  //Convst
						break;
					}
					//End continuous conversions
					if (continuousAdc == true && (rxChar == 'e' || rxChar == 'E'))
					{
						continuousAdc = false;
						rxChar = '?';
						continue;
					}
					//don't allow false exit of continuous mode
					if (continuousAdc == true && (rxChar != 'e' || rxChar != 'E'))
					{
						rxChar = 'c';
						sprintf(str, "To exit continuous mode - enter 'e' or 'E'\n");
						continue;
					}

					//Not in automatic conversion mode - do convst - 
					//setup for Volts out or codes out
					returnVAdc = (rxChar == 'v' || rxChar == 'V') ? true : false;
					continuousAdc = false; 
					ADC1->CR |= 0x00000004;  //Convst
					rxChar = '?'; 			//clear the received character so no repeat
					break; 
				}

			/* Measure 555 clock /period/frequncy */
			case 'q': case 'Q':
			case 't': case 'T':   case 'f': 
			case 'w': case 'W'	: case 'F': {
			
				if(rxChar == 'w' || rxChar == 'W')
					continuous555 = true;
				else if(rxChar == 'q' || rxChar == 'Q')
					continuous555 = false;

				/* Don't allow continous ADC read mode to be stopped here*/
				if (continuousAdc == true)
				{
					rxChar = 'c';
					sprintf(str, "To exit continuous mode - enter 'e' or 'E'\n");
					continue;
				}
				NVIC_EnableIRQ(TIM1_CC_IRQn);        //Enable TIM1 Capture Compare Interrupt                  

				//Skip the first capture because it is invalid
				if(firstCaptureDone == true)
				{
					sprintf(dataPtr, "");

					if (rxChar == 'f' || rxChar == 'F' ||
						rxChar == 'w' || rxChar == 'W')
					{
						
						if ((rxChar == 'f' || rxChar == 'F') && continuous555 == true)
						{
							rxChar = 'w';
							sprintf(str, "To exit continuous mode - enter 'q' or 'Q'\n");
							continue;
						}
						else
							sprintf(dataPtr, "555-frequency %.4fKhz\n", frequency / 1000);
						
					}
					else if ((rxChar == 't' || rxChar == 'T') && continuous555 == false)
					{
						sprintf(dataPtr, "555-Period %.3fuS\n", periodInNs);
						continuous555 = false;
					}
					else if (rxChar == 't' || rxChar == 'T')
					{
						rxChar = 'w';
						sprintf(str, "To exit continuous mode - enter 'q' or 'Q'\n");
						continue;
					}
					
					else if(rxChar != 'q' && rxChar != 'Q')
					{
						rxChar = 'w';
						sprintf(str, "To exit continuous mode - enter 'q' or 'Q'\n");
						continue;
					} 

					strcpy(buffer, dataPtr);
					str = buffer;
					if (continuous555 == false)
						NVIC_DisableIRQ(TIM1_CC_IRQn);
				}
				// Stop continuous output from the 555
				if(continuous555 == false && firstCaptureDone)
					rxChar = '?';
				break;
			}
			//Handle invalid characters - treat them all the same
			default: {
							
				if (rxChar != '?')
				{
					sprintf(dataPtr, "Invalid character - try again...\n");
					strcpy(buffer, dataPtr);
					str = buffer;
					rxChar = lastRxChar; //keep the last command if an invalid character was entered
				}
				else if (*str == '\0')
					str = buffer;
				}
			}
		} //if timer2RolledOver
	}//while(1)
} //main()


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
	SysTick->CTRL = 0;        					//Stop the timer
}

void initTim3() {
	/* Using timer 3 to write values to the LED in responce to the ADC value
	 * Interrupt for Timer3 is #29
	 * 1/16MHz = 62.5nS -  * 16000 = 1mS
	 * Futher divide this by 1000, in the reload value
	 * to give a 1Hz rollover
	 *
	 * Priority of interrupt may have to be changed to give
	 * this interrupt a higher priority to ensure delays actually work
	 **/

	RCC->APB1ENR1	|= 0x2;        		//Enable timer 3
	TIM3->PSC 		= 16000 - 1;       	//Prescalar value - divide 16MHz by 16000
	TIM3->ARR 		= (1000 / 1) - 1;   //Reload value
	TIM3->CR1		= 1;       			//enable timer
	TIM3->DIER      |= 1;       		//enable interrupt
	NVIC_EnableIRQ(TIM3_IRQn);       	//Enable the interrupt in the NVIC
}
void initTim2() {
	/* Interrupt for Timer 2 is #28
	 * 1/16MHz = 62.5nS -  * 16000 = 1mS
	 * Futher divide this by 1000, in the reload value
	 * to give a 1Hz rollover
	 *
	 * Priority of interrupt may have to be changed to give
	 * this interrupt a higher priority to ensure delays actually work
	 **/

	RCC->APB1ENR1	|= 0x1;        		//Enable timer 2
	TIM2->PSC 		= 16000 - 1;       	//Prescalar value - divide 16MHz by 16000
	TIM2->ARR 		= (1000 / 1) - 1;  	//Reload value
	TIM2->CR1		= 1;       			//enable timer
	TIM2->DIER      |= 1;       		//enable interrupt
	NVIC_EnableIRQ(TIM2_IRQn);       	//Enable the interrupt in the NVIC
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
	TIM2->CNT 		= 0;        	//Clear the count register
	TIM2->CR1 		= 1;			//Start the counter
	while(!(TIM2->SR & 1)) {}		//wait for rollover
	TIM2->SR &= ~1;        			//Clear UIF bit
	TIM2->CR1 		= 0;			//Stop the counter
}

void initUserSw() {
	/* Configure switch on the Nucleo board*/
	RCC->AHB2ENR 	|= 0x4;        		//ENABLE GPIOC port clk
	GPIOC->MODER 	&= 0xF3FFFFFF;       //Clear GPIOC[13] for input mode

	/* Configure the 2 switches on the Arduino shield
	   The Pushbutton Switches on the board are connected as
		S1: D12 – PA6
		S2: D13 – PA5
		Switches are to ground - so need to enable pull-up's on them
		2-bits per port-bit, 0x1 is pull-up
		S2 won't actually be used as it's conflicting with the 
		LED on the Nucleo board - also connected to PA5
	*/
	RCC->AHB2ENR 	|= 0x1;        		//ENABLE GPIOA port clk
	GPIOA->MODER 	&= 0xFFFFCFFF;       //Clear GPIOA[6] for input mode
	GPIOA->PUPDR	&= 0xFFFFCFFF;       //clear the PA6 pupdx bits
	GPIOA->PUPDR	|= 0x00001000;       //Enable the pull-up on PA6
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

	RCC->AHB2ENR 	|= RCC_AHB2RSTR_GPIOARST_Msk; 	//Enable the clock for GPIOA
	RCC->APB1ENR1	|= RCC_APB1ENR1_USART2EN_Msk;   //Enable the USART2 clock
	GPIOA->MODER 	&= 0xFFFFFF0F;        			//Clear PA2/3 bits
	GPIOA->MODER 	|= 0x000000A0;        			//Set PA2/3 for AF mode
	GPIOA->AFR[0]   &= 0xFFFF00FF;       			//Clear the AF bits for 2&3
	GPIOA->AFR[0]   |= 0x00007700;       			//Set both AF modes to AF7, for bits 2/3
	USART2->BRR		&= 0x0000;       				//clear BAUD Rate bits 
	USART2->BRR		|= 0x008B;       				//Set BAUD Rate to 115200 with UartClk at 16MHz
	USART2->CR1     &= 0xEFFF6FFE;       			//Clear the M1,OVER8,M0 bits, set 1 start-bit, 8-data bits n stop-bits, keep UE low
	USART2->CR2     &= 0xFFFFC000;       			//Clear the stop-bits to give 1 stop-bit (default anyway)
	USART2->CR1     |= 0x0000004D;       			//Enable Transmit-complete interrupt, RX/TX and the uart itself


}

void initPbInterrupt() {
	
	RCC->APB2ENR        |= 1;        	//Enable SYSCFG clk (for GPIO interrupt enables)
	SYSCFG->EXTICR[3] 	&= ~0x00F0;      //CLEAR_BIT the EXTI[13] bits
	SYSCFG->EXTICR[3]   |= 0x20;        	//Enable GPIOC for EXTI[13]
	EXTI->IMR1 			|= 0x2000;       //Unmask EXTI13
	EXTI->FTSR1 		|= 0x2000;       //Enable falling edge triggered interrupts (pushbutton high to low on push)
	NVIC_EnableIRQ(EXTI15_10_IRQn);      //Enable EXTI15-to-10 interrupts
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
	RCC->AHB2ENR	|= 0x00000001;          //Enable GPIOA CLK
	RCC->CCIPR		|= 0x30000000;          //Select SYSCLK as ADC clk source
	RCC->AHB2ENR	|= 0x00002000;          //Enable the ADC clock
	  
	ADC1->CR		&= 0xDFFFFFFF;          //Take ADC out of deep power down
	delayMs(1);  						    //Allow 1mS  - only needs Tadcvreg_stup - 20uS (datasheet p178)
	GPIOA->ASCR	    |= 0x00000010;          //Connect analog switch to GPIOA[4]
	GPIOA->MODER	|= 0x00000300;          //Set A4 for analog input mode  - actually reset to analog input mode
	ADC1->CR		|= 0x10000000;          //Enable ADC1 votage regulator

	ADC1->IER		|= 0x00000004;          //Enable ADC1 EOC interrupt
	NVIC_EnableIRQ(ADC1_2_IRQn); 			   //Enable interrupts on ADC1

	ADC1->SMPR1    |= 0x08008000;          //Add a little more sampling time to channel 9 (6.5 ADC clk cycles)
	ADC1->ISR		|= 0x00000001;          //Clear the ADRDY bit in the ADCx_ISR register by writing ‘1’.
	ADC1->SQR1		|= 0x00000240;          //Set for a sequence of 1 conversion on CH9 _01001_00_0000
	ADC1->CR		|= 0x00000001;          //Enable ADC1
	 
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
	A9 - D8 (Arduino connector), 555 input
	C7 - D9 (Arduino connector), 555 input
	*/

	RCC->AHB2ENR 	|= 	0x00000007;         //Enable PA,B,C clocks
	
	/* Enable PA8, PA10 for output */
	GPIOA->MODER 	&= 	0xFFCCFFFF;         //Clear GPIOA[10,8] MODER bits
	GPIOA->MODER 	|= 	0x00110000;         //Enable GPIOA[10,8] MODER  bits for output
	/* Configure PA9 for 555 input capture*/
	GPIOA->MODER    &=  0xFFF3FFFF;			//clear A9 
	GPIOA->MODER    |=  0x00080000;			//set A9 for alternate mode 
 
	GPIOA->AFR[1]   &=  0xFFFFFF0F;			//clear AF bits for gpio[9]
	GPIOA->AFR[1]   |=  0x00000010;			//set AF bits for gpio[9] - tim1_ch2 (AF1 - page 92 datasheet)
	


	/* Enable PA3,PB4,PB5,PB10 for output */
	GPIOB->MODER 	&= 	0xFFCFF03F;         //Clear GPIOB[10,5,4,3] MODER bits
	GPIOB->MODER 	|= 	0x00100540;         //Enable GPIOB[10,5,4,3] MODER bits for output
	/* Enable PC0, PC1 for output */
	GPIOC->MODER 	&= 	0xFFFFFFF0;         //Clear GPIOC[1,0] MODER bits
	GPIOC->MODER 	|= 	0x00000005;         //Enable GPIOC[1,0] MODER  bits for output

}


void EXTI15_10_IRQHandler(void) {
	/**** Handle PC13 (bushbutton) interrupt 
	 **** Not used in this lab - just a test ****/
	buttonInterrupt = true;
	EXTI->PR1 = 0x2000;    //clear pending interrupt
}

void TIM3_IRQHandler(void)
{
	/**** When timer 3 rolls over, set a flag *****/
	TIM3->SR = 0;
	timer3RolledOver = true;
}
void TIM2_IRQHandler(void)
{
	/**** When timer 2 rolls over, set a flag *****/
	TIM2->SR = 0;
	timer2RolledOver = true;
}

void ADC1_2_IRQHandler(void) {
	adcComplete = true;
	adcResult = ADC1->DR;
}
void USART2_IRQHandler(void) {
	USART2->ICR |= 0x00000040;  //Clear the overrun error flag
	USART2->TDR = *str++;
}
void toggleLed(void)
{
	
	if (GPIOA->IDR & 0x00000400)
		GPIOA->BRR |= 0x00000400;
	else
		GPIOA->ODR |= 0x00000400;
}
void initTim1(void){
/*
 * Calculate the Period of the Input signal from the 555 Timer 
 * at input PA9, using the Input Capture Technique. 
 * Also Calculate the width of the High and Low pulses 
 * generated by the Timer (they are not necessarily equal). 
 * Enable the appropriate interrupt and use an interrupt 
 * handler to read the timer values.

 * ‘T’ or ‘t’: Report the 555 Timer Output Period in microseconds.
 * ‘H’ or ‘h’: Report the 555 Timer Output High Time of its period in microseconds.
 * ‘L’ or ‘l’: Report the 555 Timer Output Low Time of its period in microseconds.
 * ‘F’ or ‘f’: Report the 555 Timer Output Frequency in kHz 
 * ‘W’ or ‘w’: Display the 555 Timer Output Frequency in kHz Continuously at 1 second intervals.
 * ‘Q’ or ‘q’: Stop displaying the 555 Timer Output Frequency Continuously

 * (Recall that Frequency = 1/Period). You may use floating 
 * point arithmetic for these calculations.

 * set AF bits for gpio[9] - tim1_ch2 (AF1 - page 92 datasheet) PA9 
 * So we need to configure timer 1 channel 2 for capture

 * 1/16MHz = 62.5nS 
 * Do not prescale this so timer is clocked at 16MHz
 * No need 
 * to give a 1Hz rollover

 * Waveform from 555 is < 15kHz - 1/15k = 66uS
 * How many 62.5ns periods to count this high?
 * 66us/62.5ns = 1056 periods - so no real need to divide clock
 * at 1KHz - only 16000 clocks are required
 * 
 * CCER [7:4]		- channel 2 edge detection setup - 0001, rising edge, and enabled
 * CC1x - [1:0]		- 01: cc2 channel is configured as input, i2c is mapped on ti2
 * CCMR1 [15:12]    - Input capture 2 filter - leave it at zero - using edge detection
 *		 [11:10]    - Input capture 2 prescaler - 00 - capture every (rising) edge
 *		 [9:8] = 01 - cc2 channel is configured as input, ic2 is mapped on ti2
 *	     
 * 
 **/


	RCC->APB2ENR	|= 0x00000800;		//Enable Timer 1 clock
	TIM1->PSC 		=  0x00000000;     	//Prescalar value - no-divide
	TIM1->ARR 		=  0x0000FFFF;      //Reload with full-scale
	TIM1->CCMR1     |= 0x00000100;      //capture on every rising edge
	TIM1->CCER      &= 0xFFFFFF0F;      //Clear cc2 polarity bits
	TIM1->CCER      |= 0x00000010;      //Enable rising edge capture on cc2
	TIM1->DIER      |= 4;       		//enable capture-compare channel 2 interrupt
	TIM1->CR1		|= 1;      			//enable timer
	
}

void initSysTick(void)
{
	/* Enable SysTick Exception Request on a count to zero */
	SysTick->CTRL |= 0x00000002;
	/* Select AHB/8 as the clock source */
	SysTick->CTRL &= 0xFFFFFFF7;
	/* If AHB=40MHz - set reload to reset at 1mS
		Reload value = number of processor cycles -1
		40MHz/8 = 5MHz = period=200nS = 1mS 
		=> load = 1ms/200ns= 5000-1clk cycles = 4999 = 0x1387
	*/
	SysTick->LOAD = 0x00001387;
}

void TIM1_CC_IRQHandler(void)
{
	/**** Capture compare event on timer 1    *****/
	/* Capture 2 edges*/
	compareData[activeEdge] = TIM1->CCR2;
	if (activeEdge == 1)
	{
		NVIC_DisableIRQ(TIM1_CC_IRQn);
		activeEdge = 0;
		dataReady = true;
	}
	else 
		activeEdge++;
}

