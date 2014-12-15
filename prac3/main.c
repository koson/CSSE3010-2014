/**
  ******************************************************************************
  * @file    prac3/main.c 
  * @author  MY FIRST NAME + SURNAME
  * @date    10-January-2014
  * @brief   Prac 3 Template C main file - Laser Transmitter/Receiver.
  *			 NOTE: THIS CODE IS INCOMPLETE AND DOES NOT COMPILE. 
  *				   GUIDELINES ARE GIVEN AS COMMENTS.
  *			 REFERENCES: ex1_led, ex2_gpio, ex3_gpio_int, ex4_adc, ex5_pwm, 
  *						 ex6_pwmin
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "netduinoplus2.h"
#include "stm32f4xx_conf.h"
#include "debug_printf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
unsigned int adc_value = 2050;
unsigned int frequency = 0;
unsigned int toggle = 0;
unsigned int funct_toggle = 1;
uint16_t PrescalerValue = 0;
unsigned int period_step = 0;
unsigned int input_capture_value = 0;
unsigned int input_frequency = 0;
/* Private function prototypes -----------------------------------------------*/
void vHardwareMain_init(void);
void vHardware1_init(void);
void vHardware2_init(void);
void function_1(void);
void function_2(void);
void vDelay(__IO unsigned long nCount);
static int rescaleAdc(int AdcValue, const int newLow, const int newHigh);



/**
  * @brief  Main program - up counter and press counter.
  * @param  None
  * @retval None
  */
void main(void) {

	NP2_boardinit();	//Initalise NP2
	vHardwareMain_init();	//Initalise hardware modules
	vHardware1_init();
	ADC_SoftwareStartConv(ADC1); //Start ADC conversion
  	
	/* Main processing loop */
  	while (1) {
		
		if (funct_toggle == 1) {
			function_1();
		}
		else if (funct_toggle == 2) {
			function_2();
		}
		
    	vDelay(0x7FFF00);	//Delay 1s
	}
}



/**
  * @brief  Initialise HardwareMain, Hardware1 and Hardware2.
  * @param  None
  * @retval None
  */
void vHardwareMain_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
  	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	
	NP2_LEDInit();		//Initialise Blue LED
	NP2_LEDOff();		//Turn off Blue LED
	
	/* Enable the Clock */
	RCC_AHB1PeriphClockCmd(NP2_A2_GPIO_CLK, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //APB2 ADC CLK
  	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
  	
  	/* Configure A2 pin as pull down input */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
  	GPIO_InitStructure.GPIO_Pin = NP2_A2_PIN;
  	GPIO_Init(NP2_A2_GPIO_PORT, &GPIO_InitStructure);
  	
  	/* Connect external interrupt EXTI Line0 to DO pin */
  	SYSCFG_EXTILineConfig(NP2_A2_EXTI_PORT, NP2_A2_EXTI_SOURCE);

  	/* Configure EXTI Line0 for rising edge detection */
  	EXTI_InitStructure.EXTI_Line = NP2_A2_EXTI_LINE;
  	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  	EXTI_Init(&EXTI_InitStructure);

  	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
  	NVIC_InitStructure.NVIC_IRQChannel = NP2_A2_EXTI_IRQ;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
  	
/*-----------------------------------ADC-----------------------------------------------*/
	/* Configure A0 for ADC input */
	GPIO_InitStructure.GPIO_Pin = NP2_A0_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  	GPIO_Init(NP2_A0_GPIO_PORT, &GPIO_InitStructure);
  	
  	/* ADC Common Init */
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	/* ADC Specific Init for 12Bit resolution and continuous sampling */
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = 0;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfConversion = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* Configure ADC1 to connect to the NP2 A0 channel */
	ADC_RegularChannelConfig(ADC1, NP2_A0_ADC_CHAN, 1, ADC_SampleTime_3Cycles);

  	/* Enable ADC1 */
  	ADC_Cmd(ADC1, ENABLE);
}

void vHardware1_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure; //GPIO
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure; //TIM Input
  	NVIC_InitTypeDef   NVIC_InitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure; //TIM Output

	NP2_LEDInit();		//Initialise Blue LED
	NP2_LEDOff();		//Turn off Blue LED

/*-----------------------------RCC----------------------------------------------------*/
	/* Enable the GPIO D0/D1/A0 Clock */
	RCC_AHB1PeriphClockCmd(NP2_D0_GPIO_CLK, ENABLE); //GPIO D0 CLK
	RCC_AHB1PeriphClockCmd(NP2_D1_GPIO_CLK, ENABLE); //GPIO D0 CLK
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //APB1 TIM5
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //APB1 TIM3
  	
/*-----------------------------GPIO---------------------------------------------------*/
  	/* Configure the GPIO_D1 pin for output - Task 1*/
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Pin = NP2_D1_PIN;
  	GPIO_Init(NP2_D1_GPIO_PORT, &GPIO_InitStructure);
  	
  	/* Configure the GPIO_D2 to GPIO_D11 pins */
	GPIO_InitStructure.GPIO_Pin = NP2_D2_PIN;
  	GPIO_Init(NP2_D2_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = NP2_D3_PIN;
  	GPIO_Init(NP2_D3_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = NP2_D4_PIN;
  	GPIO_Init(NP2_D4_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = NP2_D5_PIN;
  	GPIO_Init(NP2_D5_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = NP2_D6_PIN;
  	GPIO_Init(NP2_D6_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = NP2_D7_PIN;
  	GPIO_Init(NP2_D7_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = NP2_D8_PIN;
  	GPIO_Init(NP2_D8_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = NP2_D9_PIN;
  	GPIO_Init(NP2_D9_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = NP2_D10_PIN;
  	GPIO_Init(NP2_D10_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = NP2_D11_PIN;
  	GPIO_Init(NP2_D11_GPIO_PORT, &GPIO_InitStructure);
  	
	/* Configure the GPIO_D0 pin for photodiode input capture - Task 2*/
	GPIO_InitStructure.GPIO_Pin = NP2_D0_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_Init(NP2_D0_GPIO_PORT, &GPIO_InitStructure);
  	
  	/* Connect TIM3 output to D0 pin for input capture */  
  	GPIO_PinAFConfig(NP2_D0_GPIO_PORT, NP2_D0_PINSOURCE, GPIO_AF_TIM3);
  	
/*--------------------------------TIM5-------------------------------------------------*/
	/* Configure TIM 5 for compare update interrupt. */
	/* Compute the prescaler value */
  	PrescalerValue = (uint16_t) ((SystemCoreClock /2)/50000) - 1; //Set clock prescaler to 50kHz

  	/* Time base configuration */
  	TIM_TimeBaseStructure.TIM_Period = 50000;			//Set period to be 1s
  	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	/* Set Reload Value for 1s */
	TIM_SetAutoreload(TIM5, 50000);
  	TIM_ARRPreloadConfig(TIM5, ENABLE);

	/* Enable NVIC for Timer 5 */
	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

	/* Enable timer 5 update interrupt - Page: 579, STM32F4xx Programmer's Reference Manual */
	TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);

  	/* TIM5 enable counter */
  	TIM_Cmd(TIM5, ENABLE);
  	
/*-----------------------------TIM3----------------------------------------------------*/
	/* Configure TIM 3 for input capture interrupt. */
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
  	/* Configure TIM3 Input capture for channel 2 */
  	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
  	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
  	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
  	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
  	TIM_ICInitStructure.TIM_ICFilter = 0x0;
  	
  	TIM_ICInit(TIM3, &TIM_ICInitStructure);
  	
  	TIM_SetAutoreload(TIM3, 50000);
  	
	/* Select the TIM3 Filter Input Trigger: TI2FP2 */
  	TIM_SelectInputTrigger(TIM3, TIM_TS_TI2FP2);

	/* Enable the TIM3 global Interrupt */
  	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
  	
	/* Select the slave Mode: Reset Mode */
  	TIM_SelectSlaveMode(TIM3, TIM_SlaveMode_Reset);
  	TIM_SelectMasterSlaveMode(TIM3,TIM_MasterSlaveMode_Enable);

  	/* Enable the CC2 Interrupt Request */
  	TIM_ITConfig(TIM3, TIM_IT_CC2, ENABLE);

  	/* TIM3 enable counter */
 	TIM_Cmd(TIM3, ENABLE); 
}

void vHardware2_init(void) {

	USART_InitTypeDef USART_debug;
	GPIO_InitTypeDef  GPIO_InitStructure; //GPIO
	
	NP2_LEDInit();		//Initialise Blue LED
	NP2_LEDOff();		//Turn off Blue LED
	
	/* Enable D0 and D1 GPIO clocks */
	RCC_AHB1PeriphClockCmd(NP2_D0_GPIO_CLK | NP2_D1_GPIO_CLK, ENABLE);
	
	/* Enable the USART Clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); //APB2 USART
	
/*-----------------------------USART GPIO----------------------------------------------*/
	/* Configure the GPIO_D0 & GPIO_D1 pin for UART TX & RX - Task 3*/
	/* Configure the GPIO USART 2 pins */ 
  	GPIO_InitStructure.GPIO_Pin = NP2_D1_PIN | NP2_D0_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_Init(NP2_D0_GPIO_PORT, &GPIO_InitStructure);

/*-------------------------USART GPIO CONNECTION---------------------------------------*/
	/* Connect USART 6 TX and RX pins to D1 and D0 */
	GPIO_PinAFConfig(NP2_D1_GPIO_PORT, NP2_D1_PINSOURCE, GPIO_AF_USART6);
	GPIO_PinAFConfig(NP2_D0_GPIO_PORT, NP2_D0_PINSOURCE, GPIO_AF_USART6);
	
/*-----------------------------------USART---------------------------------------------*/
	/* Configure USART 6 for 9600bps, 8bits, 1 stop bit, no parity, no flow control */
	USART_debug.USART_BaudRate = 9600;				
	USART_debug.USART_WordLength = USART_WordLength_8b;
	USART_debug.USART_StopBits = USART_StopBits_1;
	USART_debug.USART_Parity = USART_Parity_No;
	USART_debug.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_debug.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  	
	USART_Init(USART6, &USART_debug);
  	
  	/* Enable USART 6 */
	USART_Cmd(USART6, ENABLE);
}
	
/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void vDelay(__IO unsigned long nCount) {
  
	/* Delay a specific amount before returning */
	while(nCount--)	{
  	}
}

/**
  * @brief  TIM3 Interrupt handler
  * @param  None.
  * @retval None
  */
void TIM3_IRQHandler(void) {

	uint16_t first = 0;
	uint16_t second = 0;
	uint16_t last = 0;
	uint16_t first_data[2];
	uint16_t second_data[4];
	uint16_t last_data[4];

	/* Read input capture value and calculate duty cycle. */
	/* Toggle LED if Timer 3 input capture 2 interrupt has occured */
	if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET) {
    	TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);

  		/* Read and display the Input Capture value */
  		input_capture_value = TIM_GetCapture2(TIM3);
  		input_frequency = 50000 / input_capture_value;
  		//debug_printf("period %d, adc %d, capture value %d, capture freq %d\n\r", period_step, adc_value, input_capture_value, input_frequency);
	}
	
	/* extracting in decimal*/
	first = input_frequency / 100;
	second = (input_frequency % 100) / 10;
	last = ((input_frequency % 100) % 10);
	
	debug_printf("freq = %d, %d %d %d\n\r", input_frequency, first, second, last);
	
	first_data[0] = ((first >> 1) & 0x1);
	first_data[1] = (first & 0x1);
	
	second_data[0] = ((second >> 3) & 0x1);
	second_data[1] = ((second >> 2) & 0x1);
	second_data[2] = ((second >> 1) & 0x1);
	second_data[3] = (second & 0x1);
		
	last_data[0] = ((last >> 3) & 0x1);
	last_data[1] = ((last >> 2) & 0x1);
	last_data[2] = ((last >> 1) & 0x1);
	last_data[3] = (last & 0x1);
	
	debug_printf("%d %d %d %d %d %d %d %d %d %d \n\r", first_data[0],first_data[1],second_data[0],second_data[1],second_data[2],second_data[3],last_data[0],last_data[1],last_data[2],last_data[3]);
	
	GPIO_WriteBit(NP2_D2_GPIO_PORT, NP2_D2_PIN, first_data[0] & 0x01);
	GPIO_WriteBit(NP2_D3_GPIO_PORT, NP2_D3_PIN, first_data[1] & 0x01);
	GPIO_WriteBit(NP2_D4_GPIO_PORT, NP2_D4_PIN, second_data[0] & 0x01);
	GPIO_WriteBit(NP2_D5_GPIO_PORT, NP2_D5_PIN, second_data[1] & 0x01);
	GPIO_WriteBit(NP2_D6_GPIO_PORT, NP2_D6_PIN, second_data[2] & 0x01);
	GPIO_WriteBit(NP2_D7_GPIO_PORT, NP2_D7_PIN, second_data[3] & 0x01);
	GPIO_WriteBit(NP2_D8_GPIO_PORT, NP2_D8_PIN, last_data[0] & 0x01);
	GPIO_WriteBit(NP2_D9_GPIO_PORT, NP2_D9_PIN, last_data[1] & 0x01);
	GPIO_WriteBit(NP2_D10_GPIO_PORT, NP2_D10_PIN, last_data[2] & 0x01);
	GPIO_WriteBit(NP2_D11_GPIO_PORT, NP2_D11_PIN, last_data[3] & 0x01);
}

/**
  * @brief  Timer 5 Interrupt handler
  * @param  None.
  * @retval None
  */
void TIM5_IRQHandler (void) {
	/* Generate square wave using the update compare interrupt. */
	/* Toggle LED if Timer 5 update compare interrupt has occured */
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {
    	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		toggle = ~toggle;
		GPIO_WriteBit(NP2_D1_GPIO_PORT, NP2_D1_PIN, toggle & 0x01);
	}
}

// Rescales the ADC value from [0, 4095] to [newLow, newHigh]
static int rescaleAdc(int AdcValue, const int newLow, const int newHigh)
{
    unsigned int newvalue = 0;
    
    newvalue = (int)((double) AdcValue / 4095.0 * (newHigh - newLow)) + newLow;
    
    return newvalue;
}

void function_1(void) {

	/* Read A0 ADC input */
	/* Wait for ADC conversion to finish by polling the ADC Over Flag. */
	while ((ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR) != RESET) && (ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR) != RESET)); 		

	/* Extract ADC conversion values */
	adc_value = ADC_GetConversionValue(ADC1);
	
	/* Convert to frequency */
	frequency = rescaleAdc(adc_value, 1, 100);
		
	/* Print ADC conversion values */
	ADC_ClearFlag(ADC1, ADC_FLAG_OVR);

	period_step = (42000000/(PrescalerValue * frequency));
		
	/* Set Square Wave output frequency, See TIM_SetAutoreload */
	TIM_SetAutoreload(TIM5, period_step);
}

void function_2(void) {

	char tx_char;
	char rx_char;
	
	if (VCP_getchar(&tx_char)) {
		debug_printf("TX %c\n\r", tx_char);
		USART_SendData(USART6, tx_char);								//Send character via USART 6
		while (USART_GetFlagStatus(USART6, USART_FLAG_TC) == RESET);	//Wait for Transmit Clear flag to be SET
		USART_ClearFlag(USART6, USART_FLAG_TC);							//Clear Transmit Clear flag
	}
	/* Check if receive buffer has received a character */
	if (USART_GetFlagStatus(USART6, USART_FLAG_RXNE) == SET) {
		rx_char = USART_ReceiveData(USART6);						//Read character received by USART 6
		USART_ClearFlag(USART6, USART_FLAG_RXNE);					//Clear receive buffer register flag
		debug_printf("RX %c\n\r", rx_char);
	}
}

void NP2_A2_EXTI_IRQ_HANDLER(void) {

	/* Check if interrupt has occured - see ex3_gpio_interrupt */
	if (EXTI_GetITStatus(NP2_A2_EXTI_LINE) != RESET) {
		if (funct_toggle == 1) {
			funct_toggle = 2;
			vHardware2_init();
			vDelay(0x7FFF00);
		}
		else if (funct_toggle == 2) {
			funct_toggle = 1;
			vHardware1_init();
			vDelay(0x7FFF00);
		}
		debug_printf("Mode %d activated\n\r", funct_toggle);
		
    	/* Clear the EXTI line 0 pending bit */
    	EXTI_ClearITPendingBit(NP2_A2_EXTI_LINE);
  	}
}

