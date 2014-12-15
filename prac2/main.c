/**
  ******************************************************************************
  * @file    prac2/main.c 
  * @author  MY FIRST NAME + SURNAME
  * @date    10-January-2014
  * @brief   Prac 1 Template C main file - pan servo control.
  *			 NOTE: THIS CODE IS INCOMPLETE AND DOES NOT COMPILE. 
  *				   GUIDELINES ARE GIVEN AS COMMENTS.
  *			 REFERENCES: ex1_led, ex2_gpio, ex3_gpio, ex4_adc, ex6_pwm, ex11_console
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
char direction;
unsigned int original_pulse = 765;
/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);
void Delay(__IO unsigned long nCount);
void Timer(void);

/* @brief  Main program
 * @param  None
 * @retval None
 */
void main(void) {

	NP2_boardinit();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules
	ADC_SoftwareStartConv(ADC1);	//Perform ADC conversions
	char RxChar;
	unsigned int current_pulse = original_pulse;
	unsigned int nega_maxpulse = original_pulse + 472;
	unsigned int posi_maxpulse = original_pulse - 472;
	unsigned int adc_value = 2050;

	Delay(0xFFFFFF);
	/* Main processing loop */
  	while (1) {
		
		/****************** TASK 2. ***************/
		if (VCP_getchar(&RxChar)) {
			if (RxChar == '+') {
				direction = RxChar;
				debug_printf ("receive +\n\r");
			}
			else if (RxChar == '-') {
				direction = RxChar;
				debug_printf ("receive -\n\r");
			}
			else {
				if (direction == '-') {
					if (RxChar == '0') {
						current_pulse = original_pulse;
						debug_printf ("0 Degrees\n\r");
					}
					else if (RxChar == '1') {
						current_pulse = original_pulse + 56;
						debug_printf ("-10 Degrees\n\r");
					}
					else if (RxChar == '2') {
						current_pulse = original_pulse + 111;
						debug_printf ("-20 Degrees\n\r");
					}
					else if (RxChar == '3') {
						current_pulse = original_pulse + 167;
						debug_printf ("-30 Degrees\n\r");
					}
					else if (RxChar == '4') {
						current_pulse = original_pulse + 222;
						debug_printf ("-40 Degrees\n\r");
					}
					else if (RxChar == '5') {
						current_pulse = original_pulse + 278;
						debug_printf ("-50 Degrees\n\r");
					}
					else if (RxChar == '6') {
						current_pulse = original_pulse + 333;
						debug_printf ("-60 Degrees\n\r");
					}
					else if (RxChar == '7') {
						current_pulse = original_pulse + 389;
						debug_printf ("-70 Degrees\n\r");
					}
					else if (RxChar == '8') {
						current_pulse = original_pulse + 444;
						debug_printf ("-80 Degrees\n\r");
					}
					else if (RxChar == '9') {
						current_pulse = nega_maxpulse;
						debug_printf ("Max -85 Degrees\n\r");
					}
				}
				if (direction == '+') {
					if (RxChar == '0') {
						current_pulse = original_pulse;
						debug_printf ("0 Degrees\n\r");
					}
					else if (RxChar == '1') {
						current_pulse = original_pulse - 56;
						debug_printf ("+10 Degrees\n\r");
					}
					else if (RxChar == '2') {
						current_pulse = original_pulse - 111;
						debug_printf ("+20 Degrees\n\r");
					}
					else if (RxChar == '3') {
						current_pulse = original_pulse - 167;
						debug_printf ("+30 Degrees\n\r");
					}
					else if (RxChar == '4') {
						current_pulse = original_pulse - 222;
						debug_printf ("+40 Degrees\n\r");
					}
					else if (RxChar == '5') {
						current_pulse = original_pulse - 278;
						debug_printf ("+50 Degrees\n\r");
					}
					else if (RxChar == '6') {
						current_pulse = original_pulse - 333;
						debug_printf ("+60 Degrees\n\r");
					}
					else if (RxChar == '7') {
						current_pulse = original_pulse - 389;
						debug_printf ("+70 Degrees\n\r");
					}
					else if (RxChar == '8') {
						current_pulse = original_pulse - 444;
						debug_printf ("+80 Degrees\n\r");
					}
					else if (RxChar == '9') {
						current_pulse = posi_maxpulse;
						debug_printf ("Max +85 Degrees\n\r");
					}
				}
			}
		}

		/****************** TASK 3. ***************/
		/* Read A0 ADC channel 

			if Joystick is pushed forward do:
				move pan servo in positive direction

			if Joystick is pushed backwards do:
				move pan servo in negative direction

		*/
		
		/* Wait for ADC conversion to finish by polling the ADC Over Flag. */
		while ((ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR) != RESET) && (ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR) != RESET)); 		

		/* Extract ADC conversion values */
		adc_value = ADC_GetConversionValue(ADC1);
		
		if (adc_value < 1900) {
			current_pulse = current_pulse + 56;
			debug_printf("subtracting 10 degrees.\n\r");
		}
		else if (adc_value > 2200) {
			current_pulse = current_pulse - 56;
			debug_printf("adding 10 degrees.\n\r");
		}
		
		ADC_ClearFlag(ADC1, ADC_FLAG_OVR);
		/* Toggle 'Keep Alive Indicator' LED */
		
		if (current_pulse >= nega_maxpulse) {
			current_pulse = nega_maxpulse;
		}
		if (current_pulse <= posi_maxpulse) {
			current_pulse = posi_maxpulse;
		}
		
		TIM_SetCompare4 (TIM2, current_pulse);
		
		//debug_printf("%d \n\r", TIM_GetCounter(TIM2));
    	Delay(0x7FFF00);	//Delay 1s

	}
}

/**
  * @brief  Initialise Hardware 
  * @param  None
  * @retval None
  */
void Hardware_init(void) {

	/****************** TASK 1. ***************/
	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	ADC_InitTypeDef ADC_InitStructure;
  	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	uint16_t PrescalerValue = 0;

	NP2_LEDInit();		//Initialise Blue LED
	NP2_LEDOff();		//Turn off Blue LED

	/* Enable the GPIO D2 Clock */
  	RCC_AHB1PeriphClockCmd(NP2_D2_GPIO_CLK, ENABLE);
  	
  	/* TIM2 clock enable */
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  	
  	/* Configure the D2 pin for PWM output */
  	GPIO_InitStructure.GPIO_Pin = NP2_D2_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_Init(NP2_D2_GPIO_PORT, &GPIO_InitStructure);
  	
  	/* Connect TIM2 output to D2 pin */  
  	GPIO_PinAFConfig(NP2_D2_GPIO_PORT, NP2_D2_PINSOURCE, GPIO_AF_TIM2);
  	
  	/* Compute the prescaler value. SystemCoreClock = 168000000 - set for 500Khz clock */
  	PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 500000) - 1;
  	
  	/* Time 2 mode and prescaler configuration */
  	TIM_TimeBaseStructure.TIM_Period = 500000/50; 	//Set for 20ms (50Hz) period
  	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  	
  	/* Configure Timer 2 mode and prescaler */
  	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* Enable TIM 2 channel 4 for PWM operation */
	/* PWM Mode configuration for Channel4 - set pulse width*/
  	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;		//Set PWM MODE (1 or 2 - NOT CHANNEL)
  	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  	TIM_OCInitStructure.TIM_Pulse = original_pulse;		//1ms pulse width
  	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  	
  	/* Enable Output compare channel 4 */
  	TIM_OC4Init(TIM2, &TIM_OCInitStructure);

  	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

  	/* TIM2 enable counter */
 	TIM_Cmd(TIM2, ENABLE); 

	/* Configure A0 for ADC input */
	/* Configure A0 as analog input */
  	GPIO_InitStructure.GPIO_Pin = NP2_A0_PIN;
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
  	GPIO_Init(NP2_A0_GPIO_PORT, &GPIO_InitStructure);

	/* Enable clock for ADC 1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

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

/**
  * @brief  Delay Function.
  * @param  nCount:specifies the Delay time length.
  * @retval None
  */
void Delay(__IO unsigned long nCount) {
  
	/* Delay a specific amount before returning */
	while(nCount--)	{
  	}
}




