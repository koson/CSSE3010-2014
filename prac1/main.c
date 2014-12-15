/**
  ******************************************************************************
  * @file    prac1/main.c 
  * @author  MY_FIRST_NAME + SURNAME
  * @date    10-January-2014
  * @brief   Prac 1 Template C main file - BCD timer and press counter.
  *			 NOTE: THIS CODE IS PSEUDOCODE AND DOES NOT COMPILE. 
  *				   GUIDELINES ARE GIVEN AS COMMENTS.
  *			 REFERENCES: ex1_led, ex2_gpio, ex3_gpio, ex11_character
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "netduinoplus2.h"
#include "stm32f4xx_conf.h"
#include "debug_printf.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define DEBOUNCE_TIME 100
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint16_t counter = 0;
uint16_t t_counter = 0;
uint16_t c_counter = 0;
uint16_t first_data[2];
uint16_t second_data[4];
uint16_t last_data[4];
uint16_t press_count = 0;
uint16_t first = 0;
uint16_t second = 0;
uint16_t last = 0;
uint16_t i;
char RxChar = 'c';


/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);
void Delay(__IO unsigned long nCount);

/**
  * @brief  Main program - timer and press counter.
  * @param  None
  * @retval None
  */
void main(void) {

	NP2_boardinit();	//Initalise NP2
	Hardware_init();	//Initalise hardware modules
  	
	/* Main processing loop */
  	while (1) {
  	
  		/* Check for keystroke */
  		if (VCP_getchar(&RxChar)) {
  			debug_printf("Mode Change to %c!\n\r", RxChar);
  		}
  		
  		/* check for modes */ 
		if (RxChar == 'c') {
			counter = c_counter;
		}
		else if (RxChar == 't')	{
			counter = t_counter;
			t_counter++;
			debug_printf("mode = %c value = %d %d %d \n\r", RxChar, first, second, last);
		}
		else {
			continue;
		}

		/****************** Display counter. ***************/
		// Check for max which is 399
		if (counter >= 399) {
			GPIO_WriteBit(NP2_D0_GPIO_PORT, NP2_D0_PIN, 0x00);
			GPIO_WriteBit(NP2_D1_GPIO_PORT, NP2_D1_PIN, 0x00);
			GPIO_WriteBit(NP2_D2_GPIO_PORT, NP2_D2_PIN, 0x00);
			GPIO_WriteBit(NP2_D3_GPIO_PORT, NP2_D3_PIN, 0x00);
			GPIO_WriteBit(NP2_D4_GPIO_PORT, NP2_D4_PIN, 0x00);
			GPIO_WriteBit(NP2_D5_GPIO_PORT, NP2_D5_PIN, 0x00);
			GPIO_WriteBit(NP2_D6_GPIO_PORT, NP2_D6_PIN, 0x00);
			GPIO_WriteBit(NP2_D7_GPIO_PORT, NP2_D7_PIN, 0x00);
			GPIO_WriteBit(NP2_D8_GPIO_PORT, NP2_D8_PIN, 0x00);
			GPIO_WriteBit(NP2_D9_GPIO_PORT, NP2_D9_PIN, 0x00);
			counter = 0;
			debug_printf ("BOOYA!!!!!!\n\r");
		}
		
		/* extracting in decimal 3*/
		first = counter / 100;
		second = (counter % 100) / 10;
		last = ((counter % 100) % 10);
		
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

		/* lighting up led base on BCD */
		/* first d0 and d1 */
		if (first_data[0] == 1) {
			GPIO_WriteBit(NP2_D0_GPIO_PORT, NP2_D0_PIN, 0x01);
		}
		else {
			GPIO_WriteBit(NP2_D0_GPIO_PORT, NP2_D0_PIN, 0x00);
		}
		if (first_data[1] == 1) {
			GPIO_WriteBit(NP2_D1_GPIO_PORT, NP2_D1_PIN, 0x01);
		}
		else {
			GPIO_WriteBit(NP2_D1_GPIO_PORT, NP2_D1_PIN, 0x00);
		}
		
		/* second d2 to d5 */
		if (second_data[0] == 1) {
			GPIO_WriteBit(NP2_D2_GPIO_PORT, NP2_D2_PIN, 0x01);
		}
		else {
			GPIO_WriteBit(NP2_D2_GPIO_PORT, NP2_D2_PIN, 0x00);
		}
		if (second_data[1] == 1) {
			GPIO_WriteBit(NP2_D3_GPIO_PORT, NP2_D3_PIN, 0x01);
		}
		else {
			GPIO_WriteBit(NP2_D3_GPIO_PORT, NP2_D3_PIN, 0x00);
		}
		if (second_data[2] == 1) {
			GPIO_WriteBit(NP2_D4_GPIO_PORT, NP2_D4_PIN, 0x01);
		}
		else {
			GPIO_WriteBit(NP2_D4_GPIO_PORT, NP2_D4_PIN, 0x00);
		}
		if (second_data[3] == 1) {
			GPIO_WriteBit(NP2_D5_GPIO_PORT, NP2_D5_PIN, 0x01);
		}
		else {
			GPIO_WriteBit(NP2_D5_GPIO_PORT, NP2_D5_PIN, 0x00);
		}
		
		/* last d6 to d9 */
		if (last_data[0] == 1) {
			GPIO_WriteBit(NP2_D6_GPIO_PORT, NP2_D6_PIN, 0x01);
		}
		else {
			GPIO_WriteBit(NP2_D6_GPIO_PORT, NP2_D6_PIN, 0x00);
		}
		if (last_data[1] == 1) {
			GPIO_WriteBit(NP2_D7_GPIO_PORT, NP2_D7_PIN, 0x01);
		}
		else {
			GPIO_WriteBit(NP2_D7_GPIO_PORT, NP2_D7_PIN, 0x00);
		}
		if (last_data[2] == 1) {
			GPIO_WriteBit(NP2_D8_GPIO_PORT, NP2_D8_PIN, 0x01);
		}
		else {
			GPIO_WriteBit(NP2_D8_GPIO_PORT, NP2_D8_PIN, 0x00);
		}
		if (last_data[3] == 1) {
			GPIO_WriteBit(NP2_D9_GPIO_PORT, NP2_D9_PIN, 0x01);
		}
		else {
			GPIO_WriteBit(NP2_D9_GPIO_PORT, NP2_D9_PIN, 0x00);
		}
		
		/* Toggle 'Keep Alive Indicator' BLUE LED */
		
    	Delay(0x7FFF00);	//Delay 1s
	}
}

/**
  * @brief  Initialise Hardware  
  * @param  None
  * @retval None
  */
void Hardware_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;

	NP2_LEDInit();		//Initialise Blue LED
	NP2_LEDOff();		//Turn off Blue LED

	/* Enable the GPIO D0 & A2 Clock */
  	RCC_AHB1PeriphClockCmd(NP2_D0_GPIO_CLK, ENABLE);
  	RCC_AHB1PeriphClockCmd(NP2_A2_GPIO_CLK, ENABLE);

  	/* Configure the GPIO pin */
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* Configure the GPIO_D0 to GPIO_D9 pins */
	GPIO_InitStructure.GPIO_Pin = NP2_D0_PIN;
  	GPIO_Init(NP2_D0_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = NP2_D1_PIN;
  	GPIO_Init(NP2_D1_GPIO_PORT, &GPIO_InitStructure);

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

	/* Configure A2 interrupt for Prac 1, Task 2 or 3 only */
	NVIC_InitTypeDef   NVIC_InitStructure;
	EXTI_InitTypeDef   EXTI_InitStructure;

	/* Enable SYSCFG clock for interrupt hardware */
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
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
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

/**
  * @brief  NP2_A2_EXTI Interrupt handler - see netduinoplus2.h
  * @param  None.
  * @retval None
  */
void NP2_A2_EXTI_IRQ_HANDLER(void) {

	/* Check if interrupt has occured - see ex3_gpio_interrupt */
	if (EXTI_GetITStatus(NP2_A2_EXTI_LINE) != RESET) {
		c_counter++;
		debug_printf("mode = %c value = %d %d %d \n\r", RxChar, first, second, last);
		Delay(0x9999);

    	/* Clear the EXTI line 0 pending bit */
    	EXTI_ClearITPendingBit(NP2_A2_EXTI_LINE);
  	}
}
