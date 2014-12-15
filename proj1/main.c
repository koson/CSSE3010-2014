/*
 ******************************************************************************
 * @file    proj1/main.c 
 * @author  HONG RUI, CHONG
 * @date    03-04-2014
 * @brief   Project 1 Task 1-3 file - pan/tilt servo control.
 ******************************************************************************
 */ 

/* Includes ------------------------------------------------------------------*/
#include "netduinoplus2.h"
#include "stm32f4xx_conf.h"
#include "debug_printf.h"
#include "nrf24l01plus.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Mode Selection: 0 = None, 1 = Joystick, 2 = WASD */
volatile uint8_t ctrl_mode = 1;
/* Transmit Mode Toggle */
volatile uint8_t transmit_toggle = 0;
/* Laser Mode Toggle */
volatile uint8_t laser_toggle = 0;
/* External Mode Toggle */
volatile uint8_t external_toggle = 0;
/* found Mode flag*/
volatile uint8_t partner_found = 0;
volatile uint8_t found_count = 0;
volatile uint8_t found_toggle = 0;
volatile uint8_t found_flag = 0;
/* allow find flag*/
volatile uint8_t allow_found = 0;
/* for servo control */
volatile static uint16_t x_pulse = 765;
volatile static uint16_t y_pulse = 645;
uint16_t nega_xlimit = 1237;
uint16_t posi_xlimit = 293;
uint16_t nega_ylimit = 1145;
uint16_t posi_ylimit = 250;
/* for RF transmit/receive */
uint8_t rf_channel = 48;
uint8_t packet_header[] = {0xA1, 0x72, 0x61, 0x03, 0x43, 0x28, 0x92, 0x96, 0x42};
uint8_t des_addr[] = {0x72, 0x61, 0x03, 0x43};
uint8_t scr_addr[] = {0x28, 0x92, 0x96, 0x42};
/* for laser transmit */
volatile uint8_t laser_tx_cnt = 0;
volatile uint8_t laser_tx_full = 0;
uint8_t laser_tx_packet[44];
/* for photodiode receive */
volatile uint8_t repeatbyte = 0;
volatile uint8_t decodedbyte = 0;
volatile uint8_t laser_rx_cnt = 0;
volatile uint8_t laser_rx_toggle = 1;
volatile uint8_t laser_speed_cnt = 0;
volatile uint8_t reset_counter = 0;
uint8_t laser_rx_packet[38];
volatile uint8_t inputbmc_cap1 = 23;	//			  10/01 			 00/11
volatile uint8_t inputbmc_cap2 = 27;	//1kHz: 23 <= input <= 27, 48 <= input <= 52
volatile uint8_t inputbmc_cap3 = 48;	//2kHz: 10 <= input <= 14, 23 <= input <= 27
volatile uint8_t inputbmc_cap4 = 52;
/* for VCP_getchar */
uint8_t RxChar;
/* Private function prototypes -----------------------------------------------*/
void Hardware_PWM_init(void);
void Hardware_ADC_init(void);
void Hardware_ISR_init(void);
void Hardware_SPI_init(void);
void Hardware_LSR_init(void);
void servo_ctrl(void);
void rf_transmit(uint8_t readchar);
void rf_receive(void);
void laser_transmit(uint8_t readchar);
void laser_receive(void);
void external_control(void);
void find_friend(void);
uint8_t spi_sendbyte(uint8_t sendbyte);
uint16_t hamming_byte_encoder(uint8_t input);
uint8_t hamming_hbyte_encoder(uint8_t input);
uint8_t hamming_byte_decoder(uint8_t input1, uint8_t input2);
uint8_t hamming_hbyte_decoder(uint8_t input);
void Delay(__IO unsigned long nCount);
uint8_t mode_toggle (uint8_t);
uint8_t frequency_change (uint8_t);
void NP2_A2_EXTI_IRQ_HANDLER(void);
void TIM5_IRQHandler (void);

/***************************************************************************************************************************
 ****************************************************************************************************************************
														MAIN PROGRAM														
 ****************************************************************************************************************************
 ***************************************************************************************************************************/
void main(void) {

	NP2_boardinit();	//Initalise NP2
	nrf24l01plus_init();
	Hardware_PWM_init();
	Hardware_ADC_init();
	Hardware_ISR_init();
	Hardware_SPI_init();
	Hardware_LSR_init();
	ADC_SoftwareStartConv(ADC1);	//Perform ADC1 conversions
	ADC_SoftwareStartConv(ADC2);	//Perform ADC2 conversions
	NP2_LEDInit();					//Initialise Blue LED
	Delay(0xFFFFFF);

	/* Set Rx Mode */
	nrf24l01plus_mode_rx();

	/* Main processing loop */
	while (1) {

		NP2_LEDToggle();	//Toggle LED on/off
		if (transmit_toggle == 1) {
			while(transmit_toggle == 1) {
				if (VCP_getchar(&RxChar)) {
					rf_transmit(RxChar);
				}
				Delay(10000);
				rf_receive();
				Delay(10000);
			}
		}
		else if (laser_toggle == 1) {
			while(laser_toggle == 1) {
				if (VCP_getchar(&RxChar)) {
					laser_transmit(RxChar);
				}
				Delay(10000);
				rf_receive();
				Delay(10000);
			}
		}
		else if (external_toggle == 1) {
			while(external_toggle == 1) {
				external_control();
				Delay(10000);
			}
		}
		else if (found_toggle == 1) {
			find_friend();
		}
		else {
			servo_ctrl();
			Delay(10000);
			rf_receive();
			Delay(10000);
		}
	}
}

/***********************************************************************************************************************
 ************************************************************************************************************************
											HARDWARE INTIALISATION FUNCTIONS											
 ************************************************************************************************************************
 ***********************************************************************************************************************/

/***********************************
	Hardware Init PWM for Servos
 ***********************************/
void Hardware_PWM_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	uint16_t original_x_pulse = 765;
	uint16_t original_y_pulse = 645;
	uint16_t PrescalerValue = 0;

	/* Enable the GPIO D2/D3 Clock */
	RCC_AHB1PeriphClockCmd(NP2_D2_GPIO_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(NP2_D3_GPIO_CLK, ENABLE);

	/* TIM2 clock enable */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	/* Configure the D2/D3 pin for PWM output */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = NP2_D2_PIN;
	GPIO_Init(NP2_D2_GPIO_PORT, &GPIO_InitStructure);	//Struct for D2
	GPIO_InitStructure.GPIO_Pin = NP2_D3_PIN;
	GPIO_Init(NP2_D3_GPIO_PORT, &GPIO_InitStructure);	//Struct for D3	

	/* Connect TIM2 output to D2/D3 pin */
	GPIO_PinAFConfig(NP2_D2_GPIO_PORT, NP2_D2_PINSOURCE, GPIO_AF_TIM2);
	GPIO_PinAFConfig(NP2_D3_GPIO_PORT, NP2_D3_PINSOURCE, GPIO_AF_TIM2);

	/* Compute the prescaler value. SystemCoreClock = 168000000 - set for 500Khz clock */
	PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 500000) - 1;

	/* Time 2 mode and prescaler configuration */
	TIM_TimeBaseStructure.TIM_Period = 10000;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	/* Configure Timer 2 mode and prescaler */
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* PWM Mode configuration for Channel3/4 - set pulse width*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				//Set PWM MODE (1 or 2 - NOT CHANNEL)
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_Pulse = original_y_pulse;
	TIM_OC3Init(TIM2, &TIM_OCInitStructure);						//Channel 3 - D3
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);
	TIM_OCInitStructure.TIM_Pulse = original_x_pulse;
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);						//Channel 4 - D2
	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);
}

/***********************************
	Hardware Init ADC for Servos
 ***********************************/
void Hardware_ADC_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;

	/* Configure A0/A1/A2 as analog input */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = NP2_A0_PIN;
	GPIO_Init(NP2_A0_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = NP2_A1_PIN;
	GPIO_Init(NP2_A1_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = NP2_A2_PIN;
	GPIO_Init(NP2_A2_GPIO_PORT, &GPIO_InitStructure);

	/* Enable clock for ADC 1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);

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
	ADC_Init(ADC2, &ADC_InitStructure);

	/* Configure ADC1 to connect to the NP2 A0 channel */
	ADC_RegularChannelConfig(ADC1, NP2_A0_ADC_CHAN, 1, ADC_SampleTime_3Cycles);
	ADC_RegularChannelConfig(ADC2, NP2_A1_ADC_CHAN, 1, ADC_SampleTime_3Cycles);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);
	ADC_Cmd(ADC2, ENABLE);
}

/*********************************
	Hardware Init ISR for Mode
 *********************************/
void Hardware_ISR_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;
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

/****************************************
	Hardware Init SPI for Transceiver
 ****************************************/
void Hardware_SPI_init(void) {

	uint8_t i;

	GPIO_InitTypeDef GPIO_spi;	
	SPI_InitTypeDef	NP2_spiInitStruct;
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHB1PeriphClockCmd(NP2_D10_GPIO_CLK, ENABLE);

	/* Enacble SPI clock */
	RCC_APB1PeriphClockCmd(NP2_SPI_CLK, ENABLE);

	/* SPI SCK pin configuration */
	GPIO_spi.GPIO_Mode = GPIO_Mode_AF;		//Alternate function
	GPIO_spi.GPIO_OType = GPIO_OType_PP;
	GPIO_spi.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_spi.GPIO_PuPd  = GPIO_PuPd_DOWN;	//Pull down resistor
	GPIO_spi.GPIO_Pin = NP2_SPI_SCK_PIN;
	GPIO_Init(NP2_SPI_SCK_GPIO_PORT, &GPIO_spi);

	/* SPI MISO pin configuration */
	GPIO_spi.GPIO_Mode = GPIO_Mode_AF;		//Alternate function
	GPIO_spi.GPIO_OType = GPIO_OType_PP;
	GPIO_spi.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_spi.GPIO_PuPd  = GPIO_PuPd_UP;		//Pull up resistor
	GPIO_spi.GPIO_Pin = NP2_SPI_MISO_PIN;
	GPIO_Init(NP2_SPI_MISO_GPIO_PORT, &GPIO_spi);

	/* SPI  MOSI pin configuration */
	GPIO_spi.GPIO_Mode = GPIO_Mode_AF;		//Alternate function
	GPIO_spi.GPIO_OType = GPIO_OType_PP;	
	GPIO_spi.GPIO_PuPd  = GPIO_PuPd_DOWN;	//Pull down resistor	
	GPIO_spi.GPIO_Pin =  NP2_SPI_MOSI_PIN;
	GPIO_Init(NP2_SPI_MOSI_GPIO_PORT, &GPIO_spi);

	/* Connect SPI pins */
	GPIO_PinAFConfig(NP2_SPI_SCK_GPIO_PORT, NP2_SPI_SCK_SOURCE, NP2_SPI_SCK_AF);
	GPIO_PinAFConfig(NP2_SPI_MISO_GPIO_PORT, NP2_SPI_MISO_SOURCE, NP2_SPI_MISO_AF);
	GPIO_PinAFConfig(NP2_SPI_MOSI_GPIO_PORT, NP2_SPI_MOSI_SOURCE, NP2_SPI_MOSI_AF);

	/* SPI configuration */
	SPI_I2S_DeInit(NP2_SPI);

	NP2_spiInitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	NP2_spiInitStruct.SPI_DataSize = SPI_DataSize_8b;
	NP2_spiInitStruct.SPI_CPOL = SPI_CPOL_Low;	//SPI_CPOL_Low;
	NP2_spiInitStruct.SPI_CPHA = SPI_CPHA_1Edge;	//SPI_CPHA_1Edge;
	NP2_spiInitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set;	//SPI_NSS_Soft;
	NP2_spiInitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;	//256;
	NP2_spiInitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	NP2_spiInitStruct.SPI_CRCPolynomial = 0;	//7;
	NP2_spiInitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_Init(NP2_SPI, &NP2_spiInitStruct);

	/* Enable NP2 external SPI */
	SPI_Cmd(NP2_SPI, ENABLE);

	/* Configure GPIO PIN for Chip select */
	GPIO_InitStructure.GPIO_Pin = NP2_SPI_CS_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(NP2_SPI_CS_GPIO_PORT, &GPIO_InitStructure);

	/* Set chip select high */
	GPIO_SetBits(NP2_D10_GPIO_PORT, NP2_D10_PIN);

	/* Write TX_ADDR */
	GPIO_ResetBits(NP2_D10_GPIO_PORT, NP2_D10_PIN);		//Set Chip Select low
	spi_sendbyte(0x30);									//Command Write to TX_ADDR Register
	for (i = 0; i < 4; i++) {
		spi_sendbyte(des_addr[i]);
	}
	spi_sendbyte(0x00);
	GPIO_SetBits(NP2_D10_GPIO_PORT, NP2_D10_PIN);		//Set Chip Select high

	/* Write RF_CH */
	GPIO_ResetBits(NP2_D10_GPIO_PORT, NP2_D10_PIN);
	spi_sendbyte(0x25);									//Command Write to RF_CH Register
	spi_sendbyte(rf_channel);							//Set to rf_ch
	GPIO_SetBits(NP2_D10_GPIO_PORT, NP2_D10_PIN);

	/* Write RX_ADDR_P1 */
	GPIO_ResetBits(NP2_D10_GPIO_PORT, NP2_D10_PIN);	
	spi_sendbyte(0x2A);								
	for (i = 0; i < 4; i++)
	{
		spi_sendbyte(scr_addr[i]);
	}
	spi_sendbyte(0x00);
	GPIO_SetBits(NP2_D10_GPIO_PORT, NP2_D10_PIN);
}

/**************************************************************
	Hardware Init Laser and Photodiode for Transmit/Receive
 **************************************************************/
void Hardware_LSR_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure; //GPIO
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_ICInitTypeDef  TIM_ICInitStructure; //TIM Input
	TIM_OCInitTypeDef  TIM_OCInitStructure; //TIM Output
	NVIC_InitTypeDef   NVIC_InitStructure;
	uint16_t PrescalerValue = 0;

	/* Enable the GPIO D0/D1/A0 Clock */
	RCC_AHB1PeriphClockCmd(NP2_D0_GPIO_CLK, ENABLE); //GPIO D0 CLK
	RCC_AHB1PeriphClockCmd(NP2_D1_GPIO_CLK, ENABLE); //GPIO D1 CLK
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE); //APB1 TIM5
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //APB1 TIM3

	/* Configure the GPIO_D1 pin for output */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin = NP2_D1_PIN;
	GPIO_Init(NP2_D1_GPIO_PORT, &GPIO_InitStructure);

	/* Configure the GPIO_D0 pin for photodiode input capture */
	GPIO_InitStructure.GPIO_Pin = NP2_D0_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_Init(NP2_D0_GPIO_PORT, &GPIO_InitStructure);

	/* Connect TIM3 output to D0 pin for input capture */  
	GPIO_PinAFConfig(NP2_D0_GPIO_PORT, NP2_D0_PINSOURCE, GPIO_AF_TIM3);

	/* Configure TIM 5 for compare update interrupt. */
	/* Compute the prescaler value */
	PrescalerValue = (uint16_t) ((SystemCoreClock/2)/50000) - 1;

	/* Time base configuration */
	TIM_TimeBaseStructure.TIM_Period = 25;
	TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

	/* Set Reload Value for 1s */
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

	/* Configure TIM 3 for input capture interrupt. */
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* Configure TIM3 Input capture for channel 2 */
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_BothEdge;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0x0;

	TIM_ICInit(TIM3, &TIM_ICInitStructure);

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
	TIM_SetAutoreload(TIM3, 50000);
	TIM_Cmd(TIM3, ENABLE); 
}


/***********************************************************************************************************************
 ************************************************************************************************************************
													PRIVATE FUNCTION													
 ************************************************************************************************************************
 ***********************************************************************************************************************/

/*********************************
	Servos Control, Mode 1 & 2 
 *********************************/
void servo_ctrl(void) {

	volatile static int16_t x_degree = 0;
	volatile static int16_t y_degree = 0;
	volatile static uint16_t x_adc = 2050;
	volatile static uint16_t y_adc = 2050;
	volatile static uint16_t print_counter = 0;
	uint8_t readchar = 0;
	//GPIO_WriteBit(NP2_D1_GPIO_PORT, NP2_D1_PIN, 0x01);

	/* Mode 1 = Joystick Control */
	if (ctrl_mode == 1) {
		/* Wait for ADC conversion to finish by polling the ADC Over Flag. */
		while ((ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR) != RESET) && (ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR) != RESET));
		while ((ADC_GetFlagStatus(ADC2, ADC_FLAG_OVR) != RESET) && (ADC_GetFlagStatus(ADC2, ADC_FLAG_OVR) != RESET));

		/* Extract ADC conversion values */
		x_adc = ADC_GetConversionValue(ADC1);
		y_adc = ADC_GetConversionValue(ADC2);

		/* Increase/Decrease pulse by checking ADC value */
		if (x_adc < 1900) {
			x_pulse = x_pulse + 3;
		}
		else if (x_adc > 2200) {
			x_pulse = x_pulse - 3;
		}
		if (y_adc < 1900) {
			y_pulse = y_pulse + 3;
		}
		else if (y_adc > 2200) {
			y_pulse = y_pulse - 3;
		}
		ADC_ClearFlag(ADC1, ADC_FLAG_OVR);
		ADC_ClearFlag(ADC2, ADC_FLAG_OVR);
		if (VCP_getchar(&readchar));
	}
	/* Mode 2 = WASD Control */
	if (ctrl_mode == 2) {
		/* Check for keystroke and Increase/Decrease pulse */
		if (VCP_getchar(&readchar)) {
			if (readchar == 'W') {
				y_pulse = y_pulse - 3;
			}
			else if (readchar == 'A') {
				x_pulse = x_pulse + 3;
			}
			else if (readchar == 'S') {
				y_pulse = y_pulse + 3;
			}
			else if (readchar == 'D') {
				x_pulse = x_pulse - 3;
			}
		}
	}

	/* Calculate/Estimate Angle */
	if (x_pulse <= 765) {
		x_degree = (765-x_pulse)/6;
	}
	else if (x_pulse > 765) {
		x_degree = -((x_pulse-765)/6);
	}
	if (y_pulse <= 645) {
		y_degree = (645-y_pulse)/5;
	}
	else if (y_pulse > 645) {
		y_degree = -((y_pulse-645)/5);
	}

	/* Limit maximum servo angle */
	if (x_pulse >= nega_xlimit) {
		x_pulse = nega_xlimit;
		x_degree = -85;
	}
	if (x_pulse <= posi_xlimit) {
		x_pulse = posi_xlimit;
		x_degree = 85;
	}
	if (y_pulse >= nega_ylimit) {
		y_pulse = nega_ylimit;
		y_degree = -95;
	}
	if (y_pulse <= posi_ylimit) {
		y_pulse = posi_ylimit;
		y_degree = 90;
	}

	if (print_counter == 100) {
		debug_printf("Pan: %d, Tilt: %d \r\n", x_degree, y_degree);
		print_counter = 0;
	}

	/* Set PWM */
	TIM_SetCompare3 (TIM2, y_pulse);
	TIM_SetCompare4 (TIM2, x_pulse);
	mode_toggle(readchar);
	frequency_change(readchar);
	print_counter++;

	Delay(0x10000);
}

/***************************
	RF Transmit Function
 ***************************/
void rf_transmit(uint8_t readchar) {

	uint8_t i;
	static uint8_t tx_buffer[32];
	static uint8_t payload[20];
	static uint8_t payload_counter = 0;
	//uint8_t readchar;

	//encode packet headers and put to packet buffer
	for (i = 0; i < 9; i++) {
		tx_buffer[i] = packet_header[i]; //tx_buffer 0 to 8
	}

	if (mode_toggle(readchar) == 0 && frequency_change(readchar) == 0) {
		if (readchar == 13) {
			payload_counter = 20;
		}
		else {
			payload[payload_counter] = readchar;
			payload_counter++;
		}
		if (payload_counter == 20) { //encode payload and put to packet buffer
			debug_printf("Sent From Radio: ");
			for (i = 0; i < 20; i++) {
				debug_printf("%c", payload[i]);
				tx_buffer[9+i] = payload[i]; //tx_buffer 9 to 31
				payload[i] = 0; 			//clear payload
			}
			debug_printf("\r\n");
			nrf24l01plus_send_packet(tx_buffer);
			for (i = 0; i < 32; i++) {
				tx_buffer[i] = 0; //clear tx_buffer
			}
			Delay(50000);
			nrf24l01plus_mode_rx(); //change to rx mode
			payload_counter = 0;
		}
	}
}

/**************************
	RF Receive Function
 **************************/
void rf_receive(void) {

	uint8_t i;
	static uint8_t rx_buffer[32];
	uint8_t error[] = {'E', 'R', 'R', 'O', 'R'};
	uint8_t found[] = {'F', 'O', 'U', 'N', 'D'};
	volatile static uint8_t addr_check = 0, error_check = 0, found_check = 0;

	nrf24l01plus_mode_rx(); //change to rx mode

	for (i = 0; i < 32; i++) {
		rx_buffer[i] = 0; //clear rx_buffer
	}
	if (nrf24l01plus_receive_packet(rx_buffer) == 1) {
		debug_printf("\r\nReceived From Radio: ");
		/*for (i = 0; i < 32; i++ ) {
			debug_printf("%x ", rx_buffer[i]);
		}
		debug_printf("\r\n");*/
		for (i = 0; i < 4; i++) {
			if (rx_buffer[1+i] == scr_addr[i]) {
				addr_check++;
			}
		}
		for (i = 0; i < 5; i++) {
			if (rx_buffer[9+i] == error[i]) {
				error_check++;
			}
		}
		for (i = 0; i < 5; i++) {
			if (rx_buffer[9+i] == found[i]) {
				found_check++;
			}
		}
		if (addr_check == 4) {
			for (i = 0; i < 20; i++) {
				debug_printf("%c ", rx_buffer[9+i]);
			}
			addr_check = 0;
		}
		debug_printf("\r\n");
		if (error_check == 5) {
			laser_transmit(repeatbyte);
			Delay(500);
			error_check = 0;
		}
		else {
			//debug_printf("Repeat = %c", repeatbyte);
			repeatbyte = 0;
			error_check = 0;
		}
		if (found_check == 5) {
			found_count++;
			debug_printf("found count = %d\r\n", found_count);
			found_check = 0;
		}
		for (i = 0; i < 32; i++) {
			rx_buffer[i] = 0; //clear rx_buffer
		}
	}
}

/******************************
	Laser Transmit Function
 ******************************/
void laser_transmit(uint8_t readchar) {

	uint8_t i;
	uint16_t encodedbyte;
	int8_t upperbyte, lowerbyte;
	uint8_t laser_tx_buffer[22];

	laser_tx_buffer[0] = 1;
	laser_tx_buffer[1] = 1;
	laser_tx_buffer[10] = 0;
	laser_tx_buffer[11] = 1;
	laser_tx_buffer[12] = 1;
	laser_tx_buffer[21] = 0;


	if (mode_toggle(readchar) == 0 && frequency_change(readchar) == 0) {
		debug_printf("\r\nSending From Laser: %c - Raw: %x \r\n", readchar, readchar);
		encodedbyte = hamming_byte_encoder(readchar);
		if (repeatbyte == 0) {
			repeatbyte = readchar;
		}
		lowerbyte = (encodedbyte & 0xFF);
		upperbyte = ((encodedbyte >> 8) & 0xFF);

		//populate laser_tx_buffer with 22bits of hamming encoded data with start/stop bits.
		for (i = 0; i < 8; i++) {
			laser_tx_buffer[2+i] = !!((lowerbyte & (1 << i)) >> i);
		}
		for (i = 0; i < 8; i++) {
			laser_tx_buffer[13+i] = !!((upperbyte & (1 << i)) >> i);
		}
		debug_printf("Hamming Encoded MSG = ");
		for (i = 0; i < 22; i++) {
			debug_printf("%d", laser_tx_buffer[i]);
		}
		debug_printf("\r\n");
		//populate laser_tx_packet with 44bits of data after BCM
		for (i = 0; i < 22; i++) {
			if (laser_tx_buffer[i] == 1) {
				if (laser_tx_packet[(2*i)-1] == 0) {
					laser_tx_packet[2*i] = 1;
					laser_tx_packet[(2*i)+1] = 0;
				} 
				else if (laser_tx_packet[(2*i)-1] == 1) {
					laser_tx_packet[2*i] = 0;
					laser_tx_packet[(2*i)+1] = 1;
				}
			}
			else if (laser_tx_buffer[i] == 0) {
				if (laser_tx_packet[(2*i)-1] == 0) {
					laser_tx_packet[2*i] = 1;
					laser_tx_packet[(2*i)+1] = 1;
				} 
				else if (laser_tx_packet[(2*i)-1] == 1) {
					laser_tx_packet[2*i] = 0;
					laser_tx_packet[(2*i)+1] = 0;
				}
			}
			laser_tx_full = i;
			//debug_printf("laser tx = %d\r\n", laser_tx_full);
		}
		debug_printf("BMC Encoded MSG = ");
		for (i = 0; i < 44; i++) {
			debug_printf("%d", laser_tx_packet[i]);
		}
		debug_printf("\r\n");
		laser_tx_cnt = 0;
		laser_rx_cnt = 0;
		laser_speed_cnt = 0;
	}
}

/*****************************
	Laser Receive function
 *****************************/
void laser_receive(void) {

	uint8_t i;
	static uint8_t upperbyte = 0, lowerbyte = 0;
	uint8_t laser_rx_buffer[20];

	for (i = 0; i < 20; i++)
	{
		laser_rx_buffer[i] = 0;
	}
	for (i = 0; i < 20; i++)
	{
		laser_rx_buffer[i] = laser_rx_packet[2*i] ^ laser_rx_packet[(2*i)+1];
	}
	for (i = 0; i < 38; i++) {
		laser_rx_packet[i] = 0;
	}
	debug_printf("Hamming Decoded MSG = ");
	for (i = 0; i < 20; i++){
		debug_printf("%d", laser_rx_buffer[i]);
	}
	debug_printf("\r\n");
	for (i = 0; i < 8; i++) {
		lowerbyte = laser_rx_buffer[i] | (lowerbyte);
		if (i < 7) {
			lowerbyte = lowerbyte << 1;
		}
	}
	for (i = 0; i < 8; i++) {
		upperbyte = laser_rx_buffer[i+11] | (upperbyte);
		if (i < 7) {
			upperbyte = upperbyte << 1;
		}
	}

	hamming_byte_decoder(upperbyte, lowerbyte);

	lowerbyte = 0;
	upperbyte = 0;
}

/*******************************
	SPI Sendbyte to Transmit
 *******************************/
void external_control (void) {

	uint8_t i;
	static uint8_t tx_buffer[32];
	uint8_t rx_buffer[32];
	volatile static uint8_t addr_check = 0;
	uint8_t sentchar = 0, readchar;

	if (VCP_getchar(&readchar)) {
		if (mode_toggle(readchar) == 0 && frequency_change(readchar) == 0) {
			if (readchar == 'W' || readchar == 'A' || readchar == 'S' || readchar == 'D') {
				//encode packet headers and put to packet buffer
				for (i = 0; i < 9; i++) {
					tx_buffer[i] = packet_header[i]; //tx_buffer 0 to 8
				}
				tx_buffer[9] = readchar;
				debug_printf("Sending: %c \r\n", tx_buffer[9]);
				nrf24l01plus_send_packet(tx_buffer);
				Delay(10000);
				nrf24l01plus_mode_rx(); //change to rx mode
				for (i = 0; i < 32; i++) {
					tx_buffer[i] = 0; //clear tx_buffer
				}
			}	
		}
	}
	Delay(10000);
	nrf24l01plus_mode_rx(); //change to rx mode

	if (nrf24l01plus_receive_packet(rx_buffer) == 1)
	{
		debug_printf("Received From Radio: ");			
		for (i = 0; i < 32; i++ ) {
			debug_printf("%x ", rx_buffer[i]);
		}
		debug_printf("\r\n");
		for (i = 0; i < 4; i++) {
			if (rx_buffer[1+i] == scr_addr[i]) {
				addr_check++;
			}
		}
		if (addr_check == 4) {
			sentchar = rx_buffer[9];
			debug_printf("%c\r\n", sentchar);
			addr_check = 0;
		}
		if (sentchar == 'W') {
			y_pulse = y_pulse - 3;
		}
		else if (sentchar == 'A') {
			x_pulse = x_pulse + 3;
		}
		else if (sentchar == 'S') {
			y_pulse = y_pulse + 3;
		}
		else if (sentchar == 'D') {
			x_pulse = x_pulse - 3;
		}
		for (i = 0; i < 32; i++) {
			rx_buffer[i] = 0; //clear rx_buffer
		}

		/* Limit maximum servo angle */
		if (x_pulse >= nega_xlimit) {
			x_pulse = nega_xlimit;
		}
		if (x_pulse <= posi_xlimit) {
			x_pulse = posi_xlimit;
		}
		if (y_pulse >= nega_ylimit) {
			y_pulse = nega_ylimit;
		}
		if (y_pulse <= posi_ylimit) {
			y_pulse = posi_ylimit;
		}

		TIM_SetCompare3 (TIM2, y_pulse);
		TIM_SetCompare4 (TIM2, x_pulse);
	}
}

/*******************************
	SPI Sendbyte to Transmit
 *******************************/
void find_friend(void) {

	uint8_t readchar = 0;
	GPIO_WriteBit(NP2_D1_GPIO_PORT, NP2_D1_PIN, 0x01);
	debug_printf("Finding Friend\r\n");

	if (found_flag == 0) {
		y_pulse = 1145;
		TIM_SetCompare3 (TIM2, y_pulse);
		Delay(40000);
		x_pulse = 765;
		TIM_SetCompare4 (TIM2, x_pulse);
		Delay(40000);
		while (y_pulse >= 1066) {
			while (x_pulse <= nega_xlimit){
				x_pulse = x_pulse + 3;
				TIM_SetCompare4 (TIM2, x_pulse);
				Delay(40000);
				rf_receive();
				if (found_count >= 1) {
					found_flag = 1;
					break;
				}
			}
			if (found_flag == 1) {
				break;
			}
			Delay(100000);
			y_pulse = y_pulse - 5;
			TIM_SetCompare3 (TIM2, y_pulse);
			Delay(40000);
			while(x_pulse >= posi_xlimit){
				x_pulse = x_pulse - 3;
				TIM_SetCompare4 (TIM2, x_pulse);
				Delay(40000);
				rf_receive();
				if (found_count >= 1) {
					found_flag = 1;
					break;
				}
			}
			if (found_flag == 1) {
				break;
			}
			Delay(100000);
			y_pulse = y_pulse - 5;
			TIM_SetCompare3 (TIM2, y_pulse);
			Delay(40000);
		}
		if (found_flag == 0) {
			y_pulse = 250;
			TIM_SetCompare3 (TIM2, y_pulse);
			Delay(40000);
		}
		while (y_pulse <= 328) {
			while (x_pulse <= nega_xlimit){
				x_pulse = x_pulse + 3;
				TIM_SetCompare4 (TIM2, x_pulse);
				Delay(40000);
				rf_receive();
				if (found_count >= 1) {
					found_flag = 1;
					break;
				}
			}
			if (found_flag == 1) {
				break;
			} 
			Delay(100000);
			y_pulse = y_pulse + 5;
			TIM_SetCompare3 (TIM2, y_pulse);
			Delay(40000);
			while(x_pulse >= posi_xlimit){
				x_pulse = x_pulse - 3;
				TIM_SetCompare4 (TIM2, x_pulse);
				Delay(40000);
				rf_receive();
				if (found_count >= 1) {
					found_flag = 1;
					break;
				}
			}
			if (found_flag == 1) {
				break;
			}
			Delay(100000);
			y_pulse = y_pulse + 5;
			TIM_SetCompare3 (TIM2, y_pulse);
			Delay(40000);
		}
	}
	if (found_flag == 1) {
		while(x_pulse >= posi_xlimit && found_flag == 1){
			x_pulse = x_pulse - 3;
			TIM_SetCompare4 (TIM2, x_pulse);
			rf_receive();
			Delay(800000);
			if (found_count >= 3) {
				found_flag = 2;
				found_count = 0;
				GPIO_WriteBit(NP2_D1_GPIO_PORT, NP2_D1_PIN, 0x01);
				Delay(800000);
				break;
			}
		}
		while (x_pulse <= nega_xlimit && found_flag == 1){
			x_pulse = x_pulse + 3;
			TIM_SetCompare4 (TIM2, x_pulse);
			rf_receive();
			Delay(800000);
			if (found_count >= 3) {
				found_flag = 2;
				found_count = 0;
				GPIO_WriteBit(NP2_D1_GPIO_PORT, NP2_D1_PIN, 0x01);
				Delay(800000);
				break;
			}
		}
	}
	if (found_flag == 1 || found_flag == 0) {
		found_toggle = 0;
		found_count = 0;
		Delay(50000);
		GPIO_WriteBit(NP2_D1_GPIO_PORT, NP2_D1_PIN, 0x00);
		debug_printf("No Friend Found, Forever Alone\r\n");
	}
	if (found_flag == 2) {
		GPIO_WriteBit(NP2_D1_GPIO_PORT, NP2_D1_PIN, 0x00);
		debug_printf("Friend Found\r\n");
		laser_toggle = 1;
		Delay(0xFFFFFF);
		laser_transmit('F');
		Delay(0xFFFFF);
		found_flag = 0;
		found_toggle = 0;
		found_count = 0;
		laser_toggle = 0;
	}
}

/*******************************
	SPI Sendbyte to Transmit
 *******************************/
uint8_t spi_sendbyte(uint8_t sendbyte) {

	uint8_t readbyte;

	/* Loop while DR register in not empty */
	while (SPI_I2S_GetFlagStatus(NP2_SPI, SPI_I2S_FLAG_TXE) == RESET);

	/* Send a Byte through the SPI peripheral */
	SPI_I2S_SendData(NP2_SPI, sendbyte);

	/* Wait while busy flag is set */
	while (SPI_I2S_GetFlagStatus(NP2_SPI, SPI_I2S_FLAG_BSY) == SET);

	/* Wait to receive a Byte */
	while (SPI_I2S_GetFlagStatus(NP2_SPI, SPI_I2S_FLAG_RXNE) == RESET);

	/* Return the Byte read from the SPI bus */
	readbyte = (uint8_t)SPI_I2S_ReceiveData(NP2_SPI);

	/* Return the Byte read from the SPI bus */
	return readbyte;
}

/***************************
	Hamming byte encoder
 ***************************/
uint16_t hamming_byte_encoder(uint8_t input) {

	uint16_t out;

	/* first encode D0..D3 (first 4 bits), 
	 * then D4..D7 (second 4 bits).
	 */
	out = hamming_hbyte_encoder(input & 0xF) | (hamming_hbyte_encoder(input >> 4) << 8);

	return(out);
}

/****************************
	Hamming hbyte encoder
 ****************************/
uint8_t hamming_hbyte_encoder(uint8_t input) {

	uint8_t i;
	uint8_t d0, d1, d2, d3;
	uint8_t p0, h0, h1, h2;
	uint8_t out;

	/* extract bits */
	d0 = !!(input & 0x1);
	d1 = !!(input & 0x2);
	d2 = !!(input & 0x4);
	d3 = !!(input & 0x8);

	/* calculate hamming parity bits */
	h0 = d1 ^ d2 ^ d3;
	h1 = d0 ^ d2 ^ d3;
	h2 = d0 ^ d1 ^ d3;

	/* generate out byte without parity bit P0 */
	out = (h0 << 1) | (h1 << 2) | (h2 << 3) | (d0 << 4) | (d1 << 5) | (d2 << 6) | (d3 << 7);

	/* calculate even parity bit */
	p0 = h0 ^ h1 ^ h2 ^ d0 ^ d1 ^ d2 ^ d3;

	out |= p0;

	return(out);
}

/***************************
	Hamming byte decoder
 ***************************/
uint8_t hamming_byte_decoder(uint8_t input_up, uint8_t input_low) {

	uint8_t i;
	volatile static uint8_t up[8], low[8];
	volatile static uint8_t out = 0;
	volatile static uint8_t new_upbyte = 0, new_lowbyte = 0;
	volatile static uint16_t new_16 = 0, old_16 = 0;
	uint16_t errormask, errorcount = 0;

	for (i = 0; i < 8; i++){
		up[i] = 0;
		low[i] = 0;
	}

	old_16 = (input_up << 8) | input_low;
	new_upbyte = hamming_hbyte_decoder(input_up);
	new_lowbyte = hamming_hbyte_decoder(input_low);
	new_16 = (new_upbyte << 8) | new_lowbyte;
	errormask = new_16 ^ old_16;
	for (i = 0; i < 16; i++) {
		errorcount =  ((errormask>>i)&1) + errorcount;
	}
	debug_printf("errorcount: %d \r\n", errorcount);

	up[0] = !!(new_upbyte & 0x1);		//d3
	up[1] = !!(new_upbyte & 0x2);		//d2
	up[2] = !!(new_upbyte & 0x4);		//d1
	up[3] = !!(new_upbyte & 0x8);		//d0
	up[4] = !!(new_upbyte & 0x10);		//h2
	up[5] = !!(new_upbyte & 0x20);		//h1
	up[6] = !!(new_upbyte & 0x40);		//h0
	up[7] = !!(new_upbyte & 0x80);		//p0
	low[0] = !!(new_lowbyte & 0x1);		//d3
	low[1] = !!(new_lowbyte & 0x2);		//d2
	low[2] = !!(new_lowbyte & 0x4);		//d1
	low[3] = !!(new_lowbyte & 0x8);		//d0
	low[4] = !!(new_lowbyte & 0x10);	//h2
	low[5] = !!(new_lowbyte & 0x20);	//h1
	low[6] = !!(new_lowbyte & 0x40);	//h0
	low[7] = !!(new_lowbyte & 0x80);	//p0

	decodedbyte = (up[0] << 7) | (up[1] << 6) | (up[2] << 5) | (up[3] << 4) | (low[0] << 3) | (low[1] << 2) | (low[2] << 1) | low[3];

	debug_printf("Receive From Laser: %c - Raw: %x (ErrorMask %04x)\r\n", decodedbyte, decodedbyte, errormask);

	if (errorcount >= 3) {
		rf_transmit('E');
		Delay(500);
		rf_transmit('R');
		Delay(500);
		rf_transmit('R');
		Delay(500);
		rf_transmit('O');
		Delay(500);
		rf_transmit('R');
		Delay(500);
		rf_transmit(13);
		Delay(500);
	}
	else if (errorcount == 0) {
		rf_transmit(decodedbyte);
		Delay(500);
		rf_transmit(13);
		Delay(500);
	}
}

/****************************
	Hamming hbyte decoder
 ****************************/
uint8_t hamming_hbyte_decoder(uint8_t input) {

	uint8_t ex[8];
	uint8_t syn1, syn2, syn3, syndrome, pcheck;
	uint8_t syndrome_array[] = {111, 110, 101, 11, 1, 10, 100};
	uint8_t i;
	uint8_t out;

	/* extract bits */
	ex[0] = !!(input & 0x1);		//d3
	ex[1] = !!(input & 0x2);		//d2
	ex[2] = !!(input & 0x4);		//d1
	ex[3] = !!(input & 0x8);		//d0
	ex[4] = !!(input & 0x10);		//h2
	ex[5] = !!(input & 0x20);		//h1
	ex[6] = !!(input & 0x40);		//h0
	ex[7] = !!(input & 0x80);		//p0

	syn1 = ex[0] ^ ex[1] ^ ex[2] ^ ex[6];
	syn2 = ex[0] ^ ex[1] ^ ex[3] ^ ex[5];
	syn3 = ex[0] ^ ex[2] ^ ex[3] ^ ex[4];
	syndrome = (syn1 * 100) + (syn2 * 10) + syn3;

	if (syndrome != 000) {
		for (i = 0; i < 7; i++) {
			if (syndrome == syndrome_array[i]) {
				debug_printf("old: %d\r\n", ex[i]);
				ex[i] = !ex[i];
				debug_printf("Syndrome %d: %d, new: %d\r\n", i, syndrome_array[i], ex[i]);
				i++;
			}
		}
	}

	pcheck = ex[0] ^ ex[1] ^ ex[2] ^ ex[3] ^ ex[4] ^ ex[5] ^ ex[6];

	if (syndrome == 000 && pcheck != ex[7]) {
		//debug_printf("ori = %d, pcheck = %d\r\n", ex[7], pcheck);
		ex[7] = !ex[7];
	}
	out = (ex[7] << 7) | (ex[6] << 6) | (ex[5] << 5) | (ex[4] << 4) | (ex[3] << 3) | (ex[2] << 2) | (ex[1] << 1) | ex[0];

	return out;

}

/*********************
	Delay Function
 *********************/
void Delay(__IO unsigned long nCount) {

	/* Delay a specific amount before returning */
	while(nCount--)	{
	}
}

/***************************************************
	Toggle between Laser and RF Mode to transmit
 ***************************************************/
uint8_t mode_toggle(uint8_t input_char) {

	uint8_t output = 0;

	if (input_char == '~') { //detecting ~ means RF mode
		laser_toggle = 0;
		transmit_toggle = 1;
		external_toggle = 0;
		found_toggle = 0;
		output = 1;
		debug_printf("RF Transmit Mode \r\n");
	}
	else if (input_char == '!') { //detecting ! means laser mode
		laser_toggle = 1;
		transmit_toggle = 0;
		external_toggle = 0;
		found_toggle = 0;
		output = 1;
		debug_printf("Laser Transmit Mode \r\n");
	}
	else if (input_char == '@') { //detecting @ means servo mode.
		transmit_toggle = 0;
		laser_toggle = 0;
		external_toggle = 0;
		found_toggle = 0;
		output = 1;
		debug_printf("Servo Mode \r\n");
	}
	else if (input_char == '#') { //detecting # means servo mode.
		laser_toggle = 0;
		transmit_toggle = 0;
		external_toggle = 1;
		found_toggle = 0;
		output = 1;
		debug_printf("External Mode \r\n");
	}
	else if (input_char == '$') { //detecting $ means servo mode.
		laser_toggle = 0;
		transmit_toggle = 0;
		external_toggle = 0;
		found_toggle = 1;
		output = 1;
		found_flag = 0;
		debug_printf("Find Friend Mode \r\n");
	}
	else if (input_char == '%') { //detecting $ means servo mode.
		allow_found = !allow_found;
		output = 1;
		partner_found = 0;
		debug_printf("Find Me Mode: %d \r\n", allow_found);
	}
	else {
		output = 0;
	}
	return output;
}

/***********************************
	Toggle between 1kHz and 2kHz
 ***********************************/
uint8_t frequency_change (uint8_t input_char) {

	uint8_t output = 0;

	if (input_char == '+') {
		TIM_SetAutoreload(TIM5, 12);
		output = 1;
		debug_printf("2kHz Selected!\r\n");
	}
	else if (input_char == '_') {
		TIM_SetAutoreload(TIM5, 25);
		output = 1;
		debug_printf("1kHz Selected!\r\n");
	}
	else {
		output = 0;
	}
	return output;
}

/*****************************
	Button Press Interrupt
 *****************************/
void NP2_A2_EXTI_IRQ_HANDLER(void) {

	volatile uint8_t read = 0;

	/* Check if interrupt has occured - see ex3_gpio_interrupt */
	if (EXTI_GetITStatus(NP2_A2_EXTI_LINE) != RESET) {
		if (ctrl_mode == 1) {
			ctrl_mode = 2;
			debug_printf("Mode 2, WASD Mode Selected! \r\n");
		}
		else if (ctrl_mode == 2) {
			ctrl_mode = 1;
			debug_printf("Mode 1, Joystick Mode Selected! \r\n");
		}

		do {
			Delay(0xFFF);
			read = GPIO_ReadInputDataBit(NP2_A2_GPIO_PORT, NP2_A2_PIN); 
		} while (read == 0 );

		/* Clear the EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(NP2_A2_EXTI_LINE);
	}
}

/**********************************
	TIM5 to generate squarewave
 **********************************/
void TIM5_IRQHandler (void) { 

	uint8_t i;
	/* Generate square wave using the update compare interrupt. */
	/* Toggle LED if Timer 5 update compare interrupt has occured */
	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET) {
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
		if (laser_toggle == 1 && laser_tx_full == 21) {
			GPIO_WriteBit(NP2_D1_GPIO_PORT, NP2_D1_PIN, laser_tx_packet[laser_tx_cnt] & 0x01);
			laser_tx_cnt++;
			//debug_printf("laser_tx_cnt = %d\r\n", laser_tx_cnt);
			if (laser_tx_cnt == 44) {
				laser_tx_cnt = 0;
				laser_tx_full = 0;
				for (i = 0; i < 44; i++){
					laser_tx_packet[i]	= 0;
				}
				debug_printf("\r\n");
			}
		}
	}
}

/***************************
	TIM3 to detect edges
 ***************************/
void TIM3_IRQHandler(void) {

	uint8_t i;
	volatile static uint8_t set_toggle = 0;
	volatile static uint8_t laser_check_speed[4];
	uint16_t input_capture_value;

	/* Read input capture value and calculate duty cycle. */
	/* Toggle LED if Timer 3 input capture 2 interrupt has occured */
	if (TIM_GetITStatus(TIM3, TIM_IT_CC2) != RESET) {
		TIM_ClearITPendingBit(TIM3, TIM_IT_CC2);
		input_capture_value = TIM_GetCapture2(TIM3);
		reset_counter++;

		if (allow_found == 1) {
			rf_transmit('F');
			Delay(500);
			rf_transmit('O');
			Delay(500);
			rf_transmit('U');
			Delay(500);
			rf_transmit('N');
			Delay(500);
			rf_transmit('D');
			Delay(500);
			rf_transmit(13);
			Delay(500);
			partner_found++;
			if (partner_found >= 4) {
				allow_found = 0;
				partner_found = 0;
			}
		}

		if (laser_speed_cnt < 4 && input_capture_value <= 52 && input_capture_value >= 10) {
			laser_check_speed[laser_speed_cnt] = input_capture_value;
			//debug_printf("Reading %d = %d \r\n", laser_speed_cnt, laser_check_speed[laser_speed_cnt]);
			laser_speed_cnt++;
			set_toggle = 0;
		}
		else {
			if (set_toggle == 0) {
				if (laser_check_speed[1] >= 23 && laser_check_speed[2] >= 23 && laser_check_speed[3] >= 23
					&& laser_check_speed[1] <= 27 && laser_check_speed[2] <= 27 && laser_check_speed[3] <= 27) {
					debug_printf("1kHz detected!\r\n");
					inputbmc_cap1 = 23;
					inputbmc_cap2 = 27;
					inputbmc_cap3 = 48;
					inputbmc_cap4 = 52;
					set_toggle = 1;
				}
				else if (laser_check_speed[1] >= 10 && laser_check_speed[2] >= 10 && laser_check_speed[3] >= 10 
						&& laser_check_speed[1] <= 14 && laser_check_speed[2] <= 14 && laser_check_speed[3] <= 14) {
					debug_printf("2kHz detected!\r\n");
					inputbmc_cap1 = 10;
					inputbmc_cap2 = 14;
					inputbmc_cap3 = 23;
					inputbmc_cap4 = 27;
					set_toggle = 1;
				}
			}

			if (input_capture_value > inputbmc_cap1 && input_capture_value <= inputbmc_cap2) {
				laser_rx_packet[laser_rx_cnt] = laser_rx_toggle;
				//debug_printf("Capture %d = %d \r\n", laser_rx_cnt, input_capture_value);
				laser_rx_cnt++;
				laser_rx_toggle = !laser_rx_toggle;
			}
			else if (input_capture_value >= inputbmc_cap3 && input_capture_value <= inputbmc_cap4) {
				laser_rx_packet[laser_rx_cnt] = laser_rx_toggle;
				laser_rx_packet[laser_rx_cnt+1] = laser_rx_toggle;
				//debug_printf("Capture %d = %d \r\n", laser_rx_cnt, input_capture_value);
				laser_rx_cnt = laser_rx_cnt+2;
				laser_rx_toggle = !laser_rx_toggle;
			}
		}
		if (laser_rx_cnt >= 38) {
			laser_speed_cnt = 0;
			laser_rx_cnt = 0;
			debug_printf("BMC Decoded MSG = ");
			for (i = 0; i < 38; i++) {
				debug_printf("%d", laser_rx_packet[i]);
			}
			debug_printf("\r\n");
			for (i = 0; i < 4; i++) {
				laser_check_speed[i] = 0;
			}
			laser_receive();
			set_toggle = 0;
			laser_rx_toggle = 1;
			reset_counter = 0;
		}
		if (reset_counter == 1000) {
			laser_speed_cnt = 0;
			laser_rx_cnt = 0;
			for (i = 0; i < 38; i++) {
				laser_rx_packet[i] = 0;
			}
			for (i = 0; i < 4; i++) {
				laser_check_speed[i] = 0;
			}
			set_toggle = 0;
			laser_rx_toggle = 1;
			reset_counter = 0;
		}
		Delay(10000);
	}
}