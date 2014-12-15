/*
 ******************************************************************************
 * @file    proj2.2/main.c 
 * @author  HONG RUI, CHONG
 * @date    20/05/2014
 * @brief   Project 2.2 main program
 ******************************************************************************
 */ 

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Task Priorities -----------------------------------------------------------*/
#define DISTANCE_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define CLI_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define RADIO_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define ADC_PRIORITY			( tskIDLE_PRIORITY + 2 )
#define ACCE_PRIORITY			( tskIDLE_PRIORITY + 2 )
/* Task Stack Allocations ----------------------------------------------------*/
#define DISTANCE_TASK_STACK_SIZE	( configMINIMAL_STACK_SIZE * 4 )
#define CLI_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 6 )
#define RADIO_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 4 )
#define ADC_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 2 )
#define ACCE_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void Hardware_init(void);
void Hardware_PWM_init(void);
void Hardware_SPI_init(void);
void Hardware_ADC_init(void);
void Hardware_I2C_Init(void);
void Hardware_LED_init(void);

/* Command line structure variables ------------------------------------------*/
static const xCommandLineInput xTaskUsage = {
		( char * ) "task_usage",
		( char * ) "Taskusage: TaskUsage output.\r\n",
		prvTaskUsageCommand,
		0
};
static const xCommandLineInput xEncode = {
		( char * ) "encode",
		( char * ) "Encode: Encode input.\r\n",
		prvEncodeCommand,
		1
};
static const xCommandLineInput xDecode = {
		( char * ) "decode",
		( char * ) "Decode: Decode input.\r\n",
		prvDecodeCommand,
		1
};
static const xCommandLineInput xLaser = {
		( char * ) "laser",
		( char * ) "Laser: Turn laser on/off\r\n",
		prvLaserCommand,
		1
};
static const xCommandLineInput xAccelerometer = {
		( char * ) "acce",
		( char * ) "Accelerometer: Turn Accelerometer on/off\r\n",
		prvAccelerometerCommand,
		1
};
static const xCommandLineInput xJoystick = {
		( char * ) "joystick",
		( char * ) "Joystick: Turn Joystick on/off\r\n",
		prvJoystickCommand,
		1
};
static const xCommandLineInput xChannel = {
		( char * ) "channel",
		( char * ) "Channel: Change Channel.\r\n",
		prvChannelCommand,
		1
};
static const xCommandLineInput xGetPassKey = {
		( char * ) "getpasskey",
		( char * ) "GetPassKey: Isn't it obvious?\r\n",
		prvGetPassKeyCommand,
		0
};
static const xCommandLineInput xForward = {
		( char * ) "forward",
		( char * ) "Forward: Move rover forward.\r\n",
		prvForwardCommand,
		2
};
static const xCommandLineInput xReverse = {
		( char * ) "reverse",
		( char * ) "Reverse: Move rover Backwards.\r\n",
		prvReverseCommand,
		2
};
static const xCommandLineInput xRotate = {
		( char * ) "rotate",
		( char * ) "Rotate: Rotate rover Left/Right.\r\n",
		prvRotateCommand,
		3
};
static const xCommandLineInput xAngle = {
		( char * ) "angle",
		( char * ) "Angle: Rotate rover clockwise to an angle.\r\n",
		prvAngleCommand,
		1
};
static const xCommandLineInput xBoundary = {
		( char * ) "boundary",
		( char * ) "Boundary: Move rover until it hits a boundary.\r\n",
		prvBoundaryCommand,
		0
};
static const xCommandLineInput xWaypoint = {
		( char * ) "waypoint",
		( char * ) "Waypoint: Move rover to a certain waypoint.\r\n",
		prvWaypointCommand,
		1
};
static const xCommandLineInput xGetSensor = {
		( char * ) "getsensor",
		( char * ) "Getsensor: Get sensor values.\r\n",
		prvGetSensorCommand,
		0
};
static const xCommandLineInput xGetTime = {
		( char * ) "gettime",
		( char * ) "Gettime: Get time that scheduler been on.\r\n",
		prvGetTimeCommand,
		0
};
static const xCommandLineInput xGetDistance = {
		( char * ) "getdistance",
		( char * ) "Getdistance: Get Distance from rover.\r\n",
		prvGetDistanceCommand,
		0
};


/***************************************************************************************************************************
 ****************************************************************************************************************************
														MAIN PROGRAM														
 ****************************************************************************************************************************
 ***************************************************************************************************************************/
int main(void) {

	NP2_boardinit();
	nrf24l01plus_init();
	Hardware_init();
	Hardware_SPI_init();
	Hardware_ADC_init();
	Hardware_I2C_Init();
	Hardware_LED_init();
	Hardware_PWM_init();
	ADC_SoftwareStartConv(ADC1);	//Perform ADC1 conversions
	ADC_SoftwareStartConv(ADC2);	//Perform ADC2 conversions
	nrf24l01plus_mode_rx();

	/* Initilaize the LwIP stack */
	LwIP_Init();

	gettimeflag = 0;

	RadioTXQueue = xQueueCreate(5, 32);
	RadioWPQueue = xQueueCreate(5, 1);

	vSemaphoreCreateBinary(boundarysem);
	vSemaphoreCreateBinary(waypointsem);
	vSemaphoreCreateBinary(sensorsem);
	vSemaphoreCreateBinary(distsem);
	xSemaphoreTake(boundarysem, 10);
	xSemaphoreTake(waypointsem, 10);
	xSemaphoreTake(sensorsem, 10);
	xSemaphoreTake(distsem, 10);

	/* Create all tasks */
	xTaskCreate( (void *) &DistanceTask, (const signed char *) "Dist", DISTANCE_TASK_STACK_SIZE, NULL, DISTANCE_PRIORITY, &xDistanceTask);
	xTaskCreate( (void *) &CLITask, (const signed char *) "CLI", CLI_TASK_STACK_SIZE, NULL, CLI_PRIORITY, &xCLITask);
	xTaskCreate( (void *) &RadioTask, (const signed char *) "RADIO", RADIO_TASK_STACK_SIZE, NULL, RADIO_PRIORITY, &xRadioTask);
	xTaskCreate( (void *) &ADCTask, (const signed char *) "ADC", ADC_TASK_STACK_SIZE, NULL, ADC_PRIORITY, &xADCTask);
	xTaskCreate( (void *) &ACCETask, (const signed char *) "ACCE", ACCE_TASK_STACK_SIZE, NULL, ACCE_PRIORITY, &xACCETask);

	xCoRoutineCreate(TimerCoroutine, 0, 0 );

	/* Register all CMDs */
	FreeRTOS_CLIRegisterCommand(&xTaskUsage);
	FreeRTOS_CLIRegisterCommand(&xEncode);
	FreeRTOS_CLIRegisterCommand(&xDecode);
	FreeRTOS_CLIRegisterCommand(&xLaser);
	FreeRTOS_CLIRegisterCommand(&xAccelerometer);
	FreeRTOS_CLIRegisterCommand(&xJoystick);
	FreeRTOS_CLIRegisterCommand(&xChannel);
	FreeRTOS_CLIRegisterCommand(&xGetPassKey);
	FreeRTOS_CLIRegisterCommand(&xForward);
	FreeRTOS_CLIRegisterCommand(&xReverse);
	FreeRTOS_CLIRegisterCommand(&xRotate);
	FreeRTOS_CLIRegisterCommand(&xAngle);
	FreeRTOS_CLIRegisterCommand(&xBoundary);
	FreeRTOS_CLIRegisterCommand(&xWaypoint);
	FreeRTOS_CLIRegisterCommand(&xGetSensor);
	FreeRTOS_CLIRegisterCommand(&xGetTime);
	FreeRTOS_CLIRegisterCommand(&xGetDistance);

	/* Start scheduler */
	vTaskStartScheduler();

	return 0;
}


/***********************************************************************************************************************
 ************************************************************************************************************************
											HARDWARE INTIALISATION FUNCTIONS										
 ************************************************************************************************************************
 ***********************************************************************************************************************/

void Hardware_init( void ) {
	portDISABLE_INTERRUPTS();

	NP2_LEDInit();		//Initialise Blue LED
	NP2_LEDOff();		//Turn off Blue LED

	portENABLE_INTERRUPTS();
}

void Hardware_SPI_init(void) {

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

	/* Write RF_CH */
	nrf24l01plus_WriteRegister(0x25, 50);
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

/***********************************
	Hardware Init I2C for Accele
 ***********************************/
void Hardware_I2C_Init() {

	GPIO_InitTypeDef GPIO_i2c;	

	//Initialise I2C
	RCC_APB1PeriphClockCmd(NP2_I2C_CLK, ENABLE);

	/* Configure the GPIO I2C1 SDA/SCL pins */ 
	GPIO_i2c.GPIO_Pin = NP2_SCL_PIN | NP2_SDA_PIN;
	GPIO_i2c.GPIO_Mode = GPIO_Mode_AF;
	GPIO_i2c.GPIO_OType = GPIO_OType_OD;
	GPIO_i2c.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_i2c.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(NP2_SDA_GPIO_PORT, &GPIO_i2c);

	GPIO_PinAFConfig(NP2_SDA_GPIO_PORT, NP2_SDA_PINSOURCE, NP2_SDA_AF);
	GPIO_PinAFConfig(NP2_SCL_GPIO_PORT, NP2_SCL_PINSOURCE, NP2_SCL_AF);

	I2C_DeInit(I2C1);

	NP2_i2cInitStruct.I2C_ClockSpeed = 5000;          /*!< Specifies the clock frequency. This parameter must be set to a value lower than 400kHz */
	NP2_i2cInitStruct.I2C_Mode = I2C_Mode_I2C;                /*!< Specifies the I2C mode. This parameter can be a value of @ref I2C_mode */
	NP2_i2cInitStruct.I2C_DutyCycle = I2C_DutyCycle_2;           /*!< Specifies the I2C fast mode duty cycle. This parameter can be a value of @ref I2C_duty_cycle_in_fast_mode */
	NP2_i2cInitStruct.I2C_OwnAddress1 = 0;         /*!< Specifies the first device own address. This parameter can be a 7-bit or 10-bit address. */
	NP2_i2cInitStruct.I2C_Ack = I2C_Ack_Enable;                 /*!< Enables or disables the acknowledgement. This parameter can be a value of @ref I2C_acknowledgement */
	NP2_i2cInitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; /*!< Specifies if 7-bit or 10-bit address is acknowledged. This parameter can be a value of @ref I2C_acknowledged_address */

	I2C_Init(I2C1, &NP2_i2cInitStruct);

	//Register and Init Accelerometer
	I2C_Cmd(I2C1, ENABLE);

	I2C_GenerateSTART(I2C1, ENABLE);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1, 0x3A, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C1, 0x2A);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_SendData(I2C1, 0x01);
	while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTOP(I2C1, ENABLE);
}

/***********************************
	Hardware Init GPIO for LED
 ***********************************/
void Hardware_LED_init(void) {

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
}

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
	RCC_AHB1PeriphClockCmd(NP2_D1_GPIO_CLK, ENABLE);
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

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = NP2_D1_PIN;
	GPIO_Init(NP2_D1_GPIO_PORT, &GPIO_InitStructure);

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

/***********************************************************************************************************************
 ************************************************************************************************************************
											OTHER TASK FUNCTIONS											
 ************************************************************************************************************************
 ***********************************************************************************************************************/

/*******************************
	SPI Sendbyte to Transmit
 *******************************/
extern uint8_t spi_sendbyte(uint8_t sendbyte) {

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

extern void vApplicationIdleHook( void ) {

	vCoRoutineSchedule();
	
}

extern void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName ) {
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	NP2_LEDOff();
	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}

/***************************
	Hamming byte decoder
 ***************************/
extern uint8_t hamming_byte_decoder(uint8_t input_up, uint8_t input_low) {

	uint8_t i;
	volatile static uint8_t up[8], low[8];
	volatile static uint8_t new_upbyte = 0, new_lowbyte = 0;
	volatile static uint16_t new_16 = 0, old_16 = 0;
	volatile uint8_t decodedbyte = 0;
	uint16_t errormask, errorcount = 0;

	for (i = 0; i < 8; i++){
		up[i] = 0;
		low[i] = 0;
	}

	old_16 = (input_up << 8) | input_low;
	new_upbyte = hamming_hbyte_decoder(input_up);
	new_lowbyte = hamming_hbyte_decoder(input_low);
	new_16 = (new_upbyte << 8) | new_lowbyte;
	debug_printf("old %04x, new %04x\r\n", old_16, new_16);
	errormask = new_16 ^ old_16;
	for (i = 0; i < 16; i++) {
		errorcount =  ((errormask>>i)&1) + errorcount;
	}
	debug_printf("errorcount: %d \r\n", errorcount);

	up[7] = !!(new_upbyte & 0x1);		//p0
	up[6] = !!(new_upbyte & 0x2);		//h0
	up[5] = !!(new_upbyte & 0x4);		//h1
	up[4] = !!(new_upbyte & 0x8);		//h2
	up[3] = !!(new_upbyte & 0x10);		//d0
	up[2] = !!(new_upbyte & 0x20);		//d1
	up[1] = !!(new_upbyte & 0x40);		//d2
	up[0] = !!(new_upbyte & 0x80);		//d3
	low[7] = !!(new_lowbyte & 0x1);		//p0
	low[6] = !!(new_lowbyte & 0x2);		//h0
	low[5] = !!(new_lowbyte & 0x4);		//h1
	low[4] = !!(new_lowbyte & 0x8);		//h2
	low[3] = !!(new_lowbyte & 0x10);	//d0
	low[2] = !!(new_lowbyte & 0x20);	//d1
	low[1] = !!(new_lowbyte & 0x40);	//d2
	low[0] = !!(new_lowbyte & 0x80);	//d3

	decodedbyte = (up[0] << 7) | (up[1] << 6) | (up[2] << 5) | (up[3] << 4) | (low[0] << 3) | (low[1] << 2) | (low[2] << 1) | low[3];

	return decodedbyte;
}

/****************************
	Hamming hbyte decoder
 ****************************/
extern uint8_t hamming_hbyte_decoder(uint8_t input) {

	uint8_t ex[8];
	uint8_t syn1, syn2, syn3, syndrome, pcheck;
	uint8_t syndrome_array[] = {111, 110, 101, 11, 1, 10, 100};
	uint8_t i;
	uint8_t out;

	/* extract bits */
	ex[7] = !!(input & 0x1);		//p0
	ex[6] = !!(input & 0x2);		//h0
	ex[5] = !!(input & 0x4);		//h1
	ex[4] = !!(input & 0x8);		//h2
	ex[3] = !!(input & 0x10);		//d0
	ex[2] = !!(input & 0x20);		//d1
	ex[1] = !!(input & 0x40);		//d2
	ex[0] = !!(input & 0x80);		//d3

	syn1 = ex[0] ^ ex[1] ^ ex[2] ^ ex[6];
	syn2 = ex[0] ^ ex[1] ^ ex[3] ^ ex[5];
	syn3 = ex[0] ^ ex[2] ^ ex[3] ^ ex[4];
	syndrome = (syn1 * 100) + (syn2 * 10) + syn3;

	if (syndrome != 000) {
		for (i = 0; i < 7; i++) {
			if (syndrome == syndrome_array[i]) {
				debug_printf("old: %d, ", ex[i]);
				ex[i] = !ex[i];
				debug_printf("Syndrome %d: %03d, new: %d\r\n", i, syndrome_array[i], ex[i]);
				i++;
			}
		}
	}

	pcheck = ex[0] ^ ex[1] ^ ex[2] ^ ex[3] ^ ex[4] ^ ex[5] ^ ex[6];

	if (syndrome == 000 && pcheck != ex[7]) {
		//debug_printf("ori = %d, pcheck = %d\r\n", ex[7], pcheck);
		ex[7] = !ex[7];
	}
	out = (ex[0] << 7) | (ex[1] << 6) | (ex[2] << 5) | (ex[3] << 4) | (ex[4] << 3) | (ex[5] << 2) | (ex[6] << 1) | ex[7];

	return out;

}

/***************************
	Hamming byte encoder
 ***************************/
extern uint16_t hamming_byte_encoder(uint8_t input) {

	uint16_t out;

	/* first encode D0..D3 (first 4 bits), 
	 * then D4..D7 (second 4 bits).
	 */
	out = hamming_hbyte_encoder(input & 0xF) | (hamming_hbyte_encoder(input >> 4) << 8);
	debug_printf("out %04x\r\n", out);

	return(out);
}

/****************************
	Hamming hbyte encoder
 ****************************/
extern uint8_t hamming_hbyte_encoder(uint8_t input) {

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