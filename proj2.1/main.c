/*
 ******************************************************************************
 * @file    proj2.1/main.c 
 * @author  HONG RUI, CHONG
 * @date    20/05/2014
 * @brief   Project 2.1 main program
 ******************************************************************************
 */ 

/* Includes ------------------------------------------------------------------*/
#include "netduinoplus2.h"
#include "stm32f4xx_conf.h"
#include "debug_printf.h"
#include <string.h>
#include <stdio.h>
#include "stdlib.h"
#include "math.h"
#include "netconf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "FreeRTOS_CLI.h"
#include "usbd_cdc_vcp.h"
#include "tuioclient.h"
#include "tcpip.h"
#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"
#include "main.h"

/* Private define ------------------------------------------------------------*/
#define TCP_SOCKET_PORT	10
/* Task Priorities -----------------------------------------------------------*/
#define PANTILTTASK_PRIORITY	( tskIDLE_PRIORITY + 4 )
#define DISTANCETASK_PRIORITY	( tskIDLE_PRIORITY + 2 )
#define CLI_PRIORITY			( tskIDLE_PRIORITY + 4 )
/* Task Stack Allocations ----------------------------------------------------*/
#define PANTILTTASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define DISTANCETASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )
#define CLI_TASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 2 )
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
xQueueHandle DistanceQueue;    /* Queue used */
xTaskHandle xPanTiltTask;
xTaskHandle xCLITask;
xTaskHandle xDistanceTask;
/* Private function prototypes -----------------------------------------------*/
void Hardware_init();
void Hardware_PWM_init();
void ApplicationIdleHook( void );	/* The idle hook is just used to stream data to the USB port.*/
/* Command line function prototypes ------------------------------------------*/
static portBASE_TYPE prvTaskUsageCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static portBASE_TYPE prvLaserCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static portBASE_TYPE prvDistanceCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
static portBASE_TYPE prvTrackingCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
/* Command line structure variables ------------------------------------------*/
xCommandLineInput xTaskUsage = {
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
static const xCommandLineInput xDistance = {
		( char * ) "distance",
		( char * ) "Distance: Turn Distance Estimation on/off\r\n",
		prvDistanceCommand,
		1
};
static const xCommandLineInput xTracking = {
		( char * ) "tracking",
		( char * ) "Tracking: Turn Tracking on/off\r\n",
		prvTrackingCommand,
		1
};


/***************************************************************************************************************************
 ****************************************************************************************************************************
														MAIN PROGRAM														
 ****************************************************************************************************************************
 ***************************************************************************************************************************/
int main(void) {

	NP2_boardinit();
	Hardware_init();
	Hardware_PWM_init();

	/* Initilaize the LwIP stack */
	LwIP_Init();

	/* Create all tasks */
	xTaskCreate( (void *) &PanTiltTask, (const signed char *) "PanTilt", PANTILTTASK_STACK_SIZE, NULL, PANTILTTASK_PRIORITY, &xPanTiltTask);
	xTaskCreate( (void *) &DistanceTask, (const signed char *) "Dist", DISTANCETASK_STACK_SIZE, NULL, DISTANCETASK_PRIORITY, &xDistanceTask);
	xTaskCreate( (void *) &CLITask, (const signed char *) "CLI", CLI_TASK_STACK_SIZE, NULL, CLI_PRIORITY, &xCLITask);

	/* Register all CMDs */
	FreeRTOS_CLIRegisterCommand(&xTaskUsage);
	FreeRTOS_CLIRegisterCommand(&xEncode);
	FreeRTOS_CLIRegisterCommand(&xDecode);
	FreeRTOS_CLIRegisterCommand(&xLaser);
	FreeRTOS_CLIRegisterCommand(&xDistance);
	FreeRTOS_CLIRegisterCommand(&xTracking);

	/* Start scheduler */
	vTaskStartScheduler();

	return 0;
}


/***********************************************************************************************************************
 ************************************************************************************************************************
											CLI COMMAND FUNCTIONS									
 ************************************************************************************************************************
 ***********************************************************************************************************************/

/***********************************
			TASK USAGE CMD
 ***********************************/
static portBASE_TYPE prvTaskUsageCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	static portCHAR buffer[400];
	// get current running tasks
	vTaskList((signed char *)(buffer));
	debug_printf("NAME          STATE  PRIORITY  STACK   NUM\r\n");
	debug_printf("*********************************************\r\n");
	debug_printf("%s", buffer);
	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n\r");
	return pdFALSE;
}

/***********************************
			LASER CMD
 ***********************************/
static portBASE_TYPE prvLaserCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len;
	const char *cCmd_string;

	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	// check and turn laser on/off
	if (strcmp(cCmd_string, "on") == 0 || strcmp(cCmd_string, "On") == 0 || strcmp(cCmd_string, "ON") == 0) {
		GPIO_WriteBit(NP2_D1_GPIO_PORT, NP2_D1_PIN, 0x01);
		debug_printf("Laser On!\n\r");
	}
	else if (strcmp(cCmd_string, "off") == 0 || strcmp(cCmd_string, "Off") == 0 || strcmp(cCmd_string, "OFF") == 0) {
		GPIO_WriteBit(NP2_D1_GPIO_PORT, NP2_D1_PIN, 0x00);
		debug_printf("Laser Off\n\r");
	}
	else {
		debug_printf("Invalid argument. Use only on/On/ON or off/Off/OFF.\n\r");
	}

	xWriteBufferLen = sprintf(pcWriteBuffer, "\n\r");

	return pdFALSE;
}

/***********************************
			DISTANCE CMD
 ***********************************/
static portBASE_TYPE prvDistanceCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len;
	const char *cCmd_string;

	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	// check and turn distance estimation task on/off
	if (strcmp(cCmd_string, "on") == 0 || strcmp(cCmd_string, "On") == 0 || strcmp(cCmd_string, "ON") == 0) {
		vTaskResume(xDistanceTask);
		debug_printf("Distance Estimation On!\n\r");
	}
	else if (strcmp(cCmd_string, "off") == 0 || strcmp(cCmd_string, "Off") == 0 || strcmp(cCmd_string, "OFF") == 0) {
		vTaskSuspend(xDistanceTask);
		debug_printf("Distance Estimation Off!\n\r");
	}
	else {
		debug_printf("Invalid argument. Use only on/On/ON or off/Off/OFF.\n\r");
	}

	xWriteBufferLen = sprintf(pcWriteBuffer, "\n\r");

	return pdFALSE;
}

/***********************************
			TRACKING CMD
 ***********************************/
static portBASE_TYPE prvTrackingCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len;
	const char *cCmd_string;

	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	// check and turn servo control task on/off
	if (strcmp(cCmd_string, "on") == 0 || strcmp(cCmd_string, "On") == 0 || strcmp(cCmd_string, "ON") == 0) {
		vTaskResume(xPanTiltTask);
		vTaskResume(xDistanceTask);
		debug_printf("Tracking and Distance Estimation On!\n\r");
	}
	else if (strcmp(cCmd_string, "off") == 0 || strcmp(cCmd_string, "Off") == 0 || strcmp(cCmd_string, "OFF") == 0) {
		vTaskSuspend(xPanTiltTask);
		vTaskSuspend(xDistanceTask);
		debug_printf("Tracking and Distance Estimation Off\n\r");
	}
	else {
		debug_printf("Invalid argument. Use only on/On/ON or off/Off/OFF.\n\r");
	}

	xWriteBufferLen = sprintf(pcWriteBuffer, "\n\r");

	return pdFALSE;
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
											oTHER TASK FUNCTIONS											
 ************************************************************************************************************************
 ***********************************************************************************************************************/

void vApplicationIdleHook( void ) {
	static portTickType xLastTx = 0;

	NP2_LEDOff();
	/* The idle hook simply prints the idle tick count */
	if ((xTaskGetTickCount() - xLastTx ) > (1000 / portTICK_RATE_MS)) {
		xLastTx = xTaskGetTickCount();
		debug_printf("IDLE Tick %d\n", xLastTx);
	}
}


void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName ) {
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	NP2_LEDOff();
	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}
