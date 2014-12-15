/*
 ******************************************************************************
 * @file    proj2.2/main.h 
 * @author  HONG RUI, CHONG
 * @date    20/05/2014
 * @brief   Project 2.2 main program header file
 ******************************************************************************
 */ 

#ifndef __MAINH_H
#define __MAINH_H

#include "netduinoplus2.h"
#include "stm32f4xx_conf.h"
#include "debug_printf.h"
#include "nrf24l01plus.h"
#include <string.h>
#include <stdio.h>
#include "stdlib.h"
#include "math.h"
#include "netconf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "croutine.h"
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

/* Private variables ---------------------------------------------------------*/
I2C_InitTypeDef	NP2_i2cInitStruct;
xQueueHandle RadioTXQueue;	//queue used to for transmiting through nrf
xQueueHandle RadioWPQueue;	//queue used to transmit Waypoint
xTaskHandle xCLITask;		//task handler for CLI
xTaskHandle xDistanceTask;	//task handler for Distance
xTaskHandle xRadioTask;		//task handler for Radio
xTaskHandle xADCTask;		//task handler for Joystick ADC
xTaskHandle xACCETask;		//task handler for Accelerometer
xSemaphoreHandle boundarysem;	//semaphore for boundary
xSemaphoreHandle waypointsem;	//semaphore for waypoint
xSemaphoreHandle sensorsem;		//semaphore to print sensor
xSemaphoreHandle distsem;		//semaphore to print distance
uint8_t gettimeflag;	//flag to print time
uint8_t min, sec;		//min, sec and ms values.
uint16_t ms;
/* Command line function prototypes ------------------------------------------*/
extern portBASE_TYPE prvDecodeCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvEncodeCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvTaskUsageCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvLaserCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvAccelerometerCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvJoystickCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvChannelCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvGetPassKeyCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvForwardCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvReverseCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvRotateCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvAngleCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvBoundaryCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvWaypointCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvGetSensorCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvGetTimeCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );
extern portBASE_TYPE prvGetDistanceCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString );

/* Task function prototypes ------------------------------------------*/
extern void DistanceTask(void * pvParameters);
extern void RadioTask(void * pvParameters);
extern void CLITask(void * pvParameters);
extern void ADCTask(void * pvParameters);
extern void ACCETask(void * pvParameters);
extern void ApplicationIdleHook( void );

/* Co-Routine function prototypes ------------------------------------------*/
extern void TimerCoroutine( xCoRoutineHandle xTimer, unsigned portBASE_TYPE uxIndex );

/* Other function prototypes ------------------------------------------*/
extern void Delay(__IO unsigned long nCount);
extern uint8_t spi_sendbyte(uint8_t sendbyte);
extern uint8_t hamming_byte_decoder(uint8_t input1, uint8_t input2);
extern uint8_t hamming_hbyte_decoder(uint8_t input);
extern uint16_t hamming_byte_encoder(uint8_t input);
extern uint8_t hamming_hbyte_encoder(uint8_t input);
extern int rescale(float AdcValue, const float maxValue, const int newLow, const int newHigh);
extern void ledtask(uint8_t input);

#endif /* FREERTOS_CONFIG_H */