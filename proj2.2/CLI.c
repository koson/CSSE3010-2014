/*
 ******************************************************************************
 * @file    proj2.1/main.c
 * @author  HONG RUI, CHONG
 * @date    20/05/2014
 * @brief   Project 2.1 main program
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/***********************************************************************************************************************
 ************************************************************************************************************************
											CLI COMMAND FUNCTIONS									
 ************************************************************************************************************************
 ***********************************************************************************************************************/
/***********************************
			TASK USAGE CMD
 ***********************************/
extern portBASE_TYPE prvTaskUsageCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

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
extern portBASE_TYPE prvLaserCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

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

	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n\r");

	return pdFALSE;
}

/***********************************
			ACCELEROMETER CMD
 ***********************************/
extern portBASE_TYPE prvAccelerometerCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len;
	const char *cCmd_string;

	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	// check and turn distance estimation task on/off
	if (strcmp(cCmd_string, "on") == 0 || strcmp(cCmd_string, "On") == 0 || strcmp(cCmd_string, "ON") == 0) {
		vTaskResume(xACCETask);
		debug_printf("Acceletometer Control On!\n\r");
	}
	else if (strcmp(cCmd_string, "off") == 0 || strcmp(cCmd_string, "Off") == 0 || strcmp(cCmd_string, "OFF") == 0) {
		vTaskSuspend(xACCETask);
		debug_printf("Acceletometer Control Off!\n\r");
	}
	else {
		debug_printf("Invalid argument. Use only on/On/ON or off/Off/OFF.\n\r");
	}

	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n\r");

	return pdFALSE;
}

/***********************************
			JOYSTICK CMD
 ***********************************/
extern portBASE_TYPE prvJoystickCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len;
	const char *cCmd_string;

	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	// check and turn servo control task on/off
	if (strcmp(cCmd_string, "on") == 0 || strcmp(cCmd_string, "On") == 0 || strcmp(cCmd_string, "ON") == 0) {
		vTaskResume(xADCTask);
		debug_printf("Joystick Control On!\n\r");
	}
	else if (strcmp(cCmd_string, "off") == 0 || strcmp(cCmd_string, "Off") == 0 || strcmp(cCmd_string, "OFF") == 0) {
		vTaskSuspend(xADCTask);
		debug_printf("Joystick Control Off\n\r");
	}
	else {
		debug_printf("Invalid argument. Use only on/On/ON or off/Off/OFF.\n\r");
	}

	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n\r");

	return pdFALSE;
}

/***********************************
			CHANNEL CMD
 ***********************************/
extern portBASE_TYPE prvChannelCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	long lParam_len;
	const char *cCmd_string;
	uint8_t channel;

	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);
	channel = strtol (cCmd_string, NULL, 0);
	nrf24l01plus_WriteRegister(0x25, channel);
	vTaskDelay(10);
	debug_printf("Channel Changed to %d.\r\n", channel);

	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n\r");

	return pdFALSE;
}

/***********************************
			GETPASSKEY CMD
 ***********************************/
extern portBASE_TYPE prvGetPassKeyCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	const static uint8_t TX_PasskeyBuffer[] = {0x30, 0x78, 0x56, 0x34, 0x12, 0x28, 0x92, 0x96, 0x42, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	if (RadioTXQueue != NULL) {
		/*Send message to the front of the queue - wait atmost 0 ticks */
		xQueueReset(RadioTXQueue);
		vTaskDelay(10);
		if( xQueueSendToFront(RadioTXQueue, ( void * ) TX_PasskeyBuffer, ( portTickType ) 0 ) != pdPASS ) {
			debug_printf("Unable to send RadioTXQueue\r\n");
		}
	}

	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n\r");

	return pdFALSE;
}

/***********************************
			FORWARD CMD
 ***********************************/
extern portBASE_TYPE prvForwardCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	uint8_t TX_ForwardBuffer[] = {0x32, 0x78, 0x56, 0x34, 0x12, 0x28, 0x92, 0x96, 0x42, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	long lParam_len;
	const char *cCmd_string;
	char * pEnd;
	uint8_t speed = 0;
	uint8_t duration = 0;


	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	speed = strtol (cCmd_string, &pEnd, 0);
	duration = strtol (pEnd, NULL, 0);


	if (speed > 100) {
		speed = 100;
		debug_printf("Set to Max Speed 100\r\n");
	}
	if (duration > 15) {
		duration = 15;
		debug_printf("Set to Max Duration 15\r\n");
	}

	TX_ForwardBuffer[11] = hamming_hbyte_encoder(speed & 0x0F);
	TX_ForwardBuffer[12] = hamming_hbyte_encoder(speed >> 4);
	TX_ForwardBuffer[13] = hamming_hbyte_encoder(speed & 0x0F);
	TX_ForwardBuffer[14] = hamming_hbyte_encoder(speed >> 4);
	TX_ForwardBuffer[15] = hamming_hbyte_encoder(5);
	TX_ForwardBuffer[16] = hamming_hbyte_encoder(duration);

	if (RadioTXQueue != NULL) {
		/*Send message to the front of the queue - wait atmost 0 ticks */
		if( xQueueSendToFront(RadioTXQueue, ( void * ) TX_ForwardBuffer, ( portTickType ) 0 ) != pdPASS ) {
			debug_printf("Unable to send RadioTXQueue\r\n");
		}
	}

	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n\r");

	return pdFALSE;
}

/***********************************
			REVERSE CMD
 ***********************************/
extern portBASE_TYPE prvReverseCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	uint8_t TX_ReverseBuffer[] = {0x32, 0x78, 0x56, 0x34, 0x12, 0x28, 0x92, 0x96, 0x42, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	long lParam_len;
	const char *cCmd_string;
	char * pEnd;
	uint8_t speed = 0;
	uint8_t duration = 0;


	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	speed = strtol (cCmd_string, &pEnd, 0);
	duration = strtol (pEnd, NULL, 0);

	if (speed > 100) {
		speed = 100;
		debug_printf("Set to Max Speed 100\r\n");
	}
	if (duration > 15) {
		duration = 15;
		debug_printf("Set to Max Duration 15\r\n");
	}

	TX_ReverseBuffer[11] = hamming_hbyte_encoder(speed & 0x0F);
	TX_ReverseBuffer[12] = hamming_hbyte_encoder(speed >> 4);
	TX_ReverseBuffer[13] = hamming_hbyte_encoder(speed & 0x0F);
	TX_ReverseBuffer[14] = hamming_hbyte_encoder(speed >> 4);
	TX_ReverseBuffer[15] = hamming_hbyte_encoder(10);
	TX_ReverseBuffer[16] = hamming_hbyte_encoder(duration);

	if (RadioTXQueue != NULL) {
		/*Send message to the front of the queue - wait atmost 0 ticks */
		if( xQueueSendToFront(RadioTXQueue, ( void * ) TX_ReverseBuffer, ( portTickType ) 0 ) != pdPASS ) {
			debug_printf("Unable to send RadioTXQueue\r\n");
		}
	}

	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n\r");

	return pdFALSE;
}

/***********************************
			ROTATE CMD
 ***********************************/
extern portBASE_TYPE prvRotateCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	uint8_t TX_RotateBuffer[] = {0x32, 0x78, 0x56, 0x34, 0x12, 0x28, 0x92, 0x96, 0x42, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	long lParam_len;
	const char *cCmd_string;
	char * string[3];
	uint8_t speed = 0;
	uint8_t duration = 0;

	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);


	string[0] = strtok (cCmd_string," ,.-");
	string[1] = strtok (NULL," ,.-");
	string[2] = strtok (NULL," ,.-");
	debug_printf("%s, %s, %s\r\n", string[0], string[1], string[2]);
	speed = strtol (string[1], NULL, 0);
	duration = strtol (string[2], NULL, 0);

	if (speed > 100) {
		speed = 100;
		debug_printf("Set to Max Speed 100\r\n");
	}
	if (duration > 15) {
		duration = 15;
		debug_printf("Set to Max Duration 15\r\n");
	}

	TX_RotateBuffer[11] = hamming_hbyte_encoder(speed & 0x0F);
	TX_RotateBuffer[12] = hamming_hbyte_encoder(speed >> 4);
	TX_RotateBuffer[13] = hamming_hbyte_encoder(speed & 0x0F);
	TX_RotateBuffer[14] = hamming_hbyte_encoder(speed >> 4);
	TX_RotateBuffer[16] = hamming_hbyte_encoder(duration);

	if (strcmp(string[0], "left") == 0 ) {
		TX_RotateBuffer[15] = hamming_hbyte_encoder(9);
	}
	else if (strcmp(string[0], "right") == 0 ) {
		TX_RotateBuffer[15] = hamming_hbyte_encoder(6);
	}
	else {
		debug_printf("There is only left and right...LEFT...AND...RIGHT...\r\n");
		return;
	}

	if (RadioTXQueue != NULL) {
		/*Send message to the front of the queue - wait atmost 0 ticks */
		if( xQueueSendToFront(RadioTXQueue, ( void * ) TX_RotateBuffer, ( portTickType ) 0 ) != pdPASS ) {
			debug_printf("Unable to send RadioTXQueue\r\n");
		}
	}

	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n\r");

	return pdFALSE;
}

/***********************************
			ANGLE CMD
 ***********************************/
extern portBASE_TYPE prvAngleCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	uint8_t TX_AngleBuffer[] = {0x32, 0x78, 0x56, 0x34, 0x12, 0x28, 0x92, 0x96, 0x42, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	long lParam_len;
	const char *cCmd_string;
	uint16_t angle = 0;
	uint8_t duration = 0;
	uint8_t speed = 20;

	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	angle = strtol (cCmd_string, NULL, 0);
	if (angle >= 360) {
		angle = 360;
	}

	duration = angle/45;

	debug_printf("rotating for %d to get %d degrees\r\n", duration, angle);

	TX_AngleBuffer[11] = hamming_hbyte_encoder(speed & 0x0F);
	TX_AngleBuffer[12] = hamming_hbyte_encoder(speed >> 4);
	TX_AngleBuffer[13] = hamming_hbyte_encoder(speed & 0x0F);
	TX_AngleBuffer[14] = hamming_hbyte_encoder(speed >> 4);
	TX_AngleBuffer[15] = hamming_hbyte_encoder(6);
	TX_AngleBuffer[16] = hamming_hbyte_encoder(duration);

	if (RadioTXQueue != NULL) {
		/*Send message to the front of the queue - wait atmost 0 ticks */
		if( xQueueSendToFront(RadioTXQueue, ( void * ) TX_AngleBuffer, ( portTickType ) 0 ) != pdPASS ) {
			debug_printf("Unable to send TX_AngleBuffer\r\n");
		}
	}

	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n\r");

	return pdFALSE;
}

/***********************************
			BOUNDARY CMD
 ***********************************/
extern portBASE_TYPE prvBoundaryCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	uint8_t TX_BoundaryBuffer[] = {0x32, 0x78, 0x56, 0x34, 0x12, 0x28, 0x92, 0x96, 0x42, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t speed = 30;
	uint8_t duration = 15;

	TX_BoundaryBuffer[11] = hamming_hbyte_encoder(speed & 0x0F);
	TX_BoundaryBuffer[12] = hamming_hbyte_encoder(speed >> 4);
	TX_BoundaryBuffer[13] = hamming_hbyte_encoder(speed & 0x0F);
	TX_BoundaryBuffer[14] = hamming_hbyte_encoder(speed >> 4);
	TX_BoundaryBuffer[15] = hamming_hbyte_encoder(5);
	TX_BoundaryBuffer[16] = hamming_hbyte_encoder(duration);

	if (RadioTXQueue != NULL) {
		/*Send message to the front of the queue - wait atmost 0 ticks */
		if( xQueueSendToFront(RadioTXQueue, ( void * ) TX_BoundaryBuffer, ( portTickType ) 0 ) != pdPASS ) {
			debug_printf("Unable to send RadioTXQueue\r\n");
		}
		else {
			if (boundarysem != NULL) {	/* Check if semaphore exists */
				xSemaphoreGive(boundarysem);
				debug_printf("boundary semaphore given!\r\n");
			}
		}
	}

	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n\r");

	return pdFALSE;
}

/***********************************
			WAYPOINT CMD
 ***********************************/
extern portBASE_TYPE prvWaypointCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	uint8_t TX_WaypointBuffer[] = {0x32, 0x78, 0x56, 0x34, 0x12, 0x28, 0x92, 0x96, 0x42, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t speed = 50;
	uint8_t duration = 15;
	uint8_t waypoint[1] = {0};
	long lParam_len;
	const char *cCmd_string;

	//Get parameters from command string
	cCmd_string = FreeRTOS_CLIGetParameter(pcCommandString, 1, &lParam_len);

	waypoint[0] = strtol (cCmd_string, NULL, 0);

	TX_WaypointBuffer[11] = hamming_hbyte_encoder(speed & 0x0F);
	TX_WaypointBuffer[12] = hamming_hbyte_encoder(speed >> 4);
	TX_WaypointBuffer[13] = hamming_hbyte_encoder(speed & 0x0F);
	TX_WaypointBuffer[14] = hamming_hbyte_encoder(speed >> 4);
	TX_WaypointBuffer[15] = hamming_hbyte_encoder(5);
	TX_WaypointBuffer[16] = hamming_hbyte_encoder(duration);

	if (RadioWPQueue != NULL) {
		/*Send message to the front of the queue - wait atmost 0 ticks */
		if( xQueueSendToFront(RadioWPQueue, ( void * ) waypoint, ( portTickType ) 0 ) != pdPASS ) {
			debug_printf("Unable to send RadioWPQueue\r\n");
		}
	}

	if (RadioTXQueue != NULL) {
		/*Send message to the front of the queue - wait atmost 0 ticks */
		if( xQueueSendToFront(RadioTXQueue, ( void * ) TX_WaypointBuffer, ( portTickType ) 0 ) != pdPASS ) {
			debug_printf("Unable to send RadioTXQueue\r\n");
		}
		else {
			if (waypointsem != NULL) {	/* Check if semaphore exists */
				xSemaphoreGive(waypointsem);
				debug_printf("waypoint semaphore given!\r\n");
			}
		}
	}

	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n\r");

	return pdFALSE;
}

/***********************************
			GETSENSOR CMD
 ***********************************/
extern portBASE_TYPE prvGetSensorCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	uint8_t TX_SensorBuffer[] = {0x31, 0x78, 0x56, 0x34, 0x12, 0x28, 0x92, 0x96, 0x42, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	if (RadioTXQueue != NULL) {
		/*Send message to the front of the queue - wait atmost 0 ticks */
		if( xQueueSendToFront(RadioTXQueue, ( void * ) TX_SensorBuffer, ( portTickType ) 0 ) != pdPASS ) {
			debug_printf("Unable to send RadioTXQueue\r\n");
		}
		else {
			if (sensorsem != NULL) {	/* Check if semaphore exists */
				xSemaphoreGive(sensorsem);
			}
		}
	}

	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n\r");

	return pdFALSE;
}

/***********************************
			GETTIME CMD
 ***********************************/
extern portBASE_TYPE prvGetTimeCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	gettimeflag = 1;

	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n\r");

	return pdFALSE;
}

/***********************************
			GETSENSOR CMD
 ***********************************/
extern portBASE_TYPE prvGetDistanceCommand(char *pcWriteBuffer, size_t xWriteBufferLen, const char *pcCommandString ) {

	if (distsem != NULL) {	/* Check if semaphore exists */
		xSemaphoreGive(distsem);
	}

	xWriteBufferLen = sprintf((char *) pcWriteBuffer, "\n\r");

	return pdFALSE;
}