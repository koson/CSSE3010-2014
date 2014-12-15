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

/* Private typedef -----------------------------------------------------------*/
//structure for distance estimation
typedef struct {
	int opposite;
	int adjecent;
	int hypotenuse;
} distanceestimation;

/* Private variables ---------------------------------------------------------*/
xQueueHandle DistanceQueue;    /* Queue used */

/***********************************************************************************************************************
 ************************************************************************************************************************
											RTOS TASK FUNCTIONS
 ************************************************************************************************************************
 ***********************************************************************************************************************/

/***********************************
				CLI TASK
 ***********************************/
extern void CLITask(void * pvParameters) {

	char cRxedChar;
	char cInputString[100];
	int InputIndex = 0;
	char *pcOutputString;
	portBASE_TYPE xReturned;

	/* Initialise pointer to CLI output buffer. */
	memset(cInputString, 0, sizeof(cInputString));
	pcOutputString = FreeRTOS_CLIGetOutputBuffer();

	for (;;) {

		/* Receive character from USB receive */
		if (VCP_getchar((uint8_t *) &cRxedChar)) {
			if ((cRxedChar != 0) && (cRxedChar != 5)) {

				/* Process only if return is received. */
				if (cRxedChar == '\r') {

					debug_printf("\n");

					/* Put null character in command input string. */
					cInputString[InputIndex] = '\0';


					xReturned = pdTRUE;
					/* Process command input string. */
					while (xReturned != pdFALSE) {

						/* Returns pdFALSE, when all strings have been returned */
						xReturned = FreeRTOS_CLIProcessCommand( cInputString, pcOutputString, configCOMMAND_INT_MAX_OUTPUT_SIZE );

						/* Display CLI output string */
						debug_printf("%s\n\r",pcOutputString);
					}
					memset(cInputString, 0, sizeof(cInputString));
					InputIndex = 0;

				} else {
					if( cRxedChar == 127 ) {

						/* Backspace was pressed.  Erase the last character in the string - if any.*/
						if( InputIndex > 0 ) {
							InputIndex--;
							cInputString[ InputIndex ] = '\0';
							debug_printf("\b ");
						}
					} else {

						/* A character was entered.  Add it to the string
					   entered so far.  When a \n is entered the complete
					   string will be passed to the command interpreter. */
						if( InputIndex < 20 ) {
							cInputString[ InputIndex ] = cRxedChar;
							InputIndex++;
						}
					}
				}
				debug_printf("\r%s", cInputString);
			}
		}

		vTaskDelay(50);
	}
}

/***********************************
			PAN/TILT TASK
 ***********************************/
extern void PanTiltTask(void * pvParameters) {

	// Message struct for first marker
	static tuiomessage_t marker1;
	static tuiomessage_t *pmarker1;
	pmarker1 = &marker1;
	pmarker1 -> class_id = 0;

	// Message struct for second marker
	static tuiomessage_t marker2;
	static tuiomessage_t *pmarker2;
	pmarker2 = &marker2;
	pmarker2 -> class_id = 0;

	// Message struct for receive message
	tuiomessage_t recvmessage;
	tuiomessage_t *precvmessage;
	precvmessage = &recvmessage;

	// set recvmessage stuct to be all 0
	memset(precvmessage, 0, sizeof(recvmessage));

	// Message Struct for send message for queue
	distanceestimation sendmessage;
	distanceestimation *psendmessage;
	psendmessage = &sendmessage;

	// set recvmessage stuct to be all 0
	memset(psendmessage, 0, sizeof(sendmessage));

	// Create queue for send message
	DistanceQueue = xQueueCreate(10, sizeof(sendmessage));

	// pulse for pan/tilt servo
	volatile static uint16_t x_pulse = 765;
	volatile static uint16_t y_pulse = 645;

	static int sock = 0;
	static int ret = 0;
	static int conn = 0;
	unsigned char recv_buffer[300];
	struct sockaddr_in remote_addr;

	// x/y coordinate for marker
	static uint16_t x_Position = 500;
	static uint16_t y_Position = 500;

	// x/y deadzone limiter
	uint16_t nega_xlimit = 1237;
	uint16_t posi_xlimit = 293;
	uint16_t nega_ylimit = 1145;
	uint16_t posi_ylimit = 250;

	uint16_t xRightlimit = 550;
	uint16_t xLeftlimit = 450;
	uint16_t yToplimit = 550;
	uint16_t yBottomlimit = 450;

	// measurments for distance measurements
	static int16_t hypotenuse = 0;
	static int16_t opposite = 0;
	static int16_t adjecent = 0;

	/*DO NOT TOUCH!*/

	/* create a TCP socket */
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		debug_printf("Cannot Create Tuio Socket\r\n");
		return;
	}
	else {
		debug_printf("Tuio Socket Created\r\n");
	}

	remote_addr.sin_family = AF_INET;
	remote_addr.sin_port = htons(3000);
	remote_addr.sin_addr.s_addr = inet_addr("192.168.0.1");

	vTaskDelay(10000);
	if ((conn = connect(sock, (struct sockaddr *)(&remote_addr), sizeof(remote_addr))) < 0) {
		debug_printf("Cannot Connect Tuio Socket\r\n");
	}
	else {
		debug_printf("Tuio Socket Connected\r\n");
	}
	vTaskDelay(10000);

	/*DO NOT TOUCH!*/

	for (;;) {
		while ((ret = recv(sock, recv_buffer, sizeof(recv_buffer), 0)) > 0) {
			// parse the buffer into struct
			tuioclient_parser(recv_buffer, precvmessage);
			// check if valid fiducial marker
			if (precvmessage -> class_id != 0) {
				// store received data into marker structs
				if (pmarker1 -> class_id == 0 || pmarker1 -> class_id == precvmessage -> class_id) {
					memcpy(pmarker1, precvmessage, sizeof(recvmessage));
				}
				else if(pmarker2 -> class_id == 0 || pmarker2 -> class_id == precvmessage -> class_id) {
					memcpy(pmarker2, precvmessage, sizeof(recvmessage));
				}
				/* Toggle LED */
				NP2_LEDToggle();
			}
			// Marker Tracking
			if (precvmessage -> class_id == pmarker1 -> class_id) {
				if (((int)(precvmessage -> position_x * 1000) == x_Position) && ((int)(precvmessage -> position_x * 1000) == y_Position)) {
					// do nothing because message is identical which means same data
				}
				else {
					// get x/y coordinate and store into variables.
					x_Position = (int)(precvmessage -> position_x * 1000);
					y_Position = (int)(precvmessage -> position_y * 1000);

					// check for upright angle of tilt servo
					if (y_pulse < 645) {
						// check for x deadzone so and control servo to keep it as center as possible
						if (x_Position > xLeftlimit && x_Position != 0) {
							x_pulse = x_pulse - 3;
						}
						if (x_Position < xRightlimit && x_Position != 0) {
							x_pulse = x_pulse + 3;
						}
					}
					// check for upright angle of tilt servo
					else if (y_pulse > 645) {
						// check for x deadzone so and control servo to keep it as center as possible
						if (x_Position > xLeftlimit && x_Position != 0) {
							x_pulse = x_pulse + 3;
						}
						if (x_Position < xRightlimit && x_Position != 0) {
							x_pulse = x_pulse - 3;
						}
					}

					// check for y deadzone so and control servo to keep it as center as possible
					if (y_Position > yBottomlimit && y_Position != 0) {
						y_pulse = y_pulse - 3;
					}
					if (y_Position < yToplimit && y_Position != 0) {
						y_pulse = y_pulse + 3;
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

					/* Set PWM */
					TIM_SetCompare3 (TIM2, y_pulse);
					TIM_SetCompare4 (TIM2, x_pulse);
				}
			}

			// distance estimation
			opposite = (int)(1.33*((pmarker1 -> position_x * 1000) - (pmarker2 -> position_x * 1000)));
			adjecent = (int)((pmarker1 -> position_y * 1000) - (pmarker2 -> position_y * 1000));
			hypotenuse = sqrt((opposite*opposite)+(adjecent*adjecent));

			// put data into queue
			psendmessage -> opposite = opposite;
			psendmessage -> adjecent = adjecent;
			psendmessage -> hypotenuse = hypotenuse;

			/* Check if queue exists */
			if (DistanceQueue != NULL) {
				/*Send message to the front of the queue - wait atmost 0 ticks */
				if( xQueueSendToFront(DistanceQueue, ( void * ) psendmessage, ( portTickType ) 0 ) != pdPASS ) {
					continue;
				}
			}

			memset(&recv_buffer, 0, sizeof(recv_buffer));
			memset(precvmessage, 0, sizeof(recvmessage));
			memset(psendmessage, 0, sizeof(sendmessage));

			/* Delay for 100ms */
			vTaskDelay(100);
		}
	}
}

/***********************************
		DISTANCE ESTIMATION TASK
 ***********************************/
extern void DistanceTask(void * pvParameters) {

	// distance struct for distance estimation
	distanceestimation recvmessage;
	distanceestimation *precvmessage;
	precvmessage = &recvmessage;

	static uint16_t distance = 0;

	//set recvmessage stuct to be all 0
	memset(precvmessage, 0, sizeof(recvmessage));

	for(;;) {
		/* Check if queue exists */
		if (DistanceQueue != NULL) {
			/* Check for item received - block atmost for 10 ticks */
			if (xQueueReceive( DistanceQueue, precvmessage, 10 )) {
				// calculate distance
				distance = 17647/ (precvmessage -> hypotenuse);
				debug_printf("%d\n", distance);
			}
		}
		vTaskDelay(2500);
	}
}
