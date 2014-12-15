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

		vTaskDelay(100);
	}
}

/***********************************
			PAN/TILT TASK
 ***********************************/
extern void DistanceTask(void * pvParameters) {

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

	// Message struct for third marker
	static tuiomessage_t marker3;
	static tuiomessage_t *pmarker3;
	pmarker3 = &marker3;
	pmarker3 -> class_id = 0;

	// Message struct for forth marker
	static tuiomessage_t marker4;
	static tuiomessage_t *pmarker4;
	pmarker4 = &marker4;
	pmarker4 -> class_id = 0;

	// Message struct for receive message
	tuiomessage_t recvmessage;
	tuiomessage_t *precvmessage;
	precvmessage = &recvmessage;

	// set recvmessage stuct to be all 0
	memset(precvmessage, 0, sizeof(recvmessage));

	static int sock = 0;
	static int ret = 0;
	static int conn = 0;
	unsigned char recv_buffer[300];
	struct sockaddr_in remote_addr;

	// measurments for distance measurements
	static int16_t hypotenuse = 0;
	static int16_t opposite = 0;
	static int16_t adjecent = 0;
	static uint16_t distance = 0;

	static uint8_t frontflag = 0;
	static uint8_t backflag = 0;

	// pulse for pan/tilt servo
	volatile static uint16_t x_pulse = 765;
	volatile static uint16_t y_pulse = 645;

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
				else if(pmarker3 -> class_id == 0 || pmarker3 -> class_id == precvmessage -> class_id) {
					memcpy(pmarker3, precvmessage, sizeof(recvmessage));
				}
				else if(pmarker4 -> class_id == 0 || pmarker4 -> class_id == precvmessage -> class_id) {
					memcpy(pmarker4, precvmessage, sizeof(recvmessage));
				}
			}
			// Marker Tracking
			if (precvmessage -> class_id == pmarker1 -> class_id || precvmessage -> class_id == pmarker3 -> class_id) {
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
			if (precvmessage -> class_id == pmarker1 -> class_id || precvmessage -> class_id == pmarker2 -> class_id) {
				opposite = (int)(1.33*((pmarker1 -> position_x * 1000) - (pmarker2 -> position_x * 1000)));
				adjecent = (int)((pmarker1 -> position_y * 1000) - (pmarker2 -> position_y * 1000));
				frontflag = 1;
				backflag = 0;
			}
			if (precvmessage -> class_id == pmarker3 -> class_id || precvmessage -> class_id == pmarker4 -> class_id) {
				opposite = (int)(1.33*((pmarker3 -> position_x * 1000) - (pmarker4 -> position_x * 1000)));
				adjecent = (int)((pmarker3 -> position_y * 1000) - (pmarker4 -> position_y * 1000));
				backflag = 1;
				frontflag = 0;
			}
			hypotenuse = sqrt((opposite*opposite)+(adjecent*adjecent));
			distance = 155400/ hypotenuse;

			if (distsem != NULL) {	/* Check if semaphore exists */
				if ( xSemaphoreTake( distsem, 10 ) == pdTRUE ) {
					debug_printf("distance semaphore taken\r\n");
					if (frontflag == 1) {
						debug_printf("Front\r\n");
					}
					else if (backflag == 1) {
						debug_printf("Back\r\n");
					}
					debug_printf("Distance: %d\r\n\r\n", distance);
				}
			}

			memset(&recv_buffer, 0, sizeof(recv_buffer));
			memset(precvmessage, 0, sizeof(recvmessage));

			/* Delay for 100ms */
			vTaskDelay(100);
		}
	}
}

/*************************
		RADIO TASK
 *************************/
extern void RadioTask(void * pvParameters) {

	uint8_t scr_addr[] = {0x28, 0x92, 0x96, 0x42};
	uint8_t speed = 40;
	static uint8_t TX_Buffer[32];
	//Buffer to make rover stop
	const uint8_t TX_StopBuffer[] = {0x32, 0x78, 0x56, 0x34, 0x12, 0x28, 0x92, 0x96, 0x42, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	//Buffer to make rover move forward
	const uint8_t TX_ForwardBuffer[] = {0x32, 0x78, 0x56, 0x34, 0x12, 0x28, 0x92, 0x96, 0x42, 0,0,
		hamming_hbyte_encoder(speed & 0x0F),
		hamming_hbyte_encoder(speed >> 4),
		hamming_hbyte_encoder(speed & 0x0F),
		hamming_hbyte_encoder(speed >> 4),
		hamming_hbyte_encoder(5),
		hamming_hbyte_encoder(15),
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	//Buffer to make rover turn around
	const uint8_t TX_TurnaroundBuffer[] = {0x32, 0x78, 0x56, 0x34, 0x12, 0x28, 0x92, 0x96, 0x42, 0,0,
		hamming_hbyte_encoder(speed & 0x0F),
		hamming_hbyte_encoder(speed >> 4),
		hamming_hbyte_encoder(speed & 0x0F),
		hamming_hbyte_encoder(speed >> 4),
		hamming_hbyte_encoder(6),
		hamming_hbyte_encoder(2),
		0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static uint8_t RX_Buffer[32];
	uint8_t i;
	uint8_t waypoint[1] = {0};
	uint8_t waypointflag = 0;
	uint8_t boundaryflag = 0;
	uint8_t turnaroundflag = 0;
	volatile static uint8_t addr_check = 0;
	volatile static uint8_t passkey = 0;
	volatile static uint32_t seq_no = 0;
	volatile static uint8_t sensor_values = 0;
	volatile static uint8_t previous_sensor_value = 0;

	for(;;) {

		if (RadioTXQueue != NULL) {
			/* Check for item received - block atmost for 5 ticks */
			if (xQueueReceive( RadioTXQueue, TX_Buffer, 5 )) {
				//check if message is requesting for passkey
				if (TX_Buffer[0] == 0x30) {
					seq_no = 0;
					passkey = 0;
					nrf24l01plus_send_packet(TX_Buffer);
					debug_printf("Sent From Radio: ");
					for (i = 0; i < 32; i++) {
						debug_printf("%02x ", TX_Buffer[i]);
					}
					debug_printf("\r\n\r\n");
				}
				//check if message is requesting to move or sensor value
				else if (TX_Buffer[0] == 0x31 || TX_Buffer[0] == 0x32){
					TX_Buffer[9] = seq_no;
					TX_Buffer[10] = passkey;
					nrf24l01plus_send_packet(TX_Buffer);
					seq_no++;
					debug_printf("Sent From Radio: ");
					for (i = 0; i < 32; i++) {
						debug_printf("%02x ", TX_Buffer[i]);
					}
					debug_printf("\r\n\r\n");
				}
				else {
					debug_printf("Invalid Packet Type\r\n");
				}
				vTaskDelay(30);
			}
			memset(&TX_Buffer, 0, sizeof(TX_Buffer));
		}
		nrf24l01plus_mode_rx();
		if (nrf24l01plus_receive_packet(RX_Buffer) == 1) {
			if (RadioTXQueue != NULL) {
				xQueueReceive( RadioWPQueue, waypoint, 5 );
			}
			//check if the message is for me
			for (i = 0; i < 4; i++) {
				if (RX_Buffer[1+i] == scr_addr[i]) {
					addr_check++;
				}
			}
			if (addr_check == 4) {
				debug_printf("Received From Radio: ");
				for (i = 0; i < 32; i++ ) {
					debug_printf("%02x ", RX_Buffer[i]);
				}
				debug_printf("\r\n\r\n");
				//check if message is passkey, then pass the passkey into variable.
				if (RX_Buffer[0] == 0x30) {
					passkey = hamming_byte_decoder(RX_Buffer[12],RX_Buffer[11]);
					debug_printf("\r\n%02d:%02d:%02d  Passkey is %02x\r\n\r\n", min, sec, ms, passkey);
				}
				//check if message is sensor value
				if (RX_Buffer[0] == 0x31) {
					sensor_values = hamming_byte_decoder(RX_Buffer[12], RX_Buffer[11]);
					//check semaphore to print sensor
					if (sensorsem != NULL) {
						if ( xSemaphoreTake( sensorsem, 10 ) == pdTRUE ) {
							debug_printf("sensor semaphore taken\r\n");
							debug_printf("\r\n%02d:%02d:%02d  Sensor is %02x\r\n", min, sec, ms, sensor_values);
							ledtask(sensor_values);
						}
					}
					//check semaphore to execute boundary command instructions
					if (boundarysem != NULL) {
						if ( xSemaphoreTake( boundarysem, 10 ) == pdTRUE ) {
							debug_printf("boundary semaphore taken\r\n");
							boundaryflag = 1;
						}
					}
					if (sensor_values <= 0x1c && boundaryflag == 1) {
						xQueueReset(RadioTXQueue);
						vTaskDelay(10);
						if( xQueueSendToFront(RadioTXQueue, ( void * ) TX_StopBuffer, ( portTickType ) 0 ) != pdPASS ) {
							debug_printf("Unable to send RadioTXQueue\r\n");
						}
						boundaryflag = 0;
						turnaroundflag = 1;
					}
					else if (boundaryflag == 1){
						if( xQueueSendToFront(RadioTXQueue, ( void * ) TX_ForwardBuffer, ( portTickType ) 0 ) != pdPASS ) {
							debug_printf("Unable to send RadioTXQueue\r\n");
						}
					}
					//make rover turn around after reach boundary
					if (turnaroundflag == 1) {
						if( xQueueSendToFront(RadioTXQueue, ( void * ) TX_TurnaroundBuffer, ( portTickType ) 0 ) != pdPASS ) {
							debug_printf("Unable to send RadioTXQueue\r\n");
						}
						turnaroundflag = 0;
					}
					//check semaphore to execute waypoint command instructions
					if (waypointsem != NULL) {
						if ( xSemaphoreTake( waypointsem, 10 ) == pdTRUE ) {
							debug_printf("waypoint semaphore taken\r\n");
							waypointflag = 1;
						}
					}
					//check if the previous was not on line and current is on line, meaning reaching a line
					if (sensor_values != 0x1F && previous_sensor_value == 0x1F && waypointflag == 1) {
						waypoint[0] = waypoint[0]-1;
						debug_printf("waypoint count %d\r\n", waypoint[0]);
						if (waypoint[0] == 0) {
							xQueueReset(RadioTXQueue);
							vTaskDelay(10);
							if( xQueueSendToFront(RadioTXQueue, ( void * ) TX_StopBuffer, ( portTickType ) 0 ) != pdPASS ) {
								debug_printf("Unable to send RadioTXQueue\r\n");
							}
							waypointflag = 0;
						}
					}
					else if (waypointflag == 1){
						if( xQueueSendToFront(RadioTXQueue, ( void * ) TX_ForwardBuffer, ( portTickType ) 0 ) != pdPASS ) {
							debug_printf("Unable to send RadioTXQueue\r\n");
						}
					}
					previous_sensor_value = sensor_values;
				}
			}
			memset(&RX_Buffer, 0, sizeof(RX_Buffer));
			addr_check = 0;
		}
		vTaskDelay(100);
	}
}

/*************************
		JOYSTICK TASK
 *************************/
extern void ADCTask(void * pvParameters) {

	static uint8_t TX_JoystickBuffer[] = {0x32, 0x78, 0x56, 0x34, 0x12, 0x28, 0x92, 0x96, 0x42, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	volatile static uint16_t x_adc = 2050;
	volatile static uint16_t y_adc = 2050;
	static uint8_t speed = 0;
	static const uint8_t duration = 1;
	static uint8_t send_toggle = 0;
	static uint8_t send_stop = 0;

	for(;;) {
		/* Wait for ADC conversion to finish by polling the ADC Over Flag. */
		while ((ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR) != RESET) && (ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR) != RESET));
		while ((ADC_GetFlagStatus(ADC2, ADC_FLAG_OVR) != RESET) && (ADC_GetFlagStatus(ADC2, ADC_FLAG_OVR) != RESET));

		/* Extract ADC conversion values */
		x_adc = ADC_GetConversionValue(ADC1);
		y_adc = ADC_GetConversionValue(ADC2);

		/* checking ADC value */
		if (x_adc < 1850) { //forward
			TX_JoystickBuffer[15] = hamming_hbyte_encoder(10);
			speed = rescale(2047.5-x_adc, 2047.5, 0, 100);
			TX_JoystickBuffer[11] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[12] = hamming_hbyte_encoder(speed >> 4);
			TX_JoystickBuffer[13] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[14] = hamming_hbyte_encoder(speed >> 4);
			send_toggle = 1;
			send_stop = 0;
		}
		else if (x_adc > 2245) { //backwards
			TX_JoystickBuffer[15] = hamming_hbyte_encoder(5);
			speed = rescale(x_adc-2047.5, 2047.5, 0, 100);
			TX_JoystickBuffer[11] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[12] = hamming_hbyte_encoder(speed >> 4);
			TX_JoystickBuffer[13] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[14] = hamming_hbyte_encoder(speed >> 4);
			send_toggle = 1;
			send_stop = 0;
		}
		if (y_adc < 1850) { //right
			TX_JoystickBuffer[15] = hamming_hbyte_encoder(6);
			speed = rescale(2047.5-y_adc, 2047.5, 0, 100);
			TX_JoystickBuffer[11] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[12] = hamming_hbyte_encoder(speed >> 4);
			TX_JoystickBuffer[13] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[14] = hamming_hbyte_encoder(speed >> 4);
			send_toggle = 1;
			send_stop = 0;
		}
		else if (y_adc > 2245) { //left
			TX_JoystickBuffer[15] = hamming_hbyte_encoder(9);
			speed = rescale(y_adc-2047.5, 2047.5, 0, 100);
			TX_JoystickBuffer[11] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[12] = hamming_hbyte_encoder(speed >> 4);
			TX_JoystickBuffer[13] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[14] = hamming_hbyte_encoder(speed >> 4);
			send_toggle = 1;
			send_stop = 0;
		}
		if (x_adc >= 1850 && x_adc <= 2245 && y_adc >= 1850 && y_adc <= 2245 && send_stop == 0 ) { //stop
			debug_printf("Joystick Center!\r\n");
			speed = 0;
			TX_JoystickBuffer[11] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[12] = hamming_hbyte_encoder(speed >> 4);
			TX_JoystickBuffer[13] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[14] = hamming_hbyte_encoder(speed >> 4);
			send_stop = 1;
			send_toggle = 1;
		}


		TX_JoystickBuffer[16] = hamming_hbyte_encoder(duration);

		ADC_ClearFlag(ADC1, ADC_FLAG_OVR);
		ADC_ClearFlag(ADC2, ADC_FLAG_OVR);

		if (send_toggle == 1) {
			if (RadioTXQueue != NULL) {
				xQueueReset(RadioTXQueue);
				vTaskDelay(10);
				if( xQueueSendToFront(RadioTXQueue, ( void * ) TX_JoystickBuffer, ( portTickType ) 0 ) != pdPASS ) {
				}
			}
			send_toggle = 0;
		}
		vTaskDelay(1500);
	}
}

/*************************
		ACCE TASK
 *************************/
extern void ACCETask(void * pvParameters) {

	static uint8_t TX_JoystickBuffer[] = {0x32, 0x78, 0x56, 0x34, 0x12, 0x28, 0x92, 0x96, 0x42, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static uint8_t speed = 0;
	static const uint8_t duration = 1;
	static uint8_t send_toggle = 0;
	static uint8_t send_stop = 0;
	uint8_t recvbyte[4];
	uint8_t i;
	short x=0, y=0;

	for(;;) {

		//start
		I2C_GenerateSTART(I2C1, ENABLE);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
		//write device addr
		I2C_Send7bitAddress(I2C1, 0x3A, I2C_Direction_Transmitter);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
		//write device register
		I2C_SendData(I2C1, 0x01);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
		//repeat start
		I2C_GenerateSTART(I2C1, ENABLE);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)); 
		//read device addr
		I2C_Send7bitAddress(I2C1, 0x3B, I2C_Direction_Receiver);
		while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); 

		memset(recvbyte, 0, 4);

		for (i = 0; i < 4; i++) {

			if (i < 3) {
				I2C_AcknowledgeConfig(I2C1, ENABLE);
				while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

			} 
			if (i == 3) {
				I2C_AcknowledgeConfig(I2C1, DISABLE);
				while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));

			}
			//read device register
			recvbyte[i] = I2C_ReceiveData(I2C1);
		}
		//stop
		I2C_GenerateSTOP(I2C1, ENABLE);

		x = recvbyte[0] << 8 | recvbyte[1];
		y = recvbyte[2] << 8 | recvbyte[3];

		if (x > 16000) {
			x = 16000;
		}
		else if (x < -16000) {
			x = -16000;
		}
		if (y > 16000) {
			y = 16000;
		}
		else if (y < -16000) {
			y = -16000;
		}

		x = rescale(x, 16000.0, 0, 100);
		y = rescale(y, 16000.0, 0, 100);

		if (x < -10) { // Move Forward
			speed = (-1)*x;
			TX_JoystickBuffer[11] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[12] = hamming_hbyte_encoder(speed >> 4);
			TX_JoystickBuffer[13] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[14] = hamming_hbyte_encoder(speed >> 4);
			TX_JoystickBuffer[15] = hamming_hbyte_encoder(10);
			send_toggle = 1;
			send_stop = 0;
		}
		else if (x > 10) { // Move Backwards
			speed = x;
			TX_JoystickBuffer[11] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[12] = hamming_hbyte_encoder(speed >> 4);
			TX_JoystickBuffer[13] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[14] = hamming_hbyte_encoder(speed >> 4);
			TX_JoystickBuffer[15] = hamming_hbyte_encoder(5);
			send_toggle = 1;
			send_stop = 0;
		}
		if (y < -10) { //left
			speed = (-1)*y;
			TX_JoystickBuffer[11] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[12] = hamming_hbyte_encoder(speed >> 4);
			TX_JoystickBuffer[13] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[14] = hamming_hbyte_encoder(speed >> 4);
			TX_JoystickBuffer[15] = hamming_hbyte_encoder(6);
			send_toggle = 1;
			send_stop = 0;
		}
		else if (y > 10) { //right
			speed = y;
			TX_JoystickBuffer[11] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[12] = hamming_hbyte_encoder(speed >> 4);
			TX_JoystickBuffer[13] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[14] = hamming_hbyte_encoder(speed >> 4);
			TX_JoystickBuffer[15] = hamming_hbyte_encoder(9);
			send_toggle = 1;
			send_stop = 0;
		}
		if (x >= -10 && x <= 10 && y >= -10 && y <= 10 && send_stop == 0 ) { //stop
			debug_printf("Accelerometer Center!\r\n");
			speed = 0;
			TX_JoystickBuffer[11] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[12] = hamming_hbyte_encoder(speed >> 4);
			TX_JoystickBuffer[13] = hamming_hbyte_encoder(speed & 0x0F);
			TX_JoystickBuffer[14] = hamming_hbyte_encoder(speed >> 4);
			send_stop = 1;
			send_toggle = 1;
		}

		TX_JoystickBuffer[16] = hamming_hbyte_encoder(duration);

		if (send_toggle == 1) {
			if (RadioTXQueue != NULL) {
				xQueueReset(RadioTXQueue);
				vTaskDelay(10);
				if( xQueueSendToFront(RadioTXQueue, ( void * ) TX_JoystickBuffer, ( portTickType ) 0 ) != pdPASS ) {
				}
			}
			send_toggle = 0;
		}
		vTaskDelay(1500);
	}
}


/************************************************************************************************************************
											RTOS CO ROUTINES FUNCTIONS
 ************************************************************************************************************************
 ***********************************************************************************************************************/
/*************************
		TIMER COROUTINE
 *************************/
extern void TimerCoroutine( xCoRoutineHandle xTimer, unsigned portBASE_TYPE uxIndex ) {

	static portTickType count = 0;
	static uint32_t imin = 0, isec = 0, ims = 0;
	
	min = 0;
	sec = 0;
	ms = 0;

	// Co-routines must start with a call to crSTART().
	crSTART( xTimer );

	for( ;; )
	{
		// Delay for a fixed period.
		count = xTaskGetTickCount();
		ims = (count/6.8);
		ms = (ims%1000)/10;
		isec = ims/1000;
		sec = isec%60;
		imin = isec/60;
		min = imin%60;

		if (gettimeflag == 1) {
			debug_printf("%02d:%02d:%02d\r\n\r\n", min, sec, ms);
			gettimeflag = 0;
		}
		NP2_LEDToggle();
		crDELAY( xTimer, 10 );
	}

	// Co-routines must end with a call to crEND().
	crEND();
}


/***********************************************************************************************************************
 ************************************************************************************************************************
											OTHER TASK FUNCTIONS											
 ************************************************************************************************************************
 ***********************************************************************************************************************/
/*************************
		LED FUNCTION
 *************************/
extern void ledtask(uint8_t input) {

	uint8_t ledbits[5];

	ledbits[0] = !(!!(input & 0x1));
	ledbits[1] = !(!!(input & 0x2));
	ledbits[2] = !(!!(input & 0x4));
	ledbits[3] = !(!!(input & 0x8));
	ledbits[4] = !(!!(input & 0x10));

	GPIO_WriteBit(NP2_D4_GPIO_PORT, NP2_D4_PIN, ledbits[0]);
	GPIO_WriteBit(NP2_D5_GPIO_PORT, NP2_D5_PIN, ledbits[1]);
	GPIO_WriteBit(NP2_D6_GPIO_PORT, NP2_D6_PIN, ledbits[2]);
	GPIO_WriteBit(NP2_D7_GPIO_PORT, NP2_D7_PIN, ledbits[3]);
	GPIO_WriteBit(NP2_D8_GPIO_PORT, NP2_D8_PIN, ledbits[4]);

	debug_printf("led %d %d %d %d %d\r\n\r\n", ledbits[0], ledbits[1], ledbits[2], ledbits[3], ledbits[4]);
}

/*********************
	Delay Function
 *********************/
extern void Delay(__IO unsigned long nCount) {

	/* Delay a specific amount before returning */
	while(nCount--)	{
	}
}

/*************************
	RESCALE FUNCTION
 *************************/
extern int rescale(float AdcValue, const float maxValue, const int newLow, const int newHigh)
{
	unsigned int newvalue = 0;

	newvalue = (int)((double) AdcValue / maxValue * (newHigh - newLow)) + newLow;

	return newvalue;
}