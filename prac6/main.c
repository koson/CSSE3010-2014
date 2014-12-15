/**
  ******************************************************************************
  * @file    ex17_fr_socket/main.c 
  * @author  MDS
  * @date    22-April-2014
  * @brief   FreeRTOS LWIP socket networking example. Binds and listens on both
  *			 UDP and TCP sockets.Will accept and return ASCII strings in capitals.
  *
  *			 LWIP socket API uses the same commands as standard socket interfaces.
  *			 i.e. connect, bind, accept, read, write, recv, recvfrom, send, etc.
  *
  *			 OSX\Linux Use netcat (nc) to access sockets:
  *										   e.g. For TCP: "nc 192.168.0.10 10"
  *											    For UDP: "nc -u 192.168.0.10 11"
  *			 Windows: Use Putty - set connection type to 'raw'.
  *			 Default IP Address: 192.168.0.10 (see netconfg.h)
  *
  *			 Open serial terminal (kermit) to view received bytes.
  *			 
  ******************************************************************************
  *  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "netduinoplus2.h"
#include "stm32f4xx_conf.h"
#include "debug_printf.h"
#include <string.h>
#include "math.h"

#include "netconf.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "tuioclient.h"

#include "tcpip.h"
#include "lwip/opt.h"
#include "lwip/api.h"
#include "lwip/sys.h"
#include "lwip/sockets.h"
#include "lwip/inet.h"

/* Private typedef -----------------------------------------------------------*/
struct AngleMessage {
	int id;
	char charcapture;
	long angle;
};

/* Private define ------------------------------------------------------------*/
#define TCP_SOCKET_PORT	10

/* Task Priorities -----------------------------------------------------------*/
#define TCPTASK_PRIORITY		( tskIDLE_PRIORITY + 2 )      
#define PANTILTTASK_PRIORITY	( tskIDLE_PRIORITY + 4 ) 
#define TUIOTASK_PRIORITY		( tskIDLE_PRIORITY + 3 ) 

/* Task Stack Allocations -----------------------------------------------------*/
#define TCPTASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 6 )
#define PANTILTTASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 6 )
#define TUIOTASK_STACK_SIZE			( configMINIMAL_STACK_SIZE * 6 )


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
xQueueHandle PanTiltQueue;    /* Queue used */
/* Private function prototypes -----------------------------------------------*/
static void Hardware_init();
void Hardware_PWM_init();
void TCPTask(void * pvParameters);
void PanTiltTask(void * pvParameters);
void TuioTCPTask(void * pvParameters);
void vApplicationIdleHook( void );	/* The idle hook is just used to stream data to the USB port.*/


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void) {
  
	NP2_boardinit();
	Hardware_init();
	Hardware_PWM_init();
	
	/* Initilaize the LwIP stack */
	LwIP_Init();

	xTaskCreate( (void *) &TCPTask, (const signed char *) "TCP", TCPTASK_STACK_SIZE, NULL, TCPTASK_PRIORITY, NULL);
	xTaskCreate( (void *) &PanTiltTask, (const signed char *) "Pan Tilt", PANTILTTASK_STACK_SIZE, NULL, PANTILTTASK_PRIORITY, NULL);
	xTaskCreate( (void *) &TuioTCPTask, (const signed char *) "Tuio TCP", TUIOTASK_STACK_SIZE, NULL, TUIOTASK_PRIORITY, NULL);

	/* Start scheduler */
	vTaskStartScheduler();

	return 0;
}


/**
  * @brief  TCP Task. Bind to a TCP Socket.
  * @param  None
  * @retval None
  */
void TCPTask(void * pvParameters) {

	int sock = 0;
	int tcpconn = 0;
	long size = 0;
	int ret = 0;
	int i;
	long recv_buffer[20];
	struct sockaddr_in address, remotehost;

	struct AngleMessage SendMessage; 
	PanTiltQueue = xQueueCreate(10, sizeof(SendMessage));       /* Create queue of length 10 Message items */

	memset(&SendMessage, 0, sizeof(SendMessage)); //set sendmessage stuct to be all 0

	/*Initialise Message Item payload */
	SendMessage.angle = 0;

	/*DO NOT TOUCH!*/

	/* create a TCP socket */
	if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		debug_printf("Cannot Create TCP Socket\r\n");
		return;
	}
	else {
		debug_printf("TCP Socket Created\r\n");
	}

	/* bind to port 80 at any interface */
	address.sin_family = AF_INET;
	address.sin_port = htons(TCP_SOCKET_PORT);
	debug_printf("%d\r\n", address.sin_port);
	address.sin_addr.s_addr = INADDR_ANY;

	if (bind(sock, (const struct sockaddr *)(&address), sizeof (address)) < 0) {
		debug_printf("Cannot Bind TCP Socket\r\n");
		return;
	}
	else {
		debug_printf("TCP Socket Binded\r\n");
	}

	/* listen for incoming connections (TCP listen backlog = 5) */
	listen(sock, 5);

	size = sizeof(remotehost);
	
	/*DO NOT TOUCH!*/

	for (;;) {

		/* Wait for connection */
		tcpconn = accept(sock, (struct sockaddr *)(&remotehost), (socklen_t *)&size);
		
		/* Read from Socket */		
		while ((ret = read(tcpconn, recv_buffer, sizeof(recv_buffer) )) > 0) { 

			//debug_printf("TCP Socket: ");
			for (i = 0; i < ret; i++) {
				SendMessage.charcapture = recv_buffer[0];
				if (PanTiltQueue != NULL) { /* Check if queue exists */
					/*Send message to the front of the queue - wait atmost 10 ticks */
					if( xQueueSendToFront(PanTiltQueue, ( void * ) &SendMessage, ( portTickType ) 10 ) != pdPASS ) {
						debug_printf("Failed to post the message, after 10 ticks.\n\r");
					}
				}
			}
		}
		vTaskDelay(10);
	}
}


/**
  * @brief  Pan Tilt Task, to control the Pan and Tilt.
  * @param  None
  * @retval None
  */
void PanTiltTask(void * pvParameters) {

	struct AngleMessage RecvMessage;
	volatile static int id_pan = 0;
	volatile static int id_tilt = 0;
	volatile static uint16_t x_pulse = 765;
	volatile static uint16_t y_pulse = 645;
	uint16_t nega_xlimit = 1237;
	uint16_t posi_xlimit = 293;
	uint16_t nega_ylimit = 1145;
	uint16_t posi_ylimit = 250;

	memset(&RecvMessage, 0, sizeof(RecvMessage)); //set recvmessage stuct to be all 0

	for (;;) {
		if (PanTiltQueue != NULL) { /* Check if queue exists */
			/* Check for item received - block atmost for 10 ticks */
			if (xQueueReceive( PanTiltQueue, &RecvMessage, 10 )) {
				//debug_printf("id = %d, char = %c, angle = %d\r\n", RecvMessage.id, RecvMessage.charcapture, RecvMessage.angle);
				/* display received item */
				if (RecvMessage.charcapture == 'W') {
					y_pulse = y_pulse - 3;
				}
				else if (RecvMessage.charcapture == 'A') {
					x_pulse = x_pulse + 3;
				}
				else if (RecvMessage.charcapture == 'S') {
					y_pulse = y_pulse + 3;
				}
				else if (RecvMessage.charcapture == 'D') {
					x_pulse = x_pulse - 3;
				}

				if (id_pan == 0 || id_pan == RecvMessage.id) {
					id_pan = RecvMessage.id;
				}
				else if(id_tilt == 0 || id_tilt == RecvMessage.id) {
					id_tilt = RecvMessage.id;
				}

				if (RecvMessage.id == id_pan) {
					x_pulse = 287 + (RecvMessage.angle * 6);
					debug_printf("pan angle = %d, pulse = %d \r\n", RecvMessage.angle, x_pulse);
				}
				if (RecvMessage.id == id_tilt) {
					y_pulse = 250 + (RecvMessage.angle * 5);
					debug_printf("pan angle = %d, pulse = %d \r\n", RecvMessage.angle, y_pulse);
				}
				/* Toggle LED */
				NP2_LEDToggle();
			}
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

		memset(&RecvMessage, 0, sizeof(RecvMessage));

		/* Delay for 10ms */
		vTaskDelay(10);
	}
}


/**
  * @brief  Tuio TCP Task
  * @param  None
  * @retval None
  */
void TuioTCPTask(void * pvParameters) {

	int sock = 0;
	int ret = 0;
	int conn = 0;
	long degree = 0;
	int id = 0;
	unsigned char recv_buffer[300];
	struct sockaddr_in remote_addr;
	tuiomessage_t marker1;
	tuiomessage_t *pmarker1;
	pmarker1 = &marker1;
	struct AngleMessage SendMessage; 
	PanTiltQueue = xQueueCreate(10, sizeof(SendMessage));

	memset(&SendMessage, 0, sizeof(SendMessage));	//set sendmessage stuct to be all 0
	memset(&marker1, 0, sizeof(marker1));			//set marker1 stuct to be all 0

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
				
		/* Read from Socket */		
		while ((ret = recv(sock, recv_buffer, sizeof(recv_buffer), 0)) > 0) { 
			tuioclient_parser(recv_buffer, pmarker1);
			id = pmarker1 -> class_id;
			degree = pmarker1 -> angle_a * 57.2957795;
			/*debug_printf("id = %d, angle rad = %d.%d, angle deg %d\r\n", 
				pmarker1 -> class_id, 
				(long)(pmarker1 -> angle_a * 100)/100, 
				(long)(pmarker1 -> angle_a * 100)%100, 
				degree);*/

			SendMessage.id = id;
			SendMessage.angle = degree;

			if (PanTiltQueue != NULL) { /* Check if queue exists */
				/*Send message to the front of the queue - wait atmost 10 ticks */
				if( xQueueSendToFront(PanTiltQueue, ( void * ) &SendMessage, ( portTickType ) 10 ) != pdPASS ) {
					debug_printf("Failed to post the message, after 10 ticks.\n\r");
				}
			}
		}
		vTaskDelay(10);
	}
}

/**
  * @brief  Hardware Initialisation.
  * @param  None
  * @retval None
  */
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

/**
  * @brief  Idle Application Task (Disabled)
  * @param  None
  * @retval None
  */
void vApplicationIdleHook( void ) {
	static portTickType xLastTx = 0;

	NP2_LEDOff();
	/* The idle hook simply prints the idle tick count */
	if ((xTaskGetTickCount() - xLastTx ) > (1000 / portTICK_RATE_MS)) {
		xLastTx = xTaskGetTickCount();
		debug_printf("IDLE Tick %d\n", xLastTx);		
	}
}

/**
  * @brief  vApplicationStackOverflowHook
  * @param  Task Handler and Task Name
  * @retval None
  */
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName ) {
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	NP2_LEDOff();
	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}