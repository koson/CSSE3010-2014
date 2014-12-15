/**
  ******************************************************************************
  * @file    ex14_fr_ledflashing/main.c 
  * @author  MDS
  * @date    15-April-2014
  * @brief   FreeRTOS LED Flashing program.Creates a task to flash the onboard
  *			 Blue LED. Note the Idle task will also flash the Blue LED. 
  ******************************************************************************
  *  
  */ 

/* Includes ------------------------------------------------------------------*/
#include "netduinoplus2.h"
#include "stm32f4xx_conf.h"
#include "debug_printf.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static volatile xSemaphoreHandle nextten = NULL;
static volatile xSemaphoreHandle nexthundred = NULL;
/* Private function prototypes -----------------------------------------------*/
void Hardware_init();
void ApplicationIdleHook(void); /* The idle hook is used to blink the Blue 'Alive LED' every second */
void LED_Task(void );
void Task1_Task(void);
void Task2_Task(void);
void Task3_Task(void);

/* Task Priorities ------------------------------------------------------------*/
#define general_PRIORITY					( tskIDLE_PRIORITY + 2 )

/* Task Stack Allocations -----------------------------------------------------*/
#define general_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE * 2 )



/**
  * @brief  Starts all the other tasks, then starts the scheduler.
  * @param  None
  * @retval None
  */
int main(void) {

	NP2_boardinit();
	Hardware_init();
	
    xTaskCreate((void *) &Task1_Task, (const signed char *) "Task 1", general_TASK_STACK_SIZE, NULL, general_PRIORITY, NULL );
    xTaskCreate((void *) &Task2_Task, (const signed char *) "Task 2", general_TASK_STACK_SIZE, NULL, general_PRIORITY, NULL );
    xTaskCreate((void *) &Task3_Task, (const signed char *) "Task 3", general_TASK_STACK_SIZE, NULL, general_PRIORITY, NULL );

    vSemaphoreCreateBinary(nextten);
	vSemaphoreCreateBinary(nexthundred);

	xSemaphoreTake(nextten, 10);
	xSemaphoreTake(nexthundred, 10);

	/* Start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */

	vTaskStartScheduler();
	
	/* We should never get here as control is now taken by the scheduler. */
  	return 0;
}

void Task1_Task(void) {

	static volatile uint16_t counter = 0;
	static uint8_t data[4];

	NP2_LEDOff();

	for (;;) {

		data[3] = ((counter >> 3) & 0x1);
		data[2] = ((counter >> 2) & 0x1);
		data[1] = ((counter >> 1) & 0x1);
		data[0] = (counter & 0x1);

		GPIO_SetBits(NP2_A3_GPIO_PORT, NP2_A3_PIN);
		GPIO_WriteBit(NP2_D6_GPIO_PORT, NP2_D6_PIN, data[3] & 0x01);
		GPIO_WriteBit(NP2_D7_GPIO_PORT, NP2_D7_PIN, data[2] & 0x01);
		GPIO_WriteBit(NP2_D8_GPIO_PORT, NP2_D8_PIN, data[1] & 0x01);
		GPIO_WriteBit(NP2_D9_GPIO_PORT, NP2_D9_PIN, data[0] & 0x01);
		
		counter++;

		if(counter == 10) {
			if (nextten != NULL) {	/* Check if semaphore exists */
				xSemaphoreGive(nextten);
			}
			counter = 0;
			debug_printf("sent ten!\n");
		}
		vTaskDelay(10000);
		GPIO_ResetBits(NP2_A3_GPIO_PORT, NP2_A3_PIN);
		vTaskDelay(1);
	}
}

void Task2_Task(void) {

	static volatile uint16_t counter = 0;
	static uint8_t data[4];
	NP2_LEDOff();

	for (;;) {

		data[3] = ((counter >> 3) & 0x1);
		data[2] = ((counter >> 2) & 0x1);
		data[1] = ((counter >> 1) & 0x1);
		data[0] = (counter & 0x1);

		GPIO_SetBits(NP2_A4_GPIO_PORT, NP2_A4_PIN);
		GPIO_WriteBit(NP2_D2_GPIO_PORT, NP2_D2_PIN, data[3] & 0x01);
		GPIO_WriteBit(NP2_D3_GPIO_PORT, NP2_D3_PIN, data[2] & 0x01);
		GPIO_WriteBit(NP2_D4_GPIO_PORT, NP2_D4_PIN, data[1] & 0x01);
		GPIO_WriteBit(NP2_D5_GPIO_PORT, NP2_D5_PIN, data[0] & 0x01);
		
		if (nextten != NULL) {	/* Check if semaphore exists */
			if ( xSemaphoreTake( nextten, 10 ) == pdTRUE ) {
				counter++;
				debug_printf("received ten!\n");
        	}
		}

		if(counter == 10) {

			if (nexthundred != NULL) {	/* Check if semaphore exists */
				xSemaphoreGive(nexthundred);
			}
			counter = 0;
			debug_printf("sent hundred!\n");
		}		
		GPIO_ResetBits(NP2_A4_GPIO_PORT, NP2_A4_PIN);
		vTaskDelay(1);
	}
}

void Task3_Task(void) {

	static volatile uint16_t counter = 0;
	static uint8_t data[4];
	NP2_LEDOff();

	for (;;) {

		data[1] = ((counter >> 1) & 0x1);
		data[0] = (counter & 0x1);

		GPIO_SetBits(NP2_A5_GPIO_PORT, NP2_A5_PIN);
		GPIO_WriteBit(NP2_D0_GPIO_PORT, NP2_D0_PIN, data[1] & 0x01);
		GPIO_WriteBit(NP2_D1_GPIO_PORT, NP2_D1_PIN, data[0] & 0x01);
		
		if (nexthundred != NULL) {	/* Check if semaphore exists */
			if ( xSemaphoreTake( nexthundred, 10 ) == pdTRUE ) {
				counter++;
				debug_printf("received hundred!\n");
        	}
		}

		if(counter == 4) {
			counter = 0;
		}
		vTaskDelay(3);
		GPIO_ResetBits(NP2_A5_GPIO_PORT, NP2_A5_PIN);
		vTaskDelay(1);
	}
}

/**
  * @brief  Hardware Initialisation.
  * @param  None
  * @retval None
  */
void Hardware_init(void) {

	GPIO_InitTypeDef  GPIO_InitStructure;
	
	portDISABLE_INTERRUPTS();	//Disable interrupts

	NP2_LEDInit();				//Initialise Blue LED
	NP2_LEDOff();				//Turn off Blue LED

  	/* Configure the GPIO pin */
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

  	GPIO_InitStructure.GPIO_Pin = NP2_A3_PIN;
  	GPIO_Init(NP2_A3_GPIO_PORT, &GPIO_InitStructure);
  	GPIO_InitStructure.GPIO_Pin = NP2_A4_PIN;
  	GPIO_Init(NP2_A4_GPIO_PORT, &GPIO_InitStructure);
  	GPIO_InitStructure.GPIO_Pin = NP2_A5_PIN;
  	GPIO_Init(NP2_A5_GPIO_PORT, &GPIO_InitStructure);

  	GPIO_InitStructure.GPIO_Pin = NP2_D9_PIN;
  	GPIO_Init(NP2_D9_GPIO_PORT, &GPIO_InitStructure);
  	GPIO_InitStructure.GPIO_Pin = NP2_D8_PIN;
  	GPIO_Init(NP2_D8_GPIO_PORT, &GPIO_InitStructure);
  	GPIO_InitStructure.GPIO_Pin = NP2_D7_PIN;
  	GPIO_Init(NP2_D7_GPIO_PORT, &GPIO_InitStructure);
  	GPIO_InitStructure.GPIO_Pin = NP2_D6_PIN;
  	GPIO_Init(NP2_D6_GPIO_PORT, &GPIO_InitStructure);
  	GPIO_InitStructure.GPIO_Pin = NP2_D5_PIN;
  	GPIO_Init(NP2_D5_GPIO_PORT, &GPIO_InitStructure);
  	GPIO_InitStructure.GPIO_Pin = NP2_D4_PIN;
  	GPIO_Init(NP2_D4_GPIO_PORT, &GPIO_InitStructure);
  	GPIO_InitStructure.GPIO_Pin = NP2_D3_PIN;
  	GPIO_Init(NP2_D3_GPIO_PORT, &GPIO_InitStructure);
  	GPIO_InitStructure.GPIO_Pin = NP2_D2_PIN;
  	GPIO_Init(NP2_D2_GPIO_PORT, &GPIO_InitStructure);
  	GPIO_InitStructure.GPIO_Pin = NP2_D1_PIN;
  	GPIO_Init(NP2_D1_GPIO_PORT, &GPIO_InitStructure);
  	GPIO_InitStructure.GPIO_Pin = NP2_D0_PIN;
  	GPIO_Init(NP2_D0_GPIO_PORT, &GPIO_InitStructure);

	portENABLE_INTERRUPTS();	//Enable interrupts
}

/**
  * @brief  Application Tick Task.
  * @param  None
  * @retval None
  */
void vApplicationTickHook(void) {

	NP2_LEDOff();
}

/**
  * @brief  Idle Application Task
  * @param  None
  * @retval None
  */
void vApplicationIdleHook(void) {
	static portTickType xLastTx = 0;

	NP2_LEDOff();

	for (;;) {

		/* The idle hook simply prints the idle tick count, every second */
		if ((xTaskGetTickCount() - xLastTx ) > (1000 / portTICK_RATE_MS)) {

			xLastTx = xTaskGetTickCount();

			//debug_printf("IDLE Tick %d\n", xLastTx);

			/* Blink Alive LED */
			NP2_LEDToggle();		
		}
	}
}

/**
  * @brief  vApplicationStackOverflowHook
  * @param  Task Handler and Task Name
  * @retval None
  */
void vApplicationStackOverflowHook(xTaskHandle pxTask, signed char *pcTaskName) {
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	NP2_LEDOff();
	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}

