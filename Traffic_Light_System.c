/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wwrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.

This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.

This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.

Here is a description of the project's functionality:

The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html

An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html

The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.

The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().

The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/

// Standard includes.
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4_discovery.h"

// Kernel includes.
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

// Forward declarations.
static void Task_1( void *pvParameters );
static void Task_2( void *pvParameters );
static void Task_3( void *pvParameters );
static void Task_4( void *pvParameters );
static void Reset_Cars();
static void Add_Car();
static void Shift_Cars();
static void next_Light(xTimerHandle pxTimer);
static TickType_t getTimeRemaining(xTimerHandle pxTimer);
static void Update_Light(uint16_t trafficLightState);
static void Update_Cars(int trafficState);
static int Get_RandomNumber();

// Handles and #defines.
xTimerHandle xTraffic_light_timer = 0;
xQueueHandle xTraffic_rate_queue = 0;
xQueueHandle xTraffic_light_state_queue = 0;
xQueueHandle xTraffic_state_queue = 0;
xQueueHandle xTimer_expiry_queue = 0; // 1 or 0
#define mainQUEUE_LENGTH 100
#define green  	0
#define yellow 	1
#define red 	2
#define green_red_max_duration 6000
#define green_red_min_duration 3000
#define yellow_duration 2000
#define adc_output_max 65536
#define adc_period 500


/*-----------------------------------------------------------*/

int main(void)
{
	// Ensure all priority bits are assigned as preemption priority bits.
	// http://www.freertos.org/RTOS-Cortex-M3-M4.html
	NVIC_SetPriorityGrouping( 0 );

	// Hardware configuration.
	// Enable GPIOC clock.
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	printf("%s\n", "GPIOC clock enabled.");

	// Initialize GPIOC PC0-2 and PC6-8.
	// (For output)
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOC, &GPIO_InitStruct);
	printf("%s\n", "GPIOC PC0-2 and PC6-8 configured.");

	// Initialize GPIOC PC3.
	// (For input).
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;

	GPIO_Init(GPIOC, &GPIO_InitStruct);
	printf("%s\n", "GPIOC PC3 configured.");

	// Initialize ADC1.
	// Enable clock for ADC1.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	printf("%s\n", "ADC1 clock enabled.");

	// Configure ADC1.
	ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_Init(ADC1, &ADC_InitStruct);
	printf("%s\n", "ADC1 initialized.");

	// Clear traffic lights.
	GPIO_ResetBits(GPIOC, GPIO_Pin_0);
	GPIO_ResetBits(GPIOC, GPIO_Pin_1);
	GPIO_ResetBits(GPIOC, GPIO_Pin_2);

	// Clear cars.
	Reset_Cars();

	// Create timers.
	xTraffic_light_timer = xTimerCreate( "Traffic_light_timer",
									 	 pdMS_TO_TICKS(6000),
										 pdTRUE,
										 (void *) 0,
										 next_Light);

	// Create queues.
	xTraffic_rate_queue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint16_t ) );
	xTraffic_light_state_queue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint16_t ) );
	xTraffic_state_queue = xQueueCreate( mainQUEUE_LENGTH, sizeof( int ) );
	xTimer_expiry_queue = xQueueCreate( mainQUEUE_LENGTH, sizeof( uint16_t ) );

	// Add to the registry, for the benefit of kernel aware debugging.
	vQueueAddToRegistry( xTraffic_rate_queue, "Traffic_rate_queue" );
	vQueueAddToRegistry( xTraffic_light_state_queue, "Traffic_light_state_queue" );
	vQueueAddToRegistry( xTraffic_state_queue, "Traffic_state_queue" );

	// Create tasks and priorities.
	xTaskCreate( Task_1, "Task_1", configMINIMAL_STACK_SIZE, NULL, 4, NULL);
	xTaskCreate( Task_2, "Task_2", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate( Task_3, "Task_3", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate( Task_4, "Task_4", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	// Start the tasks and timer running.
	vTaskStartScheduler();

	return 0;
}

/*-----------------------------------------------------------*/

static void Task_1( void *pvParameters )
{
	uint16_t trafficRate = 0;
	while(1)
	{
		//printf("%s\n", "Task 1");

		// ADC Setup
		ADC_Cmd(ADC1, ENABLE);
		ADC_RegularChannelConfig(ADC1, 13, 1, ADC_SampleTime_3Cycles);
		ADC_SoftwareStartConv(ADC1);
		while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == RESET);
		trafficRate = ADC_GetConversionValue(ADC1);
		ADC_Cmd(ADC1, DISABLE);

		//printf("%s %u\n", "traffic rate: ", trafficRate);

		for(int i = 0; i <2; i++){
			if (!(xQueueSend(xTraffic_rate_queue, &trafficRate, adc_period)) )
			{
				printf("%s\n", "Task 1 failed to obtain ADC conversion value.");
			}
		}

		//printf("%s\n", "Task 1 send traffic rate");

		vTaskDelay(pdMS_TO_TICKS(adc_period));

	}
}

/*-----------------------------------------------------------*/

static void Task_2( void *pvParameters )
{
	uint16_t trafficRate = 0;
	uint16_t trafficRateScaled = 0;
	uint16_t trafficLightState = 0;
	uint16_t randomNumber = 0;
	int trafficState = 0b0000000000000000000;

	while(1)
	{
		//printf("%s\n", "Task 2");

		// Get random number in range 0 - 99.
		randomNumber = Get_RandomNumber();
		//randomNumber = rand() % 100;
		//printf("%s %u\n", "randomNumber: ", randomNumber);

		// Obtain traffic rate from queue.
		if( !(xQueueReceive(xTraffic_rate_queue, &trafficRate, (adc_period / 2))) )
		{
			printf("%s\n", "Task 2 did not receive a value from Traffic_rate_queue.");
		}

		//printf("%s\n", "Task 2 receive traffic rate");

		// Obtain traffic light state from queue.
		if ( !(xQueueReceive(xTraffic_light_state_queue, &trafficLightState, (adc_period / 2))) )
		{
			printf("%s\n", "Task 2 did not receive a value from Traffic_light_state_queue.");
		}

		//printf("%s\n", "Task 2 receive traffic light state");

		//printf("%s %u\n", "Traffic light state: ", trafficLightState);

		// Convert raw traffic rate to an integer in range 15 - 100.
		// We divide by adc_max_output.
		// We add 17, since the high potentiometer setting produces
		// a trafficRateScaled value of ~98, due to the inaccuracy of the potentiometer.
		// Then adding 22 ensures we always obtain a high value of 100.
		trafficRateScaled = ((trafficRate * 87)/adc_output_max) + 15;
		//printf("%s %u\n", "trafficRateScaled: ", trafficRateScaled);

		// Determine semi-randomly whether another car is added this iteration.
		// 0 = true, 1 = false.
		uint16_t addCar = 1;
		if (randomNumber <= trafficRateScaled)
		{
			//printf("%u %u %s\n", trafficRateScaled, randomNumber, "add car");
			addCar = 0;
		}

		// Advance cars according to traffic light state.
		if (trafficLightState == green)
		{
			trafficState = (trafficState >> 1);
		}
		else
		{
			// Separate state before and after the red light stop line.
			int cars_before_line = (trafficState & 0b1111111100000000000) >> 11;
			int cars_after_line = trafficState & 0b0000000011111111111;

			// Advance cars before line as necessary.
			int stopped_cars = 0b0000000000000000000;
			int temp = cars_before_line;

			while ((temp & 0b0000000000000000001) == 0b0000000000000000001)
			{
				stopped_cars = (stopped_cars << 1) | 0b0000000000000000001;
				temp = temp >> 1;
			}

			cars_before_line = (cars_before_line >> 1) | stopped_cars;

			// Advance cars after line.
			cars_after_line = (cars_after_line >> 1);

			// Recombine into .
			trafficState = (cars_before_line << 11) | cars_after_line;
		}

		if (addCar == 0)
		{
			trafficState = trafficState | 0b1000000000000000000;
		}

		if (!(xQueueSend(xTraffic_state_queue, &trafficState, (adc_period / 2))) )
		{
			printf("%s\n", "Task 2 failed to send to traffic_state_queue.");
		}

		//printf("%s\n", "Task 2 send traffic state");

		vTaskDelay(pdMS_TO_TICKS(adc_period));
	}
}


/*-----------------------------------------------------------*/

static void Task_3( void *pvParameters )
{
	uint16_t trafficRate = 0;
	uint16_t trafficRateScaled = 0;
	uint16_t green_duration = 0;
	uint16_t red_duration = 0;
	uint16_t light_duration = 0;


	// Initializes to -1, so on first iteration we enter one of the conditional
	// statements and start the timer.
	uint16_t light = green;
	uint16_t Timer_expired = 1; // To start the timer for the very first time.

	while(1)
	{
		//printf("%s\n", "Task 3");
		if( xQueueReceive(xTimer_expiry_queue, &Timer_expired, 10) ){
			//printf("%s %u\n", "Task3: ", Timer_expired);
			light = (light+1) % 3;

		}

		if( xQueueReceive(xTraffic_rate_queue, &trafficRate, (adc_period / 2)) )
		{
			//printf("%s\n", "Task 3 receive traffic rate");

			// Convert raw traffic rate to a duration in range 3000 - 6000 ms.
			// We divide by 4096 = 2^12 since the ADC output is 12-bit.
			trafficRateScaled = (trafficRate * green_red_min_duration)/adc_output_max;
			green_duration = trafficRateScaled + green_red_min_duration;
			red_duration = green_red_max_duration - trafficRateScaled;


			// Get time remaining for timer.
			TickType_t time_remaining = getTimeRemaining(xTraffic_light_timer);

			// If light has just changed, restart timer.
			switch(light)
			{
				case green:

					if (Timer_expired)
					{
						time_remaining = pdMS_TO_TICKS(green_duration);
					}
					light_duration = green_duration;
					//printf("%s\n", "green");
					break;
				case yellow:
					if (Timer_expired)
					{
						time_remaining = pdMS_TO_TICKS(yellow_duration);
					}
					light_duration = 2000;
					//printf("%s\n", "yellow");
					break;
				case red:
					if (Timer_expired)
					{
						time_remaining = pdMS_TO_TICKS(red_duration);
					}
					light_duration = red_duration;
					//printf("%s\n", "red");
					break;
			}

			// If user has shortened period for current light, shorten timer accordingly.
			if (time_remaining >= pdMS_TO_TICKS(light_duration))
			{
				// Changes remaining timer duration.
				// Starts timer if stopped.

				xTimerChangePeriod(xTraffic_light_timer, pdMS_TO_TICKS(light_duration), pdMS_TO_TICKS(500));
			}

			// Add traffic light state to traffic_light_state_queue.
			for(int i = 0; i <2; i++){
				if (!(xQueueSend(xTraffic_light_state_queue, &light, (adc_period / 2))) )
				{
					printf("%s\n", "Task 3 failed to add traffic light state to traffic_light_state_queue.");
				}
			}

			//printf("%s\n", "Task 3 send traffic light state");

			vTaskDelay(pdMS_TO_TICKS((adc_period / 2)));
		}
		else
		{
			printf("%s\n", "Task 3 did not receive a value from Traffic_rate_queue.");
		}
		Timer_expired = 0;
	}
}

/*-----------------------------------------------------------*/

static void Task_4( void *pvParameters )
{
	uint16_t trafficLightState = 0;
	int trafficState = 0;

	while(1)
	{
		//printf("%s\n", "Task 4");

		if( !(xQueueReceive(xTraffic_light_state_queue, &trafficLightState, (adc_period / 2))) )
		{
			printf("%s\n", "Task 4 did not receive a value from Traffic_light_state_queue.");
		}

		//printf("%s\n", "Task 4 receive traffic light state");

		if( !(xQueueReceive(xTraffic_state_queue, &trafficState, (adc_period / 2))) )
		{
			printf("%s\n", "Task 4 did not receive a value from Traffic_state_queue.");
		}

		//printf("%s\n", "Task 4 receive traffic state");

		Update_Light(trafficLightState);
		Update_Cars(trafficState);

		vTaskDelay(pdMS_TO_TICKS(adc_period));
	}
}

/*-----------------------------------------------------------*/
// Helper functions.

static void Reset_Cars()
{
	// Cycle reset bit (PC8).
	GPIO_SetBits(GPIOC, GPIO_Pin_8);
	GPIO_ResetBits(GPIOC, GPIO_Pin_8);
}

static void Add_Car()
{
	// PC6 = H
	// PC8 = H
	// Cycle PC7
	GPIO_SetBits(GPIOC, GPIO_Pin_6); // Set data = h
	GPIO_SetBits(GPIOC, GPIO_Pin_8); // Set reset = h
	GPIO_SetBits(GPIOC, GPIO_Pin_7); // Cycle clock (h-l)
	GPIO_ResetBits(GPIOC, GPIO_Pin_7);
}

static void Shift_Cars()
{
	GPIO_ResetBits(GPIOC, GPIO_Pin_6); // Set data = l
	GPIO_SetBits(GPIOC, GPIO_Pin_8); // Set reset = h
	GPIO_SetBits(GPIOC, GPIO_Pin_7); // Cycle clock (h-l)
	GPIO_ResetBits(GPIOC, GPIO_Pin_7);
}

static void next_Light(xTimerHandle pxTimer)
{
	uint16_t Timer_expired = 1;
	xQueueSend(xTimer_expiry_queue, &Timer_expired, adc_period);

}

static TickType_t getTimeRemaining(xTimerHandle pxTimer)
{
	// Todo:
	// Handle tick count overflow.
	TickType_t expiry_time = xTimerGetExpiryTime(pxTimer);
	TickType_t current_time = xTaskGetTickCount();

	TickType_t time_remaining = 0;
	if (expiry_time > current_time)
	{
		time_remaining = expiry_time - current_time;
	}

	return time_remaining;
}

static void Update_Light(uint16_t trafficLightState)
{
	switch (trafficLightState)
	{
		case green:
			GPIO_ResetBits(GPIOC, GPIO_Pin_0);
			GPIO_ResetBits(GPIOC, GPIO_Pin_1);
			GPIO_SetBits(GPIOC, GPIO_Pin_2);
			break;
		case yellow:
			GPIO_ResetBits(GPIOC, GPIO_Pin_0);
			GPIO_SetBits(GPIOC, GPIO_Pin_1);
			GPIO_ResetBits(GPIOC, GPIO_Pin_2);
			break;
		case red:
			GPIO_SetBits(GPIOC, GPIO_Pin_0);
			GPIO_ResetBits(GPIOC, GPIO_Pin_1);
			GPIO_ResetBits(GPIOC, GPIO_Pin_2);
			break;
	}
}

static void Update_Cars(int trafficState)
{
	for (int i = 0; i < 19; i++)
	{
		if ((trafficState & 0b1) == 0b1)
		{
			Add_Car();
			//printf("%s\n", "add");
		}
		else
		{
			Shift_Cars();
			//printf("%s\n", "shift");
		}

		trafficState = trafficState >> 1;
	}
}

static int Get_RandomNumber(){

	uint16_t random = rand() % 100;
	return random;
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}


/*-----------------------------------------------------------*/
//#define mainQUEUE_LENGTH 100
//
//#define amber  	0
//#define green  	1
//#define red  	2
//#define blue  	3
//
//#define amber_led	LED3
//#define green_led	LED4
//#define red_led		LED5
//#define blue_led	LED6
//
//
///*
// * TODO: Implement this function for any hardware specific clock configuration
// * that was not already performed before main() was called.
// */
//static void prvSetupHardware( void );
//
///*
// * The queue send and receive tasks as described in the comments at the top of
// * this file.
// */
//static void Manager_Task( void *pvParameters );
//static void Blue_LED_Controller_Task( void *pvParameters );
//static void Green_LED_Controller_Task( void *pvParameters );
//static void Red_LED_Controller_Task( void *pvParameters );
//static void Amber_LED_Controller_Task( void *pvParameters );
//
//xQueueHandle xQueue_handle = 0;
//
//
///*-----------------------------------------------------------*/
//
//int main(void)
//{
//
//	/* Initialize LEDs */
//	STM_EVAL_LEDInit(amber_led);
//	STM_EVAL_LEDInit(green_led);
//	STM_EVAL_LEDInit(red_led);
//	STM_EVAL_LEDInit(blue_led);
//
//	/* Configure the system ready to run the demo.  The clock configuration
//	can be done here if it was not done before main() was called. */
//	prvSetupHardware();
//
//
//	/* Create the queue used by the queue send and queue receive tasks.
//	http://www.freertos.org/a00116.html */
//	xQueue_handle = xQueueCreate( 	mainQUEUE_LENGTH,		/* The number of items the queue can hold. */
//							sizeof( uint16_t ) );	/* The size of each item the queue holds. */
//
//	/* Add to the registry, for the benefit of kernel aware debugging. */
//	vQueueAddToRegistry( xQueue_handle, "MainQueue" );
//
//	xTaskCreate( Manager_Task, "Manager", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
//	xTaskCreate( Blue_LED_Controller_Task, "Blue_LED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//	xTaskCreate( Red_LED_Controller_Task, "Red_LED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//	xTaskCreate( Green_LED_Controller_Task, "Green_LED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//	xTaskCreate( Amber_LED_Controller_Task, "Amber_LED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//
//	/* Start the tasks and timer running. */
//	vTaskStartScheduler();
//
//	return 0;
//}
//
//
///*-----------------------------------------------------------*/
//
//static void Manager_Task( void *pvParameters )
//{
//	uint16_t tx_data = amber;
//
//
//	while(1)
//	{
//		// Turn on LED specified by tx_data.
//		if(tx_data == amber)
//			STM_EVAL_LEDOn(amber_led);
//		if(tx_data == green)
//			STM_EVAL_LEDOn(green_led);
//		if(tx_data == red)
//			STM_EVAL_LEDOn(red_led);
//		if(tx_data == blue)
//			STM_EVAL_LEDOn(blue_led);
//
//		// Same as xQueueSendToBack()
//		if( xQueueSend(xQueue_handle,&tx_data,1000))
//		{
//			printf("Manager: %u ON!\n", tx_data);
//			if(++tx_data == 4)
//				tx_data = 0;
//			vTaskDelay(1000);
//		}
//		else
//		{
//			printf("Manager Failed!\n");
//		}
//	}
//}
//
///*-----------------------------------------------------------*/
//
//static void Blue_LED_Controller_Task( void *pvParameters )
//{
//	uint16_t rx_data;
//	while(1)
//	{
//		if(xQueueReceive(xQueue_handle, &rx_data, 500))
//		{
//			if(rx_data == blue)
//			{
//				vTaskDelay(250);
//				STM_EVAL_LEDOff(blue_led);
//				printf("Blue Off.\n");
//			}
//			else
//			{
//				if( xQueueSend(xQueue_handle,&rx_data,1000))
//					{
//						printf("BlueTask GRP (%u).\n", rx_data); // Got wwrong Package
//						vTaskDelay(500);
//					}
//			}
//		}
//	}
//}
//
//
///*-----------------------------------------------------------*/
//
//static void Green_LED_Controller_Task( void *pvParameters )
//{
//	uint16_t rx_data;
//	while(1)
//	{
//		if(xQueueReceive(xQueue_handle, &rx_data, 500))
//		{
//			if(rx_data == green)
//			{
//				vTaskDelay(250);
//				STM_EVAL_LEDOff(green_led);
//				printf("Green Off.\n");
//			}
//			else
//			{
//				if( xQueueSend(xQueue_handle,&rx_data,1000))
//					{
//						printf("GreenTask GRP (%u).\n", rx_data); // Got wrong Package
//						vTaskDelay(500);
//					}
//			}
//		}
//	}
//}
//
///*-----------------------------------------------------------*/
//
//static void Red_LED_Controller_Task( void *pvParameters )
//{
//	uint16_t rx_data;
//	while(1)
//	{
//		if(xQueueReceive(xQueue_handle, &rx_data, 500))
//		{
//			if(rx_data == red)
//			{
//				vTaskDelay(250);
//				STM_EVAL_LEDOff(red_led);
//				printf("Red off.\n");
//			}
//			else
//			{
//				if( xQueueSend(xQueue_handle,&rx_data,1000))
//					{
//						printf("RedTask GRP (%u).\n", rx_data); // Got wrong Package
//						vTaskDelay(500);
//					}
//			}
//		}
//	}
//}
//
//
///*-----------------------------------------------------------*/
//
//static void Amber_LED_Controller_Task( void *pvParameters )
//{
//	uint16_t rx_data;
//	while(1)
//	{
//		if(xQueueReceive(xQueue_handle, &rx_data, 500))
//		{
//			if(rx_data == amber)
//			{
//				vTaskDelay(250);
//				STM_EVAL_LEDOff(amber_led);
//				printf("Amber Off.\n");
//			}
//			else
//			{
//				if( xQueueSend(xQueue_handle,&rx_data,1000))
//					{
//						printf("AmberTask GRP (%u).\n", rx_data); // Got wrong Package
//						vTaskDelay(500);
//					}
//			}
//		}
//	}
//}
//
//
///*-----------------------------------------------------------*/
//
//void vApplicationMallocFailedHook( void )
//{
//	/* The malloc failed hook is enabled by setting
//	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.
//
//	Called if a call to pvPortMalloc() fails because there is insufficient
//	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
//	internally by FreeRTOS API functions that create tasks, queues, software
//	timers, and semaphores.  The size of the FreeRTOS heap is set by the
//	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
//	for( ;; );
//}
///*-----------------------------------------------------------*/
//
//void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
//{
//	( void ) pcTaskName;
//	( void ) pxTask;
//
//	/* Run time stack overflow checking is performed if
//	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
//	function is called if a stack overflow is detected.  pxCurrentTCB can be
//	inspected in the debugger if the task name passed into this function is
//	corrupt. */
//	for( ;; );
//}
///*-----------------------------------------------------------*/
//
//void vApplicationIdleHook( void )
//{
//volatile size_t xFreeStackSpace;
//
//	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
//	FreeRTOSConfig.h.
//
//	This function is called on each cycle of the idle task.  In this case it
//	does nothing useful, other than report the amount of FreeRTOS heap that
//	remains unallocated. */
//	xFreeStackSpace = xPortGetFreeHeapSize();
//
//	if( xFreeStackSpace > 100 )
//	{
//		/* By now, the kernel has allocated everything it is going to, so
//		if there is a lot of heap remaining unallocated then
//		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
//		reduced accordingly. */
//	}
//}
///*-----------------------------------------------------------*/
//
//static void prvSetupHardware( void )
//{
//	/* Ensure all priority bits are assigned as preemption priority bits.
//	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
//	NVIC_SetPriorityGrouping( 0 );
//
//	/* TODO: Setup the clocks, etc. here, if they were not configured before
//	main() was called. */
//}








/*-----------------------------------------------------------*/

