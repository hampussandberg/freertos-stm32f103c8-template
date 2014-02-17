/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* STM32 Library includes. */
#include "stm32f10x.h"

#include <stdio.h>

#include <string.h>

/* Priorities at which the tasks are created. */
#define mainQUEUE_RECEIVE_TASK_PRIORITY		(tskIDLE_PRIORITY + 2)
#define	mainQUEUE_SEND_TASK_PRIORITY		(tskIDLE_PRIORITY + 1)

/* The rate at which data is sent to the queue, specified in milliseconds, and
converted to ticks using the portTICK_RATE_MS constant. */
#define mainQUEUE_SEND_FREQUENCY_MS			( 1000 / portTICK_RATE_MS )

/* The number of items the queue can hold.  This is 1 as the receive task
will remove items as they are added, meaning the send task should always find
the queue empty. */
#define mainQUEUE_LENGTH					( 1 )


/* ----- LED definitions --------------------------------------------------- */

#define LED_1	GPIO_Pin_10
#define LED_2	GPIO_Pin_11
#define LED_3	GPIO_Pin_12

/*-----------------------------------------------------------*/

/*
 * Setup the NVIC, LED outputs, and button inputs.
 */
static void prvSetupHardware( void );

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvQueueReceiveTask( void *pvParameters );
static void prvQueueSendTask( void *pvParameters );

/*
 * The LED timer callback function.  This does nothing but switch the red LED
 * off.
 */
static void vLEDTimerCallback( xTimerHandle xTimer );

/*-----------------------------------------------------------*/

/* The queue used by both tasks. */
static xQueueHandle xQueue = NULL;

/* The LED software timer.  This uses vLEDTimerCallback() as its callback
 * function.
 */
static xTimerHandle xLEDTimer = NULL;

/*-----------------------------------------------------------*/


int main()
{
	/* Configure the NVIC, LED outputs and button inputs. */
	prvSetupHardware();

	/* Create the queue. */
	xQueue = xQueueCreate(mainQUEUE_LENGTH, sizeof(unsigned long));

	if (xQueue != NULL)
	{
		/* Start the two tasks as described in the comments at the top of this file. */
		xTaskCreate(prvQueueReceiveTask,
					(signed char *) "RX",
					configMINIMAL_STACK_SIZE,
					NULL,
					mainQUEUE_RECEIVE_TASK_PRIORITY,
					NULL);
		xTaskCreate(prvQueueSendTask,
					(signed char *) "TX",
					configMINIMAL_STACK_SIZE,
					NULL,
					mainQUEUE_SEND_TASK_PRIORITY,
					NULL);

		/* Create the software timer that is responsible for turning off the LED
		if the button is not pushed within 5000ms, as described at the top of
		this file. */
		xLEDTimer = xTimerCreate((const signed char *) "LEDTimer", /* A text name, purely to help debugging. */
								(5000 / portTICK_RATE_MS),			/* The timer period, in this case 5000ms (5s). */
								pdFALSE,							/* This is a one shot timer, so xAutoReload is set to pdFALSE. */
								(void*) 0,							/* The ID is not used, so can be set to anything. */
								vLEDTimerCallback					/* The callback function that switches the LED off. */
								);

		/* Start the tasks and timer running. */
		vTaskStartScheduler();
	}

	/* If all is well, the scheduler will now be running, and the following line
	will never be reached.  If the following line does execute, then there was
	insufficient FreeRTOS heap memory available for the idle and/or timer tasks
	to be created.  See the memory management section on the FreeRTOS web site
	for more details. */
	while (1);
}

/*-----------------------------------------------------------*/

static void vLEDTimerCallback( xTimerHandle xTimer )
{
	/* The timer has expired - so no button pushes have occurred in the last
	five seconds - turn the LEDs off.  NOTE - accessing the LED port should use
	a critical section because it is accessed from multiple tasks, and the
	button interrupt - in this trivial case, for simplicity, the critical
	section is omitted. */
	GPIO_SetBits(GPIOA, LED_1 | LED_2 | LED_3);
}

/*-----------------------------------------------------------*/

static void prvQueueSendTask( void *pvParameters )
{
	portTickType xNextWakeTime;
	const unsigned long ulValueToSend = 100UL;

	/* Initialize xNextWakeTime - this only needs to be done once. */
	xNextWakeTime = xTaskGetTickCount();

	while (1)
	{
		/* Place this task in the blocked state until it is time to run again.
		The block time is specified in ticks, the constant used converts ticks
		to ms.  While in the Blocked state this task will not consume any CPU
		time. */
		vTaskDelayUntil(&xNextWakeTime, mainQUEUE_SEND_FREQUENCY_MS);

		/* Send to the queue - causing the queue receive task to unblock and
		toggle an LED.  0 is used as the block time so the sending operation
		will not block - it shouldn't need to block as the queue should always
		be empty at this point in the code. */
		xQueueSend(xQueue, &ulValueToSend, 0);
	}
}
/*-----------------------------------------------------------*/

static void prvQueueReceiveTask( void *pvParameters )
{
	unsigned long ulReceivedValue;

	while (1)
	{
		/* Wait until something arrives in the queue - this task will block
		indefinitely provided INCLUDE_vTaskSuspend is set to 1 in
		FreeRTOSConfig.h. */
		xQueueReceive(xQueue, &ulReceivedValue, portMAX_DELAY);

		/*  To get here something must have been received from the queue, but
		is it the expected value?  If it is, toggle the LEDs. */
		if (ulReceivedValue == 100UL)
		{
			/* NOTE - accessing the LED port should use a critical section
			because it is accessed from multiple tasks, and the button interrupt
			- in this trivial case, for simplicity, the critical section is
			omitted. */
			GPIOA->ODR ^= LED_1 | LED_2 | LED_3;
		}
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure that all 4 interrupt priority bits are used as the pre-emption priority. */
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Set up the LED outputs */
	// Enable clock for GPIOA
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin   = LED_1 | LED_2 | LED_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_SetBits(GPIOA, LED_1 | LED_2 | LED_3);
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	while (1);
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* This function is called on each cycle of the idle task.  In this case it
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

#ifdef DEBUG
/* The fault handler implementation calls a function called
prvGetRegistersFromStack(). */
void HardFault_Handler(void)
{
    __asm volatile
    (
        " tst lr, #4                                                \n"
        " ite eq                                                    \n"
        " mrseq r0, msp                                             \n"
        " mrsne r0, psp                                             \n"
        " ldr r1, [r0, #24]                                         \n"
        " ldr r2, handler2_address_const                            \n"
        " bx r2                                                     \n"
        " handler2_address_const: .word prvGetRegistersFromStack    \n"
    );
}

void prvGetRegistersFromStack( uint32_t *pulFaultStackAddress )
{
	/* These are volatile to try and prevent the compiler/linker optimising them
	away as the variables never actually get used.  If the debugger won't show the
	values of the variables, make them global my moving their declaration outside
	of this function. */
	volatile uint32_t r0;
	volatile uint32_t r1;
	volatile uint32_t r2;
	volatile uint32_t r3;
	volatile uint32_t r12;
	volatile uint32_t lr; /* Link register. */
	volatile uint32_t pc; /* Program counter. */
	volatile uint32_t psr;/* Program status register. */

    r0 = pulFaultStackAddress[0];
    r1 = pulFaultStackAddress[1];
    r2 = pulFaultStackAddress[2];
    r3 = pulFaultStackAddress[3];

    r12 = pulFaultStackAddress[4];
    lr = pulFaultStackAddress[5];
    pc = pulFaultStackAddress[6];
    psr = pulFaultStackAddress[7];

    /* When the following line is hit, the variables contain the register values. */
    while (1);

    /* These lines help prevent getting warnings from compiler about unused variables */
    r0 = r1 = r2 = r3 = r12 = lr = pc = psr = 0;
    r0++;
}

#endif // #ifdef DEBUG
