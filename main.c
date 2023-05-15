#include <stdint.h>
#include <stdbool.h>
#include <FreeRTOS.h>
#include <task.h>
#include "TM4C123GH6PM.h"
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "semphr.h"
#include "queue.h"
#include "Button.h"
#include "Motor.h"

//**************************************************************************//
//																																					//
// 													Declarations																		//
//																																					//
//**************************************************************************//

// Declares Queues, Semaphores, and Mutexes

QueueHandle_t Limit_Up_Q, Limit_Down_Q, Driver_Flag_Q, Lock_Flag_Q, Jam_Flag_Q;

xSemaphoreHandle Driver_Up_Semaphore;
xSemaphoreHandle Driver_Down_Semaphore;

xSemaphoreHandle Passenger_Up_Semaphore;
xSemaphoreHandle Passenger_Down_Semaphore;

SemaphoreHandle_t Jam_Semaphore;

SemaphoreHandle_t TheMutex;

//**************************************************************************//
//																																					//
// 												Interrupt Service Routines												//
//																																					//
//**************************************************************************//

// ISR
void GPIOE_Handler(void)
{
	// Define two variables to represent limit switch states
	int value1 = 1;
	int value0 = 0;
	// Used to signal the RTOS to switch context if the interrupt wakes up a higher priority task
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	
  // Check the status of GPIO pin 0
	if ((GPIOIntStatus(GPIOE_BASE, true) & 1) == 1 << 0)
	{
	// Insert the value 1 to the Limit_Up_Q and 0 to the Limit_Down_Q
		xQueueOverwriteFromISR(Limit_Down_Q, &value0, &xHigherPriorityTaskWoken);
		xQueueOverwriteFromISR(Limit_Up_Q, &value1, &xHigherPriorityTaskWoken);
	// Clear the interrupt flag on GPIO pin 0
		GPIOIntClear(GPIOE_BASE, GPIO_INT_PIN_0);
	}
	// Check the status of GPIO pin 1
	else if (GPIOIntStatus(GPIOE_BASE, true) == 1 << 1)
	{
	// Insert the value 0 to the Limit_Up_Q and 1 to the Limit_Down_Q
		xQueueOverwriteFromISR(Limit_Down_Q, &value1, &xHigherPriorityTaskWoken);
		xQueueOverwriteFromISR(Limit_Up_Q, &value0, &xHigherPriorityTaskWoken);
	// Clear the interrupt flag on GPIO pin 1
		GPIOIntClear(GPIOE_BASE, GPIO_INT_PIN_1);
	}
}

void GPIOD_Handler(void)
{
	// Used to signal the RTOS to switch context if the interrupt wakes up a higher priority task
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	// Declare variables to store the current lock flag and driver flag
	int Flag_Lock, Value_Driver = 1, Flag_Driver = 0;

	// Peek at the current lock flag and driver flag
	xQueuePeekFromISR(Lock_Flag_Q, &Flag_Lock);
	xQueuePeekFromISR(Driver_Flag_Q, &Flag_Driver);

	// Check the status of GPIO pin 2 (Driver_Up)
	if (GPIOIntStatus(GPIOD_BASE, true) == 1 << 2)
	{
		Value_Driver = 1;
	// Overwrite the driver flag queue with the new value and signal the driver up semaphore
		xQueueOverwriteFromISR(Driver_Flag_Q, &Value_Driver, &xHigherPriorityTaskWoken);
		xSemaphoreGiveFromISR(Driver_Up_Semaphore, &xHigherPriorityTaskWoken);
		GPIOIntClear(GPIOD_BASE, GPIO_INT_PIN_2);
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
	// Driver_Down
	else if (GPIOIntStatus(GPIOD_BASE, true) == 1 << 3)
	{
	// Set driver flag to 1
		Value_Driver = 1;
	// Overwrite the driver flag queue with the new value and signal the driver down semaphore
		xQueueOverwriteFromISR(Driver_Flag_Q, &Value_Driver, &xHigherPriorityTaskWoken);
		xSemaphoreGiveFromISR(Driver_Down_Semaphore, &xHigherPriorityTaskWoken);
	// Clear the interrupt on GPIOD pin 3
		GPIOIntClear(GPIOD_BASE, GPIO_INT_PIN_3);
	// End the ISR, allowing a context switch if necessary
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
	}
	// Interrupt handler for GPIOD pin 0 (Passenger Up Button)
	else if (GPIOIntStatus(GPIOD_BASE, true) == 1 << 0)
	{
	// Check if the lock flag or driver flag is set, if so, clear the interrupt
		if (Flag_Driver || Flag_Lock)
		{
			GPIOIntClear(GPIOD_BASE, GPIO_INT_PIN_0);
		}
		else
		{
	// Signal the passenger up semaphore and clear the interrupt
			xSemaphoreGiveFromISR(Passenger_Up_Semaphore, &xHigherPriorityTaskWoken);
			GPIOIntClear(GPIOD_BASE, GPIO_INT_PIN_0);
	// End the ISR, allowing a context switch if necessary
			portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
		}
	}
	// Interrupt handler for GPIOD pin 1 (Passenger Down Button)
	else if (GPIOIntStatus(GPIOD_BASE, true) == 1 << 1)
	{
	// Check if the lock flag or driver flag is set, if so, clear the interrupt
		if (Flag_Driver || Flag_Lock)
		{
			GPIOIntClear(GPIOD_BASE, GPIO_INT_PIN_1);
		}
		else
		{
	// Signal the passenger down semaphore and clear the interrupt
			xSemaphoreGiveFromISR(Passenger_Down_Semaphore, &xHigherPriorityTaskWoken);
			GPIOIntClear(GPIOD_BASE, GPIO_INT_PIN_1);
	// End the ISR, allowing a context switch if necessary
			portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
		}
	}
}
  // Interrupt handler for GPIO pins on GPIOC
void GPIOC_Handler()
{
	// Peek at the current lock flag
	int Flag_Lock;
	xQueuePeekFromISR(Lock_Flag_Q, &Flag_Lock);
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
  // Interrupt handler for GPIOC pin 4 (lock button)
	if (GPIOIntStatus(GPIOC_BASE, true) == 1 << 4)
	{
	// Clear the interrupt and toggle the lock flag
		GPIOIntClear(GPIOC_BASE, GPIO_INT_PIN_4);
		Flag_Lock ^= 0x1;
	// Overwrite the lock flag queue with the new value and signal any tasks waiting on it
		xQueueOverwriteFromISR(Lock_Flag_Q, &Flag_Lock, &xHigherPriorityTaskWoken);
	}
	// Interrupt handler for GPIOC pin 5 (Jam Sensor)
	else if (GPIOIntStatus(GPIOC_BASE, true) == 1 << 5)
	{
	// Clear the interrupt and check if the motor is currently moving forward
		GPIOIntClear(GPIOC_BASE, GPIO_INT_PIN_5);
		if (get_state() == MOTOR_FORWARD)
		{
			int Value_Jam = 1;
			xQueueOverwriteFromISR(Jam_Flag_Q, &Value_Jam, &xHigherPriorityTaskWoken);

			xSemaphoreGiveFromISR(Jam_Semaphore, &xHigherPriorityTaskWoken);
			portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
		}
	}
}

//**************************************************************************//
//																																					//
// 																Tasks																			//
//																																					//
//**************************************************************************//


  // This initialises the queues used in the system.
void Queue_Init(void *params)
{
	for (;;)
	{
	// Initialize the integer variable to 0.
		int Init_Up = 0;
	// Send the value 0 to the end of each queue.
		xQueueSendToBack(Driver_Flag_Q, &Init_Up, 0);
		xQueueSendToBack(Limit_Up_Q, &Init_Up, 0);
		xQueueSendToBack(Limit_Down_Q, &Init_Up, 0);
	  xQueueSendToBack(Jam_Flag_Q, &Init_Up, 0);
		xQueueSendToBack(Lock_Flag_Q, &Init_Up, 0);
	
	// Suspend the task, waiting for an interrupt to wake it up.
		vTaskSuspend(NULL);
	}
}

  // This task controls the Driver's Window going up.
void Driver_Up_T(void *params)
{
	// Wait for the Driver_Up_Semaphore to become available.
	xSemaphoreTake(Driver_Up_Semaphore, 0);
	for (;;)
	{
	// Wait for the Driver_Up_Semaphore to become available again.
		xSemaphoreTake(Driver_Up_Semaphore, portMAX_DELAY);
	// Acquire the mutex to prevent other tasks from accessing shared resources.
		xSemaphoreTake(TheMutex, portMAX_DELAY);
	// Initialize variables for the limit switch status, jam flag status, and driver flag status.
		int Limit_Up, Flag_Jam, Flag_Driver = 0;
	// Peek into the Jam_Flag_Q and Limit_Up_Q queues to get and check their values.
		xQueuePeek(Jam_Flag_Q, &Flag_Jam, 0);
		xQueuePeek(Limit_Up_Q, &Limit_Up, 0);
	// If the limit switch is up, set the driver flag to 0 and release the mutex.
		if (Limit_Up)
		{
			xQueueOverwrite(Driver_Flag_Q, &Flag_Driver);
			xSemaphoreGive(TheMutex);
		}
		else
		{
	// Stop the motor and rotate it.
			Motor_Stop();
			Motor_Rotate(MOTOR_FORWARD);
	// Set the limit down value to 0.
			int value = 0;
			xQueueOverwrite(Limit_Down_Q, &value);
	// Delay for 500 milliseconds.
			TheDelay(500);
	// Check if the manual switch is pressed.
			if (GPIOPinRead(GPIOD_BASE, GPIO_PIN_2))
			{
	// If MANUAL, 
	// Keep running the motor until the limit switch is reached or jam is detected.
				while (GPIOPinRead(GPIOD_BASE, GPIO_PIN_2) && !Limit_Up && !Flag_Jam)
				{
					xQueuePeek(Limit_Up_Q, &Limit_Up, 0);
					xQueuePeek(Jam_Flag_Q, &Flag_Jam, 0);
				}
			}
			else
			{
	// If AUTOMATIC, 
	// Keep running the motor until the limit switch is reached or jam is detected or the window down is pressed.
				while (!Limit_Up && !Flag_Jam)
				{
					xQueuePeek(Limit_Up_Q, &Limit_Up, 0);
					xQueuePeek(Jam_Flag_Q, &Flag_Jam, 0);

					int WindowDown = GPIOPinRead(GPIOD_BASE, GPIO_PIN_3);
					if (WindowDown)
						break;
				}
			}
	// Stops the motor, the variables Value_Jam and Flag_Driver are set to zero.
			Motor_Stop();
			int Value_Jam = 0;
  // Writes the values of Value_Jam and Flag_Driver to the Jam_Flag_Q and Driver_Flag_Q queues, respectively.
			xQueueOverwrite(Driver_Flag_Q, &Flag_Driver);
			xQueueOverwrite(Jam_Flag_Q, &Value_Jam);
  // The Mutex is released.
			xSemaphoreGive(TheMutex);
		}
	}
}

  // This task controls the Driver's Window going down.
void Driver_Down_T(void *params)
{
	xSemaphoreTake(Driver_Down_Semaphore, 0);
	for (;;)
	{
		xSemaphoreTake(Driver_Down_Semaphore, portMAX_DELAY);
		xSemaphoreTake(TheMutex, portMAX_DELAY);

		int Limit_Down, Flag_Driver = 0;
		xQueuePeek(Limit_Down_Q, &Limit_Down, 0);

		if (Limit_Down)
		{
			xQueueOverwrite(Driver_Flag_Q, &Flag_Driver);
			xSemaphoreGive(TheMutex);
		}
		else
		{
			Motor_Stop();
			Motor_Rotate(MOTOR_BACKWARD);
			int value = 0;
			xQueueOverwrite(Limit_Up_Q, &value);
			TheDelay(500);
			if (GPIOPinRead(GPIOD_BASE, GPIO_PIN_3))
			{
  // If MANUAL
				while (GPIOPinRead(GPIOD_BASE, GPIO_PIN_3) && !Limit_Down)
				{
					xQueuePeek(Limit_Down_Q, &Limit_Down, 0);
				}
			}
			else
			{
  // If AUTOMATIC
				while (!Limit_Down)
				{
					xQueuePeek(Limit_Down_Q, &Limit_Down, 0);
					int WindowUp = GPIOPinRead(GPIOD_BASE, GPIO_PIN_2);
					if (WindowUp)
						break;
				}
			}
			Motor_Stop();
			xQueueOverwrite(Driver_Flag_Q, &Flag_Driver);
			xSemaphoreGive(TheMutex);
		}
	}
}
  // This task controls the Passenger's Window going up.
void Passenger_Up_T(void *params)
{
	// Take the Passenger Up Semaphore to start the task
	xSemaphoreTake(Passenger_Up_Semaphore, 0);
	for (;;)
	{
	// Wait for Passenger Up Semaphore
		xSemaphoreTake(Passenger_Up_Semaphore, portMAX_DELAY);
	// Take the Mutex to synchronize shared resources
		xSemaphoreTake(TheMutex, portMAX_DELAY);
  // Declare and initialise variables for Flags and Limits
		int Flag_Driver, Limit_Up, Flag_Jam, Flag_Lock;
		xQueuePeek(Driver_Flag_Q, &Flag_Driver, 0);
		xQueuePeek(Limit_Up_Q, &Limit_Up, 0);
		xQueuePeek(Jam_Flag_Q, &Flag_Jam, 0);
		xQueuePeek(Lock_Flag_Q, &Flag_Lock, 0);
  // Check if the limit is reached
		if (Limit_Up)
		{
	// Give the Mutex and continue
			xSemaphoreGive(TheMutex);
		}
		else
		{
	// Stop the motor, rotate it in the forward direction and update the Down Limit Queue
			Motor_Stop();
			Motor_Rotate(MOTOR_FORWARD);

			int value = 0;
			xQueueOverwrite(Limit_Down_Q, &value);
			TheDelay(500);
			if (GPIOPinRead(GPIOD_BASE, GPIO_PIN_0))
			{
				// If MANUAL,
				// Keep moving the motor until the limit is reached or any obstacle (Driver, Lock or Jam)
				while (GPIOPinRead(GPIOD_BASE, GPIO_PIN_0) && !Flag_Driver && !Limit_Up && !Flag_Jam && !Flag_Lock)
				{
					xQueuePeek(Driver_Flag_Q, &Flag_Driver, 0);
					xQueuePeek(Limit_Up_Q, &Limit_Up, 0);
					xQueuePeek(Jam_Flag_Q, &Flag_Jam, 0);
					xQueuePeek(Lock_Flag_Q, &Flag_Lock, 0);
				}
			}
			else
			{
				// If AUTOMATIC,
				// Keep moving the motor until the limit is reached or the window is detected going down
				while (!Flag_Driver && !Limit_Up && !Flag_Jam && !Flag_Lock)
				{
					xQueuePeek(Driver_Flag_Q, &Flag_Driver, 0);
					xQueuePeek(Limit_Up_Q, &Limit_Up, 0);
					xQueuePeek(Jam_Flag_Q, &Flag_Jam, 0);
					xQueuePeek(Lock_Flag_Q, &Flag_Lock, 0);

					int WindowDown = GPIOPinRead(GPIOD_BASE, GPIO_PIN_1);
					if (WindowDown)
						break;
				}
			}
			Motor_Stop();
			int Value_Jam = 0;
			xQueueOverwrite(Jam_Flag_Q, &Value_Jam);
  // Give the Mutex and continue
			xSemaphoreGive(TheMutex);
		}
	}
}

  // This task controls the Passenger's Window going down.
void Passenger_Down_T(void *params)
{
	xSemaphoreTake(Passenger_Down_Semaphore, 0);
	for (;;)
	{
		xSemaphoreTake(Passenger_Down_Semaphore, portMAX_DELAY);
		xSemaphoreTake(TheMutex, portMAX_DELAY);

		int Flag_Driver, Limit_Down, Flag_Lock;
    xQueuePeek(Driver_Flag_Q, &Flag_Driver, 0);
		xQueuePeek(Limit_Down_Q, &Limit_Down, 0);
		xQueuePeek(Lock_Flag_Q, &Flag_Lock, 0);

		if (Limit_Down)
		{
			xSemaphoreGive(TheMutex);
		}
		else
		{
			Motor_Stop();
			Motor_Rotate(MOTOR_BACKWARD);
			int value = 0;
			xQueueOverwrite(Limit_Up_Q, &value);
			TheDelay(500);
			if (GPIOPinRead(GPIOD_BASE, GPIO_PIN_1))
			{
	// If MANUAL
				while (GPIOPinRead(GPIOD_BASE, GPIO_PIN_1) && !Flag_Driver && !Limit_Down && !Flag_Lock)
				{
					xQueuePeek(Driver_Flag_Q, &Flag_Driver, 0);
					xQueuePeek(Limit_Down_Q, &Limit_Down, 0);
					xQueuePeek(Lock_Flag_Q, &Flag_Lock, 0);
				}
			}
			else
			{
	// If AUTOMATIC
				while (!Flag_Driver && !Limit_Down && !Flag_Lock)
				{
					xQueuePeek(Driver_Flag_Q, &Flag_Driver, 0);
					xQueuePeek(Limit_Down_Q, &Limit_Down, 0);
					xQueuePeek(Lock_Flag_Q, &Flag_Lock, 0);

					int AlreadyUp = GPIOPinRead(GPIOD_BASE, GPIO_PIN_0);
					if (AlreadyUp)
						break;
				}
			}
			Motor_Stop();
			xSemaphoreGive(TheMutex);
		}
	}
}

  // This task controls the Jam, to protect the driver and the passenger.
void Jamming(void *params)
{
	// Take the Jam_Semaphore, but don't block if it's not available
	xSemaphoreTake(Jam_Semaphore, 0);
	for (;;)
	{
	// Wait for the Jam_Semaphore indefinitely (Blocks)
		xSemaphoreTake(Jam_Semaphore, portMAX_DELAY);
	// Take the mutex to protect shared resources
		xSemaphoreTake(TheMutex, portMAX_DELAY);
	// Stop the motor to avoid any further damage
		Motor_Stop();
		TheDelay(500);
	// Rotate the motor backward to release the jam
		Motor_Rotate(MOTOR_BACKWARD);
		TheDelay(500);
	// Stop the motor again
		Motor_Stop();
	// Release the mutex
		xSemaphoreGive(TheMutex);
	}
}

int main()
{
	// Enable interrupts
	IntMasterEnable();
	// Create semaphores to be used for synchronization between tasks
	vSemaphoreCreateBinary(Driver_Up_Semaphore);
	vSemaphoreCreateBinary(Driver_Down_Semaphore);
	vSemaphoreCreateBinary(Passenger_Up_Semaphore);
	vSemaphoreCreateBinary(Passenger_Down_Semaphore);
  vSemaphoreCreateBinary(Jam_Semaphore);
	// Create a mutex to be used for mutual exclusion between tasks
	TheMutex = xSemaphoreCreateMutex();

	// Create queues for inter-task communication
	Driver_Flag_Q = xQueueCreate(1, sizeof(int));
	Limit_Up_Q = xQueueCreate(1, sizeof(int));
	Limit_Down_Q = xQueueCreate(1, sizeof(int));
	Jam_Flag_Q = xQueueCreate(1, sizeof(int));
	Lock_Flag_Q = xQueueCreate(1, sizeof(int));

	// Initialise motor
	Motor_Init();
	
	// Initialise limit switches
	Limit_Init();
	GPIOIntRegister(GPIOE_BASE, GPIOE_Handler);
	IntPrioritySet(INT_GPIOE, 0xA0);
	
	// Initialise jamming detection and lock status
	JamLock_Init();
	GPIOIntRegister(GPIOC_BASE, GPIOC_Handler);
	IntPrioritySet(INT_GPIOC, 0xA0);
	
	// Initialise push buttons
	PushButton_Init();
	GPIOIntRegister(GPIOD_BASE, GPIOD_Handler);
	IntPrioritySet(INT_GPIOD, 0xE0);

  // Create tasks with specified parameters
	xTaskCreate(Queue_Init, "Q_I", 240, NULL, 4, NULL);
	xTaskCreate(Jamming, "J", 240, NULL, 4, NULL);
	xTaskCreate(Driver_Up_T, "D_U", 240, NULL, 3, NULL);
	xTaskCreate(Driver_Down_T, "D_D", 240, NULL, 3, NULL);
	xTaskCreate(Passenger_Up_T, "P_U", 240, NULL, 2, NULL);
	xTaskCreate(Passenger_Down_T, "P_D", 240, NULL, 2, NULL);

  // Start the scheduler
	vTaskStartScheduler();

	// The following line should never be reached.
	// Failure to allocate enough memory from the heap would be one reason.
	for (;;);
}
  // Idle hook
void vApplicationIdleHook(void)
{

}
