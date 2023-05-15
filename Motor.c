#include "Motor.h"
#include <FreeRTOS.h>

void Motor_Init(void)
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
  while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOB));
  GPIOPinTypeGPIOOutput(GPIOB_BASE, GPIO_PIN_1 | GPIO_PIN_0);
  GPIOUnlockPin(GPIOB_BASE,GPIO_PIN_0|GPIO_PIN_1);
  GPIOPinWrite(GPIOB_BASE,GPIO_PIN_1|GPIO_PIN_0,0x00);
}

void Motor_Rotate(uint8_t direction)
{
	if (direction == MOTOR_FORWARD)
	{
		GPIOPinWrite(GPIOB_BASE,GPIO_PIN_0|GPIO_PIN_1,GPIO_PIN_1);
	}
	else if (direction == MOTOR_BACKWARD)
	{
		GPIOPinWrite(GPIOB_BASE,GPIO_PIN_0|GPIO_PIN_1,GPIO_PIN_0);
	}	
}
void Motor_Stop(void)
{
	GPIOPinWrite(GPIOB_BASE,GPIO_PIN_1|GPIO_PIN_0,0x00);
}

uint8_t get_state(void)
{
	if (GPIOPinRead(GPIOB_BASE,GPIO_PIN_1))
	{
		return MOTOR_FORWARD;
	}
	else if (GPIOPinRead(GPIOB_BASE,GPIO_PIN_0))
	{
		return MOTOR_BACKWARD;
	}
	return MOTOR_STOP;
}

void TheDelay(int ms)
{
	for (int i = 0; i < ms; i++)
	{
		for (int j = 0; j < 3180; j++);
	}
}
