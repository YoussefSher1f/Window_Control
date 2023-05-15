#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

#define MOTOR_FORWARD     0
#define MOTOR_BACKWARD    1
#define MOTOR_STOP        2

void Motor_Init(void);
void Motor_Rotate(uint8_t direction);
void Motor_Stop(void);
uint8_t get_state();
void TheDelay(int ms);

#endif
