#ifndef __BUTTON_H__
#define __BUTTON_H__

#include <FreeRTOS.h>
#include <stdbool.h>
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"

void PushButton_Init(void);
void JamLock_Init();
void Limit_Init();

#endif
