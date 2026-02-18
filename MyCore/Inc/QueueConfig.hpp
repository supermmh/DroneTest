#pragma once

#include"FreeRTOS.h"
#include"queue.h"
extern QueueHandle_t SensorsDataHub;
extern QueueHandle_t VehicleStateQueue;
void SystemQueueInit(void);

