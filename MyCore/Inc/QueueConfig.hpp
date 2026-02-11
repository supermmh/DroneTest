#pragma once

#include"FreeRTOS.h"
#include"queue.h"
extern QueueHandle_t SensorsDataHub;

void SystemQueueInit(void);

