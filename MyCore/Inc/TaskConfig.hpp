#pragma once
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void Bridge_System_Init(void);
extern void* T0_FastLoop_Handle_Ptr; 
extern void* T2_Comms_Handle_Ptr;
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
#include "FreeRTOS.h"
#include "task.h"

extern TaskHandle_t T0_FastLoop_Handle;
extern TaskHandle_t T1_NavLoop_Handle;
extern TaskHandle_t T2_Comms_Handle;
extern TaskHandle_t T3_Monitor_Handle;

void System_Init_Task(void *argument); 
void T0_FastLoop_Entry(void *argument);
void T1_NavLoop_Entry(void *argument);
void T2_Comms_Entry(void *argument);
void T3_Monitor_Entry(void *argument);
#endif