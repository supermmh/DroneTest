#pragma once 
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void DWT_Time_Init(void);
void delay_us(uint32_t us);
uint64_t Get_System_Time_ns(void);

#ifdef __cplusplus
}
#endif