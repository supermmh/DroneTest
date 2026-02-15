#pragma once 
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// 初始化 DWT 计数器和保活机制
void DWT_Time_Init(void);

// 传统的微秒延时 (阻塞式)
void delay_us(uint32_t us);

// 🛑 【核心】获取 64位 纳秒级系统时间戳
// 特性：线程安全、中断安全、584年不溢出
uint64_t Get_System_Time_ns(void);

#ifdef __cplusplus
}
#endif