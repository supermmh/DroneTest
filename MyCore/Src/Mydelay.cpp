#include "Mydelay.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h" // 引入软件定时器


static volatile uint32_t s_last_cyccnt = 0;
static volatile uint64_t s_total_cycles = 0;

static void TimeBase_KeepAlive_Callback(TimerHandle_t xTimer) {
    (void)xTimer;
    Get_System_Time_ns(); // 仅仅调用一次，利用其内部机制更新 s_total_cycles
}


void DWT_Time_Init(void) {
    // 1. 启用 DWT 硬件
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    
    // 初始化静态变量
    s_last_cyccnt = DWT->CYCCNT;
    s_total_cycles = 0;

    TimerHandle_t hTimer = xTimerCreate(
        "TimeGuard",             // 名字
        pdMS_TO_TICKS(4000),     // 周期 4秒 (远小于 8.9秒溢出时间)
        pdTRUE,                  // 自动重载
        (void*)0,                // ID
        TimeBase_KeepAlive_Callback // 回调函数
    );

    if (hTimer != NULL) {
        xTimerStart(hTimer, 0); // 启动定时器
    }
}

void delay_us(uint32_t us) {
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (SystemCoreClock / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

uint64_t Get_System_Time_ns(void) {

    uint32_t isr_mask = portSET_INTERRUPT_MASK_FROM_ISR();

    uint32_t current_cyccnt = DWT->CYCCNT;

    uint32_t delta = current_cyccnt - s_last_cyccnt;
    
    s_total_cycles += delta;
    s_last_cyccnt = current_cyccnt;

    uint64_t cycles_copy = s_total_cycles;

    portCLEAR_INTERRUPT_MASK_FROM_ISR(isr_mask);
    
    uint64_t seconds = cycles_copy / SystemCoreClock;         // 整数秒
    uint64_t fraction = cycles_copy % SystemCoreClock;        // 不足一秒的周期数

    uint64_t time_ns = (seconds * 1000000000ULL) + 
                       ((fraction * 1000000000ULL) / SystemCoreClock);

    return time_ns;
}