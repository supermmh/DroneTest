#include "Mydelay.hpp"
#include "FreeRTOS.h"
#include "task.h"
#include "tim.h"
extern TIM_HandleTypeDef htim7;

void DWT_Time_Init(void)
{
    // 彻底废弃 DWT！脱机运行时 DWT 极易被内核断钟导致死机！
}

uint64_t Get_System_Time_ns(void)
{
    uint32_t ms1, ms2, us;

    // 无锁防竞态读取：如果在读取微秒的瞬间发生了毫秒进位，引发重新读取
    do {
        ms1 = HAL_GetTick();
        us  = TIM7->CNT; // TIM7 配置的周期恰好是 0~999 微秒
        ms2 = HAL_GetTick();
    } while (ms1 != ms2);

    // ms 是毫秒级，us 是微秒级，完美拼装出 64 位纳秒时间戳！
    return ((uint64_t)ms1 * 1000000ULL) + ((uint64_t)us * 1000ULL);
}

void delay_us(uint32_t us)
{
    uint64_t start_ns = Get_System_Time_ns();
    uint64_t wait_ns  = (uint64_t)us * 1000ULL;
    uint32_t start_ms = HAL_GetTick();

    while ((Get_System_Time_ns() - start_ns) < wait_ns) {
        // 终极断路器：如果等了超过预期时间 1 秒钟，强制跳出防死机
        if (HAL_GetTick() - start_ms > (us / 1000) + 1000) break;
    }
}