#include "Mydelay.hpp"

void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    DWT->CYCCNT = 0;

    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void delay_us(uint64_t us)
{
    uint64_t total_ticks = us * (SystemCoreClock / 1000000);
    uint32_t last_tick = DWT->CYCCNT;
    uint32_t current_tick;
    uint32_t elapsed_ticks;
    while (total_ticks > 0)
    {
        current_tick = DWT->CYCCNT;

        elapsed_ticks = current_tick - last_tick;

        last_tick = current_tick;

        if (elapsed_ticks >= total_ticks)
        {
            total_ticks = 0; // 延时结束
        }
        else
        {
            total_ticks -= elapsed_ticks;
        }
    }
}