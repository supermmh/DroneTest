#include "DTCMinit.hpp"
#include <stdint.h>

extern uint32_t _sidtcm_data; // Flash中的源地址
extern uint32_t _sdtcm_data;  // DTCM中的目标起始地址
extern uint32_t _edtcm_data;  // DTCM中的目标结束地址

extern uint32_t _sdtcm_bss; // DTCM BSS段起始地址 (未初始化变量)
extern uint32_t _edtcm_bss; // DTCM BSS段结束地址

void System_DTCM_Init(void)
{
    uint32_t *src, *dst;

    src = &_sidtcm_data;
    dst = &_sdtcm_data;
    while (dst < &_edtcm_data)
    {
        *dst++ = *src++;
    }

    dst = &_sdtcm_bss;
    while (dst < &_edtcm_bss)
    {
        *dst++ = 0;
    }
}