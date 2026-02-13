#include "DTCMinit.h"
#include <stdint.h>

extern uint32_t _sidtcm_data; // Load Address (在 Flash 中)
extern uint32_t _sdtcm_data;  // Start Address (在 DTCM 中)
extern uint32_t _edtcm_data;  // End Address (在 DTCM 中)

// .dtcm_bss 相关符号
extern uint32_t _sdtcm_bss; // Start Address
extern uint32_t _edtcm_bss; // End Address

void System_DTCM_Init(void)
{
    uint32_t *src = &_sidtcm_data;
    uint32_t *dst = &_sdtcm_data;
    uint32_t *end = &_edtcm_data;

    if (src != dst)
    {
        while (dst < end)
        {
            *dst++ = *src++;
        }
    }

    uint32_t *bss_start = &_sdtcm_bss;
    uint32_t *bss_end = &_edtcm_bss;

    while (bss_start < bss_end)
    {
        *bss_start++ = 0;
    }
}