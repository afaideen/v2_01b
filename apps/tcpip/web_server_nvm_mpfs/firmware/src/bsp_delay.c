
//#include<machine/types.h>
#include "system_config.h"
#define READ_CORE_TIMER()                 _CP0_GET_COUNT()          // Read the MIPS Core Timer
 
void BSP_DelayUs(uint16_t microseconds)
{
    uint32_t time;
    
    time = READ_CORE_TIMER(); // Read Core Timer    
    time += (SYS_CLK_FREQ / 2 / 1000000) * microseconds; // calc the Stop Time    
    while ((int32_t)(time - READ_CORE_TIMER()) > 0){};    
}

void BSP_DelayMs(uint16_t miliseconds)
{
    uint32_t time;
    
    time = READ_CORE_TIMER(); // Read Core Timer    
    time += (SYS_CLK_FREQ / 2 / 1000000) * miliseconds * 1000; // calc the Stop Time    
    while ((int32_t)(time - READ_CORE_TIMER()) > 0){};    
}

void DelayMs(uint16_t ms)
{
    BSP_DelayMs(ms);
}

