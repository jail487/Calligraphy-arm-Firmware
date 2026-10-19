#include "runtime_monitor.h"
#include "stm32f4xx_hal.h"

static volatile uint32_t max_cycles = 0;
static volatile uint32_t last_cycles = 0;
static volatile uint32_t consecutive_misses = 0;
static volatile uint32_t total_misses = 0;
static volatile uint32_t control_heartbeat = 0;

void RuntimeMonitor_Init(void) {
    max_cycles = 0U;
    last_cycles = 0U;
    consecutive_misses = 0U;
    total_misses = 0U;
    control_heartbeat = 0U;
    // 啟用 DWT Cycle Counter
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();
}

void RuntimeMonitor_RecordSample(uint32_t cycles, bool missed) {
    last_cycles = cycles;
    if (cycles > max_cycles) {
        max_cycles = cycles;
    }
    
    if (missed) {
        consecutive_misses++;
        total_misses++;
    } else {
        consecutive_misses = 0;
        control_heartbeat++;
    }
}

uint32_t RuntimeMonitor_GetMaxCycles(void) {
    return max_cycles;
}

uint32_t RuntimeMonitor_GetLastCycles(void) {
    return last_cycles;
}

uint32_t RuntimeMonitor_GetConsecutiveMisses(void) {
    return consecutive_misses;
}

uint32_t RuntimeMonitor_GetTotalMisses(void) {
    return total_misses;
}

uint32_t RuntimeMonitor_GetControlHeartbeat(void) {
    return control_heartbeat;
}
