#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

/* 
 * Dummy implementation for Thread Local Storage pointer read.
 * Needed when linking against libraries built with thread-safe storage 
 * on bare-metal or simple RTOS environments that don't support full TLS.
 */
void * __aeabi_read_tp(void) {
    return NULL; 
}

/*
 * Implementation of 64-bit atomic operations for Cortex-M4 (32-bit).
 * Usually provided by libatomic (-latomic), but implemented here to avoid
 * changing IDE linker settings.
 * 
 * Uses FreeRTOS Critical Sections to ensure atomicity.
 */

uint64_t __atomic_load_8(volatile void *ptr, int memorder) {
    (void)memorder;
    uint64_t ret;
    vPortEnterCritical();
    ret = *(volatile uint64_t *)ptr;
    vPortExitCritical();
    return ret;
}

void __atomic_store_8(volatile void *ptr, uint64_t val, int memorder) {
    (void)memorder;
    vPortEnterCritical();
    *(volatile uint64_t *)ptr = val;
    vPortExitCritical();
}

uint64_t __atomic_exchange_8(volatile void *ptr, uint64_t val, int memorder) {
    (void)memorder;
    uint64_t ret;
    vPortEnterCritical();
    ret = *(volatile uint64_t *)ptr;
    *(volatile uint64_t *)ptr = val;
    vPortExitCritical();
    return ret;
}
