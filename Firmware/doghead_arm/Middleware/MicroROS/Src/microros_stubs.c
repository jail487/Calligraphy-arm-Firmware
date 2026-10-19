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
 * Bare-metal atomic critical section helper.
 * Safe before RTOS scheduler starts, inside ISRs, and during task context.
 */
static inline uint32_t atomic_enter_critical(void) {
    uint32_t primask;
    __asm__ volatile ("mrs %0, primask" : "=r" (primask));
    __asm__ volatile ("cpsid i" ::: "memory");
    __asm__ volatile ("dmb" ::: "memory");
    return primask;
}

static inline void atomic_exit_critical(uint32_t primask) {
    __asm__ volatile ("dmb" ::: "memory");
    if ((primask & 1U) == 0U) {
        __asm__ volatile ("cpsie i" ::: "memory");
    }
}

uint64_t __atomic_load_8(volatile void *ptr, int memorder) {
    (void)memorder;
    uint32_t flags = atomic_enter_critical();
    uint64_t ret = *(volatile uint64_t *)ptr;
    atomic_exit_critical(flags);
    return ret;
}

void __atomic_store_8(volatile void *ptr, uint64_t val, int memorder) {
    (void)memorder;
    uint32_t flags = atomic_enter_critical();
    *(volatile uint64_t *)ptr = val;
    atomic_exit_critical(flags);
}

uint64_t __atomic_exchange_8(volatile void *ptr, uint64_t val, int memorder) {
    (void)memorder;
    uint32_t flags = atomic_enter_critical();
    uint64_t ret = *(volatile uint64_t *)ptr;
    *(volatile uint64_t *)ptr = val;
    atomic_exit_critical(flags);
    return ret;
}

uint32_t __atomic_exchange_4(volatile void *ptr, uint32_t val, int memorder) {
    (void)memorder;
    uint32_t flags = atomic_enter_critical();
    uint32_t ret = *(volatile uint32_t *)ptr;
    *(volatile uint32_t *)ptr = val;
    atomic_exit_critical(flags);
    return ret;
}

void __sync_synchronize(void) {
    __asm__ volatile ("dmb" ::: "memory");
}
