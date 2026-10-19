#ifndef FREERTOS_RUNTIME_H
#define FREERTOS_RUNTIME_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint32_t Robot_GetControlStackHighWatermark(void);
uint32_t Robot_GetCommStackHighWatermark(void);

#ifdef __cplusplus
}
#endif

#endif
