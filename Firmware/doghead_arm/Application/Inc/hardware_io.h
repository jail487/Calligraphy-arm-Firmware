#ifndef HARDWARE_IO_H
#define HARDWARE_IO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// 定義關節索引
typedef enum {
    HW_JOINT_1 = 0,
    HW_JOINT_2,
    HW_JOINT_3,
    HW_JOINT_MAX
} HardwareJoint_e;

// 初始化硬體 IO 層 (若有需要特別初始化的變數)
void HardwareIO_Init(void);

// 判斷指定關節的極限開關是否被觸發
bool HW_IsHomeSwitchTriggered(HardwareJoint_e joint);

// 供 Core 中斷 (stm32f4xx_it.c 或 main.c) 呼叫，處理外部中斷 (純鎖存事件)
void HW_OnExtInterrupt(uint16_t GPIO_Pin);

// 供 Application (如 ControlTask) 取出鎖存的外部中斷事件，會自動過濾放開時的假中斷
uint32_t HW_TakeHomeEvents(void);

// 獲取當下的毫秒時間 (取代 HAL_GetTick)
uint32_t HW_GetMonotonicTimeMs(void);

#ifdef __cplusplus
}
#endif

#endif // HARDWARE_IO_H
