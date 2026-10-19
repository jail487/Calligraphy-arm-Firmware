#include "hardware_io.h"
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

// 鎖存的事件 Mask
static volatile uint32_t g_home_event_mask = 0;

void HardwareIO_Init(void) {
    g_home_event_mask = 0;
}

bool HW_IsHomeSwitchTriggered(HardwareJoint_e joint) {
    switch (joint) {
        case HW_JOINT_1:
            return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET;
        case HW_JOINT_2:
            return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12) == GPIO_PIN_RESET;
        case HW_JOINT_3:
            return HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_11) == GPIO_PIN_RESET;
        default:
            return false;
    }
}

extern "C" void HW_OnExtInterrupt(uint16_t GPIO_Pin) {
    // 鎖存事件，由於可能是放開時的 Falling Edge，我們將在 Task 層中用 HW_IsHomeSwitchTriggered 再次確認
    if (GPIO_Pin == GPIO_PIN_10) {
        g_home_event_mask |= (1u << HW_JOINT_1);
    } else if (GPIO_Pin == GPIO_PIN_12) {
        g_home_event_mask |= (1u << HW_JOINT_2);
    } else if (GPIO_Pin == GPIO_PIN_11) {
        g_home_event_mask |= (1u << HW_JOINT_3);
    }
}

uint32_t HW_TakeHomeEvents(void) {
    taskENTER_CRITICAL();
    const uint32_t events = g_home_event_mask;
    g_home_event_mask = 0;
    taskEXIT_CRITICAL();
    
    // 過濾假中斷：即使有事件，若腳位當下沒有被 Trigger，就過濾掉
    uint32_t filtered_events = 0;
    if ((events & (1u << HW_JOINT_1)) && HW_IsHomeSwitchTriggered(HW_JOINT_1)) filtered_events |= (1u << HW_JOINT_1);
    if ((events & (1u << HW_JOINT_2)) && HW_IsHomeSwitchTriggered(HW_JOINT_2)) filtered_events |= (1u << HW_JOINT_2);
    if ((events & (1u << HW_JOINT_3)) && HW_IsHomeSwitchTriggered(HW_JOINT_3)) filtered_events |= (1u << HW_JOINT_3);

    return filtered_events;
}

uint32_t HW_GetMonotonicTimeMs(void) {
    // 若在 ISR 中不可用 xTaskGetTickCount，但此函數僅供 ControlTask / Homing 使用
    return (uint32_t)xTaskGetTickCount(); 
}
