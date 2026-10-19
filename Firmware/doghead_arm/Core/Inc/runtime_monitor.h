#ifndef RUNTIME_MONITOR_H
#define RUNTIME_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

// 初始化 DWT (Data Watchpoint and Trace) Cycle Counter
void RuntimeMonitor_Init(void);

// 紀錄一次控制迴圈的取樣 (包含執行週期數，以及是否發生 backlog miss)
void RuntimeMonitor_RecordSample(uint32_t cycles, bool missed);

// Maximum and most recent measured control-task execution time in CPU cycles.
uint32_t RuntimeMonitor_GetMaxCycles(void);
uint32_t RuntimeMonitor_GetLastCycles(void);

// 獲取目前連續 Miss 的次數，以便 Safety Manager 觸發 Fault
uint32_t RuntimeMonitor_GetConsecutiveMisses(void);

// 獲取總 Miss 次數
uint32_t RuntimeMonitor_GetTotalMisses(void);

// 獲取成功執行的 Heartbeat 計數 (供未來的 IWDG 餵狗使用)
uint32_t RuntimeMonitor_GetControlHeartbeat(void);

#ifdef __cplusplus
}
#endif

#endif // RUNTIME_MONITOR_H
