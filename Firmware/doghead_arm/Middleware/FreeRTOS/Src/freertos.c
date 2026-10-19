/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications (Historical / Inactive)
  * Note               : Excluded from active build. Core/Src/freertos.c is the
  *                      sole active FreeRTOS owner.
  ******************************************************************************
  */
/* USER CODE END Header */

#if 0 /* Excluded from build. Active implementation restored in Core/Src/freertos.c */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mainpp.h"             // 用於呼叫 C++ 的 Robot_Loop
#include "microros_app.h"       // micro-ROS 應用層
#include "robot_interface.h"  // 取代原有的 motor_config.h，實作硬體解耦
#include "test_scenarios.h"     // 用於診斷輸出測試模式變數
#include "runtime_monitor.h"    // WCET 量測
#include "safety_manager.h"     // 安全管理 (Fault 觸發)
#include <stdio.h>
#include "cmsis_os2.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CONTROL_TASK_PRIORITY ((UBaseType_t)osPriorityRealtime)
#define COMM_TASK_PRIORITY    ((UBaseType_t)osPriorityAboveNormal)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void MX_FREERTOS_Init(void); // 宣告初始化函式
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// ==========================================================
// FreeRTOS 任務定義
// ==========================================================

/**
 * @brief 控制任務：高優先級，由 TIM6 硬體中斷觸發 (1kHz)
 * @note 負責直接控制馬達轉速（不含運動學）
 */
void ControlTask(void *argument) {
  // 在任務開始執行後，才啟動 TIM6 中斷，確保 xControlTaskHandle 已就緒且 OS 已運行
  HAL_TIM_Base_Start_IT(&htim6);

  for (;;) {
    // 等待硬體 TIM6 中斷的通知 (阻塞等待，極低延遲)
    uint32_t pending = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    uint32_t start = DWT->CYCCNT;

    // 呼叫 Application 層的控制主 entry
    Robot_ControlTask_Tick(0.001f);
    
    uint32_t cycles = DWT->CYCCNT - start;
    
    // STM32F446RE 預設 180/168/84MHz, SystemCoreClock/1000 剛好是 1ms 的 cycles
    bool missed = (pending != 1U) || (cycles >= SystemCoreClock / 1000U);
    
    RuntimeMonitor_RecordSample(cycles, missed);
    
    if (RuntimeMonitor_GetConsecutiveMisses() >= 3U) {
        RobotSafety_TriggerFault(ROBOT_FAULT_DEADLINE_MISS);
    }
  }
}

/**
 * @brief 通訊任務：低於控制任務，50Hz (20ms 週期)
 * @note 負責處理 micro-ROS 通訊、診斷輸出
 */
void CommTask(void *argument) {
  const TickType_t xFrequency =
      pdMS_TO_TICKS(20); // 20ms = 50Hz（micro-ROS 需要較高頻率）

  // 等待 1 秒讓系統與周邊穩定
  vTaskDelay(pdMS_TO_TICKS(1000));

  // 初始化 micro-ROS 傳輸層
  // printf("[CommTask] 啟動 micro-ROS 狀態機與 DMA Transport...\r\n");
  MicroROS_Init();
  TickType_t xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    // ========== micro-ROS 狀態機輪詢 (50Hz) ==========
    MicroROS_Spin();

    if (MicroROS_IsInitialized()) {
      RobotTelemetry_t telem;
      RobotInterface_GetTelemetry(&telem);
      MicroROS_PublishMcuState(&telem);
      MicroROS_PublishJointStates(&telem);

      static TickType_t last_diag_time = 0;
      TickType_t now_ticks = xTaskGetTickCount();
      if ((now_ticks - last_diag_time) >= pdMS_TO_TICKS(1000)) {
        last_diag_time = now_ticks;
        RobotDiagnostics_t diag;
        RobotInterface_GetDiagnostics(&diag);
        MicroROS_PublishMcuDiagnostics(&diag);
      }
    }

    TickType_t now = xTaskGetTickCount();
    if ((TickType_t)(now - xLastWakeTime) >= xFrequency) {
      xLastWakeTime = now;
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
 * @brief 在 main() 的 RTOS_THREADS 區段呼叫此函式來建立任務
 */
void MX_FREERTOS_Init(void) {
  // 引用外部宣告的任務句柄（定義在 main.c）
  extern TaskHandle_t xControlTaskHandle;

  // 建立控制任務（高於 CMSIS defaultTask = osPriorityNormal）
  if (xTaskCreate(ControlTask,        // 任務函式
                  "ControlTask",      // 任務名稱 (用於除錯)
                  512,                // Stack 大小加大到 512 words (2KB) 避免溢位
                  NULL,               // 任務參數
                  CONTROL_TASK_PRIORITY,
                  &xControlTaskHandle // 任務控制代碼
      ) != pdPASS) {
    Error_Handler();
  }

  // 建立通訊任務（高於 defaultTask，低於控制任務）
  // micro-ROS STM32 utility README 要求 CommTask 堆疊必須 > 10 kB
  TaskHandle_t commTaskHandle = NULL;
  if (xTaskCreate(
        CommTask,
        "CommTask",
        3072,                  // Stack 大小 3072 words (12 kB / 3072 32-bit words) 給 micro-ROS 和 printf
        NULL,
        COMM_TASK_PRIORITY,
        &commTaskHandle
      ) != pdPASS) {
    Error_Handler();
  }

  // defaultTask 已經在 main.c 中由 CubeMX 自動建立
  // 其優先級為 osPriorityNormal (24)，低於我們自訂的任務
}

/**
 * @brief FreeRTOS 堆疊溢位 Hook 函式
 * @note 當 configCHECK_FOR_STACK_OVERFLOW > 0 且檢測到 Task 堆疊溢位時觸發
 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
  (void)xTask;
  (void)pcTaskName;
  RobotSafety_TriggerFault(ROBOT_FAULT_DEADLINE_MISS);
  taskDISABLE_INTERRUPTS();
  for (;;) {
  }
}

/* USER CODE END Application */

#endif /* Excluded from build */
