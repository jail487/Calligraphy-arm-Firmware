/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mainpp.h"             // 用於呼叫 C++ 的 Robot_Loop
#include "microros_app.h"       // micro-ROS 應用層
#include "motor_config.h"       // 用於存取馬達物件實體與 API
#include "test_scenarios.h"     // 用於診斷輸出測試模式變數
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // 呼叫 Application 層的控制主 entry
    Robot_ControlTask_Tick(0.001f);
  }
}

/**
 * @brief 通訊任務：低優先級，10Hz (100ms 週期)
 * @note 負責處理 micro-ROS 通訊、診斷輸出
 */
void CommTask(void *argument) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency =
      pdMS_TO_TICKS(20); // 20ms = 50Hz（micro-ROS 需要較高頻率）

  uint32_t counter = 0;
  bool microros_init_attempted = false;

  // 等待一段時間讓系統穩定
  vTaskDelay(pdMS_TO_TICKS(3000)); // 3秒延遲

  // 初始化 micro-ROS
  printf("[CommTask] 嘗試初始化 micro-ROS...\r\n");
  if (MicroROS_Init()) {
    printf("[CommTask] micro-ROS 初始化成功！\r\n");
    microros_init_attempted = true;
  } else {
    printf("[CommTask] micro-ROS 初始化失敗，進入診斷模式\r\n");
  }

  for (;;) {
    counter++;

    // ========== micro-ROS 處理 ==========
    if (microros_init_attempted && MicroROS_IsInitialized()) {
      // 1. 執行 Executor（處理訂閱的訊息）
      MicroROS_Spin();

      // 2. 發布關節狀態（50Hz）
      MicroROS_PublishJointStates();
    }

    // ========== 診斷輸出（每 2 秒一次）==========
    if (counter % 100 == 0) { // 50Hz * 100 = 2秒
      float v1 = Motor_GetVelocity(&motor_joint_13pin);
      float v2 = Motor_GetVelocity(&motor_joint_13pin_2);
      float v3 = Motor_GetVelocity(&motor_joint_13pin_3);
      uint32_t cnt1 = (uint32_t)motor_joint_13pin.last_counter_val;
      uint32_t cnt2 = (uint32_t)motor_joint_13pin_2.last_counter_val;
      uint32_t cnt3 = (uint32_t)motor_joint_13pin_3.last_counter_val;

      printf("\r\n=== System Status ===\r\n");
      printf("M1: RPM=%4d (CNT=%lu) | M2: RPM=%4d (CNT=%lu) | M3: RPM=%4d "
             "(CNT=%lu)\r\n",
             (int)v1, cnt1, (int)v2, cnt2, (int)v3, cnt3);
      printf("micro-ROS: %s\r\n",
             MicroROS_IsInitialized() ? "CONNECTED" : "DISCONNECTED");
      printf("Mode: %ld | Target: M1=%ld, M2=%ld, M3=%ld\r\n",
             test_mode_selection, test_target_rpm_m1, test_target_rpm_m2,
             test_target_rpm_m3);
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

  // 建立控制任務 (最高優先級 = 3)
  xTaskCreate(ControlTask,        // 任務函式
              "ControlTask",      // 任務名稱 (用於除錯)
              512,                // Stack 大小加大到 512 words (2KB) 避免溢位
              NULL,               // 任務參數
              3,                  // 優先級
              &xControlTaskHandle // 任務控制代碼
  );

  // 建立通訊任務 (中等優先級 = 2) - 暫時註解掉以排查硬體錯誤
  // TaskHandle_t commTaskHandle = NULL;
  // xTaskCreate(
  //   CommTask,
  //   "CommTask",
  //   1024,                  // Stack 大小加大到 1024 words (4KB) 給 micro-ROS
  //   和 printf 用 NULL, 2, &commTaskHandle
  // );

  // defaultTask 已經在 main.c 中由 CubeMX 自動建立
  // 其優先級為 Normal (通常是 1)，低於我們自訂的任務
}

/* USER CODE END Application */
