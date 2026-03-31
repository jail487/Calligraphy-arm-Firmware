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
#include "task.h"
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mainpp.h"  // 用於呼叫 C++ 的 Robot_Loop
#include "nidec_motor_driver.h" // 用於存取馬達物件與 API
#include "microros_app.h"  // micro-ROS 應用層
#include <stdio.h>

// 全域變數已經移到 main.h 與 nidec_motor_driver.h 中

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
void MX_FREERTOS_Init(void);  // 宣告初始化函式
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
void ControlTask(void *argument)
{
  // 狀態變數
  static int32_t last_mode = 0;

  // Mode 2: 線性加速變數
  static float ramp_current_rpm = 0.0f;
  const float ramp_target_rpm = 5000.0f;  // 最大速度降至 5000
  const float ramp_duration_sec = 10.0f;  // 加速時間縮短至 10秒 (原本60秒)
  const float ramp_step = ramp_target_rpm / (ramp_duration_sec * 1000.0f); // 每ms增加量

  // Mode 3: 階梯加速變數
  static int32_t step_current_rpm = 0;
  const int32_t step_target_rpm = 5000;   // 最大速度降至 5000
  const int32_t step_increment = 250;     // 階梯增量加大到 250 (原本50)
  const uint32_t step_interval_ms = 1000; // 時間間隔縮短到 1秒 (原本4秒)
  static uint32_t step_timer = 0;

  // Mode 4: 自定義時序變數
  static uint32_t m4_timer = 0;
  const uint32_t m4_step_duration = 1000; // 1000ms
  const int32_t m4_spd_low = 100;
  const int32_t m4_spd_mid = 200;
  const int32_t m4_spd_high = 300;
  const int32_t m4_spd_mid_neg = -200;

  // 在任務開始執行後，才啟動 TIM6 中斷，確保 xControlTaskHandle 已就緒且 OS 已運行
  HAL_TIM_Base_Start_IT(&htim6);

  for(;;)
  {
    // 等待硬體 TIM6 中斷的通知 (阻塞等待，極低延遲)
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    
    // ========== 測試模式判斷 ==========
    // test_mode_selection == 0: PID 控制模式
    // test_mode_selection >= 1: 測試模式（純轉速控制）
    if (test_mode_selection == 0) {
        // PID 模式：使用 PID + 前饋控制，不含運動學
        // 同步 Live Expressions 的目標角度到 Robot Core
        Robot_SetTargetAngles3(pid_target_theta1, pid_target_theta2, pid_target_theta3);
        Robot_PID_Loop(0.001f);

        // 更新實際轉速回授 (PID Mode)
        real_rpm1 = Motor_GetVelocity(&motor_joint_13pin);
        real_rpm2 = Motor_GetVelocity(&motor_joint_13pin_2);
        real_rpm3 = Motor_GetVelocity(&motor_joint_13pin_3);

        continue;  // 跳過後續測試模式邏輯
    }
    
    // ========== 測試模式邏輯 ==========
    // 偵測模式切換，重置狀態
    if (test_mode_selection != last_mode) {
        // 重置 Mode 2 狀態
        ramp_current_rpm = 0.0f;
        
        // 重置 Mode 3 狀態
        step_current_rpm = 0;
        step_timer = 0;

        // 重置 Mode 4 狀態
        m4_timer = 0;
        
        // 如果切換回 Mode 1，使用設定的目標速度
        if (test_mode_selection == 1) {
            test_target_rpm_m1 = mode1_target_rpm;
        }
        
        last_mode = test_mode_selection;
    }

    // 根據模式執行邏輯
    switch (test_mode_selection) {
        case 1: // 定速模式 (可調)
            // 允許使用者透過 Live Expressions 直接修改 test_target_rpm_m1
            // 不再強制鎖定為 mode1_target_rpm
            test_target_rpm_m2 = test_target_rpm_m1;  // M2 同步
            test_target_rpm_m3 = test_target_rpm_m1;  // M3 同步
            break;

        case 2: // 線性加速 (10秒到5000)
            if (ramp_current_rpm < ramp_target_rpm) {
                ramp_current_rpm += ramp_step;
                if (ramp_current_rpm > ramp_target_rpm) ramp_current_rpm = ramp_target_rpm;
                test_target_rpm_m1 = (int32_t)ramp_current_rpm;
                test_target_rpm_m2 = (int32_t)ramp_current_rpm;  // M2 同步
            }
            break;

        case 3: // 階梯加速 (每1秒+250)
            step_timer++;
            if (step_timer >= step_interval_ms) {
                step_timer = 0;
                if (step_current_rpm < step_target_rpm) {
                    step_current_rpm += step_increment;
                    if (step_current_rpm > step_target_rpm) step_current_rpm = step_target_rpm;
                    test_target_rpm_m1 = step_current_rpm;
                    test_target_rpm_m2 = step_current_rpm;  // M2 同步
                }
            }
            break;

        case 4: // 自定義時序模式 (M1: Mid-Low-High, M2: Stop-Mid-Stop, M3: Low-Mid-NegMid)
            m4_timer++;
            // 計算目前處於哪個步驟 (週期為 3個步驟 * 4秒 = 12秒)
            {
                uint32_t period = m4_step_duration * 3;
                uint32_t current_time = m4_timer % period;
                
                if (current_time < m4_step_duration) {
                    // Step 0: 0-4s
                    // M1: Mid, M2: Stop, M3: Low
                    test_target_rpm_m1 = m4_spd_mid;
                    test_target_rpm_m2 = 0;
                    test_target_rpm_m3 = m4_spd_low;
                } else if (current_time < m4_step_duration * 2) {
                    // Step 1: 4-8s
                    // M1: Low, M2: Mid, M3: Mid
                    test_target_rpm_m1 = m4_spd_low;
                    test_target_rpm_m2 = m4_spd_mid;
                    test_target_rpm_m3 = m4_spd_mid;
                } else {
                    // Step 2: 8-12s
                    // M1: High, M2: Stop, M3: NegMid
                    test_target_rpm_m1 = m4_spd_high;
                    test_target_rpm_m2 = 0;
                    test_target_rpm_m3 = m4_spd_mid_neg;
                }
            }
            break;
            
        default:
            // 未知模式
            break;
    }

    // 只有在測試模式下 (Mode > 0)，才強制使用 test_target_rpm 覆寫馬達速度
    if (test_mode_selection > 0) {
        // 直接更新馬達速度（不經過 Robot_Loop 的運動學與 PID）
        // Motor_Update 已經在前面執行過了 (如果前面沒補，這裡該補，但 PID loop 也有做)
        Motor_Update(&motor_joint_13pin);
        Motor_Update(&motor_joint_13pin_2);
        Motor_Update(&motor_joint_13pin_3);
        
        // [新增] 更新角度回授變數，以便在 Live Expressions 監看方向
        real_theta1 = Motor_GetAngle(&motor_joint_13pin);
        real_theta2 = Motor_GetAngle(&motor_joint_13pin_2);
        real_theta3 = Motor_GetAngle(&motor_joint_13pin_3);

        // [新增] 更新轉速回授
        real_rpm1 = Motor_GetVelocity(&motor_joint_13pin);
        real_rpm2 = Motor_GetVelocity(&motor_joint_13pin_2);
        real_rpm3 = Motor_GetVelocity(&motor_joint_13pin_3);
        
        // 確保馬達啟動
        Motor_Start(&motor_joint_13pin);
        Motor_Start(&motor_joint_13pin_2);
        Motor_Start(&motor_joint_13pin_3);
        
        // 直接設定速度
        Motor_SetSpeed(&motor_joint_13pin, test_target_rpm_m1);
        Motor_SetSpeed(&motor_joint_13pin_2, test_target_rpm_m2);
        Motor_SetSpeed(&motor_joint_13pin_3, test_target_rpm_m3);
    }
    // Mode 0 時，控制權完全交給前面的 Robot_PID_Loop，這裡不做任何干涉
    
    // 不再使用 vTaskDelayUntil，改由硬體中斷觸發
    // 任務在迴圈開頭的 ulTaskNotifyTake 處阻塞等待
  }
}

/**
 * @brief 通訊任務：低優先級，10Hz (100ms 週期)
 * @note 負責處理 micro-ROS 通訊、診斷輸出
 */
void CommTask(void *argument)
{
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms = 50Hz（micro-ROS 需要較高頻率）
  
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
  
  for(;;)
  {
    counter++;
    
    // ========== micro-ROS 處理 ==========
    if (microros_init_attempted && MicroROS_IsInitialized()) {
      // 1. 執行 Executor（處理訂閱的訊息）
      MicroROS_Spin();
      
      // 2. 發布關節狀態（50Hz）
      MicroROS_PublishJointStates();
    }
    
    // ========== 診斷輸出（每 2 秒一次）==========
    if (counter % 100 == 0) {  // 50Hz * 100 = 2秒
            float v1 = Motor_GetVelocity(&motor_joint_13pin);
            float v2 = Motor_GetVelocity(&motor_joint_13pin_2);
            float v3 = Motor_GetVelocity(&motor_joint_13pin_3);
            uint32_t cnt1 = 0, cnt2 = 0, cnt3 = 0;
      if(motor_joint_13pin.config.htim_encoder) cnt1 = __HAL_TIM_GET_COUNTER(motor_joint_13pin.config.htim_encoder);
            if(motor_joint_13pin_2.config.htim_encoder)  cnt2 = __HAL_TIM_GET_COUNTER(motor_joint_13pin_2.config.htim_encoder);
            if(motor_joint_13pin_3.config.htim_encoder)  cnt3 = __HAL_TIM_GET_COUNTER(motor_joint_13pin_3.config.htim_encoder);
      
      printf("\r\n=== System Status ===\r\n");
            printf("M1: RPM=%4d (CNT=%lu) | M2: RPM=%4d (CNT=%lu) | M3: RPM=%4d (CNT=%lu)\r\n", (int)v1, cnt1, (int)v2, cnt2, (int)v3, cnt3);
      printf("micro-ROS: %s\r\n", MicroROS_IsInitialized() ? "CONNECTED" : "DISCONNECTED");
            printf("Mode: %ld | Target: M1=%ld, M2=%ld, M3=%ld\r\n", 
              test_mode_selection, test_target_rpm_m1, test_target_rpm_m2, test_target_rpm_m3);
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

/**
 * @brief 在 main() 的 RTOS_THREADS 區段呼叫此函式來建立任務
 */
void MX_FREERTOS_Init(void)
{
  // 引用外部宣告的任務句柄（定義在 main.c）
  extern TaskHandle_t xControlTaskHandle;
  
  // 建立控制任務 (最高優先級 = 3)
  xTaskCreate(
    ControlTask,           // 任務函式
    "ControlTask",         // 任務名稱 (用於除錯)
    512,                   // Stack 大小加大到 512 words (2KB) 避免溢位
    NULL,                  // 任務參數
    3,                     // 優先級
    &xControlTaskHandle    // 任務控制代碼
  );
  
  // 建立通訊任務 (中等優先級 = 2) - 暫時註解掉以排查硬體錯誤
  // TaskHandle_t commTaskHandle = NULL;
  // xTaskCreate(
  //   CommTask,
  //   "CommTask",
  //   1024,                  // Stack 大小加大到 1024 words (4KB) 給 micro-ROS 和 printf 用
  //   NULL,
  //   2,
  //   &commTaskHandle
  // );
  
  // defaultTask 已經在 main.c 中由 CubeMX 自動建立
  // 其優先級為 Normal (通常是 1)，低於我們自訂的任務
}

/* USER CODE END Application */

