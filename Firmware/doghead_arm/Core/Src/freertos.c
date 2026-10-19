/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  * Note               : Core/Src/freertos.c is the active FreeRTOS initialization
  *                      and 1 kHz ControlTask/CommTask implementation owner.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "microros_app.h"
#include "mainpp.h"
#include "robot_interface.h"
#include "safety_manager.h"
#include "runtime_monitor.h"
#include "freertos_runtime.h"
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
extern TaskHandle_t xControlTaskHandle;

static osThreadId_t commTaskHandle;
const osThreadAttr_t commTask_attributes = {
  .name = "commTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};

static osThreadId_t controlTaskHandle;
static TaskHandle_t g_commTaskHandle = NULL;
static TaskHandle_t g_controlTaskHandle = NULL;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 2000 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* USER CODE END Variables */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartCommTask(void *argument);
void StartControlTask(void *argument);
/* USER CODE END FunctionPrototypes */

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartCommTask(void *argument) {
    (void)argument;
    // Default ping-only micro-ROS transport bringup (safe against pre-main HardFaults)
    MicroROS_Init();

    TickType_t last_state_pub = xTaskGetTickCount();
    TickType_t last_diag_pub = xTaskGetTickCount();

    for (;;) {
        // MicroROS_Spin handles rmw_uros_ping_agent XRCE session verification (and RCL entity creation if enabled)
        MicroROS_Spin();
        TickType_t now = xTaskGetTickCount();

        // 50 Hz McuState & /joint_states publication (every 20 ms)
        const TickType_t state_period = pdMS_TO_TICKS(20);
        if ((now - last_state_pub) >= state_period) {
            RobotTelemetry_t telem;
            RobotInterface_GetTelemetry(&telem);
            MicroROS_PublishMcuState(&telem);
            MicroROS_PublishJointStates(&telem);

            last_state_pub += state_period;
            if ((now - last_state_pub) >= state_period) {
                last_state_pub = now;
            }
        }

        // 1 Hz McuDiagnostics publication (every 1000 ms)
        if ((now - last_diag_pub) >= pdMS_TO_TICKS(1000)) {
            last_diag_pub = now;
            RobotDiagnostics_t diag;
            RobotInterface_GetDiagnostics(&diag);
            MicroROS_PublishMcuDiagnostics(&diag);
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void StartControlTask(void *argument) {
    (void)argument;
    xControlTaskHandle = xTaskGetCurrentTaskHandle();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint32_t start_cycles = DWT->CYCCNT;
        Robot_ControlTask_Tick(0.001f);
        uint32_t cycles = DWT->CYCCNT - start_cycles;
        const bool missed = cycles >= (SystemCoreClock / 1000U);
        RuntimeMonitor_RecordSample(cycles, missed);
        if (RuntimeMonitor_GetConsecutiveMisses() >= 3U) {
            RobotSafety_TriggerFault(ROBOT_FAULT_DEADLINE_MISS);
        }
    }
}

void MX_FREERTOS_Init(void) {
    commTaskHandle = osThreadNew(StartCommTask, NULL, &commTask_attributes);
    if (commTaskHandle == NULL) {
        Error_Handler();
        return;
    }
    g_commTaskHandle = (TaskHandle_t)commTaskHandle;
    controlTaskHandle = osThreadNew(StartControlTask, NULL, &controlTask_attributes);
    if (controlTaskHandle == NULL) {
        Error_Handler();
        return;
    }
    g_controlTaskHandle = (TaskHandle_t)controlTaskHandle;
}

uint32_t Robot_GetControlStackHighWatermark(void) {
    return g_controlTaskHandle == NULL ? 0U :
        (uint32_t)uxTaskGetStackHighWaterMark(g_controlTaskHandle);
}

uint32_t Robot_GetCommStackHighWatermark(void) {
    return g_commTaskHandle == NULL ? 0U :
        (uint32_t)uxTaskGetStackHighWaterMark(g_commTaskHandle);
}

/**
 * @brief FreeRTOS Stack Overflow Hook function
 * @note Triggered when configCHECK_FOR_STACK_OVERFLOW > 0 and stack overflow is detected
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
