/**
 * @file microros_app.c
 * @brief micro-ROS 應用層：實作 Publisher, Subscriber, 訊息處理
 * @note 此檔案包含所有 micro-ROS 相關的業務邏輯
 */

#include "microros_app.h"
#include "nidec_motor_driver.h"
#include "mainpp.h"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <stdio.h>

// 外部變數宣告
extern Motor_t motor_joint_13pin;
extern Motor_t motor_joint_13pin_2;

// ==========================================================
// micro-ROS 物件宣告
// ==========================================================
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;

// Publisher: 發布關節狀態
static rcl_publisher_t joint_state_publisher;
static sensor_msgs__msg__JointState joint_state_msg;

// Subscriber: 訂閱馬達指令
static rcl_subscription_t motor_cmd_subscriber;
static std_msgs__msg__Float32MultiArray motor_cmd_msg;

// 狀態標記
static bool microros_initialized = false;
static uint32_t publish_counter = 0;

// ==========================================================
// 內部函式原型
// ==========================================================
static void motor_cmd_callback(const void *msgin);
static void init_joint_state_msg(void);
static void init_motor_cmd_msg(void);

// ==========================================================
// Subscriber 回呼函式
// ==========================================================
/**
 * @brief 馬達指令回呼函式
 * @note 從 ROS2 接收馬達指令並執行
 */
static void motor_cmd_callback(const void *msgin)
{
    const std_msgs__msg__Float32MultiArray *msg = 
        (const std_msgs__msg__Float32MultiArray *)msgin;
    
    // 檢查數據長度
    if (msg->data.size >= 3) {
        int32_t motor1_rpm = (int32_t)msg->data.data[0];
        int32_t motor2_rpm = (int32_t)msg->data.data[1];
        int32_t motor3_rpm = (int32_t)msg->data.data[2];
        
        // 使用測試模式 API 設定速度
        Robot_SetTestMode(true);
        Robot_SetTestSpeed(motor1_rpm, motor2_rpm, motor3_rpm);
        
        // 診斷輸出
        printf("[micro-ROS] 收到指令: M1=%ld RPM, M2=%ld RPM, M3=%ld RPM\r\n", 
               motor1_rpm, motor2_rpm, motor3_rpm);
    }
}

// ==========================================================
// 訊息初始化函式
// ==========================================================
static void init_joint_state_msg(void)
{
    // 初始化 JointState 訊息
    sensor_msgs__msg__JointState__init(&joint_state_msg);
    
    // 分配記憶體給關節名稱（3個關節）
    joint_state_msg.name.capacity = 3;
    joint_state_msg.name.size = 3;
    joint_state_msg.name.data = (rosidl_runtime_c__String*)malloc(
        sizeof(rosidl_runtime_c__String) * 3);
    
    // 初始化關節名稱
    rosidl_runtime_c__String__init(&joint_state_msg.name.data[0]);
    rosidl_runtime_c__String__init(&joint_state_msg.name.data[1]);
    rosidl_runtime_c__String__init(&joint_state_msg.name.data[2]);
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], "joint1");
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[1], "joint2");
    rosidl_runtime_c__String__assign(&joint_state_msg.name.data[2], "joint3");
    
    // 分配記憶體給位置、速度、力矩
    joint_state_msg.position.capacity = 3;
    joint_state_msg.position.size = 3;
    joint_state_msg.position.data = (double*)malloc(sizeof(double) * 3);
    
    joint_state_msg.velocity.capacity = 3;
    joint_state_msg.velocity.size = 3;
    joint_state_msg.velocity.data = (double*)malloc(sizeof(double) * 3);
    
    joint_state_msg.effort.capacity = 0;  // 暫不使用
    joint_state_msg.effort.size = 0;
}

static void init_motor_cmd_msg(void)
{
    // 初始化 Float32MultiArray 訊息
    std_msgs__msg__Float32MultiArray__init(&motor_cmd_msg);
    
    // 預分配記憶體給數據陣列（3個馬達）
    motor_cmd_msg.data.capacity = 3;
    motor_cmd_msg.data.size = 3;
    motor_cmd_msg.data.data = (float*)malloc(sizeof(float) * 3);
}

// ==========================================================
// micro-ROS 初始化函式
// ==========================================================
bool MicroROS_Init(void)
{
    printf("[micro-ROS] 開始初始化...\r\n");
    
    // 1. 設定記憶體分配器
    allocator = rcl_get_default_allocator();
    
    // 2. 初始化 Support
    if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
        printf("[micro-ROS] ERROR: Support 初始化失敗\r\n");
        return false;
    }
    
    // 3. 創建 Node
    if (rclc_node_init_default(&node, "doghead_arm_node", "", &support) != RCL_RET_OK) {
        printf("[micro-ROS] ERROR: Node 初始化失敗\r\n");
        return false;
    }
    printf("[micro-ROS] Node 創建成功: doghead_arm_node\r\n");
    
    // 4. 初始化訊息結構
    init_joint_state_msg();
    init_motor_cmd_msg();
    
    // 5. 創建 Publisher (發布關節狀態)
    if (rclc_publisher_init_default(
            &joint_state_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
            "/joint_states") != RCL_RET_OK) {
        printf("[micro-ROS] ERROR: Publisher 初始化失敗\r\n");
        return false;
    }
    printf("[micro-ROS] Publisher 創建成功: /joint_states\r\n");
    
    // 6. 創建 Subscriber (訂閱馬達指令)
    if (rclc_subscription_init_default(
            &motor_cmd_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
            "/motor_cmd") != RCL_RET_OK) {
        printf("[micro-ROS] ERROR: Subscriber 初始化失敗\r\n");
        return false;
    }
    printf("[micro-ROS] Subscriber 創建成功: /motor_cmd\r\n");
    
    // 7. 創建 Executor
    if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
        printf("[micro-ROS] ERROR: Executor 初始化失敗\r\n");
        return false;
    }
    
    // 8. 將 Subscription 加入 Executor
    if (rclc_executor_add_subscription(
            &executor,
            &motor_cmd_subscriber,
            &motor_cmd_msg,
            &motor_cmd_callback,
            ON_NEW_DATA) != RCL_RET_OK) {
        printf("[micro-ROS] ERROR: 無法將 Subscriber 加入 Executor\r\n");
        return false;
    }
    
    microros_initialized = true;
    printf("[micro-ROS] 初始化完成！\r\n");
    
    return true;
}

// ==========================================================
// micro-ROS 發布函式
// ==========================================================
void MicroROS_PublishJointStates(void)
{
    if (!microros_initialized) {
        return;
    }
    
    // 讀取當前關節狀態
    float theta1_deg = Motor_GetAngle(&motor_joint_13pin);
    float theta2_deg = Motor_GetAngle(&motor_joint_13pin_2);
    float theta3_deg = Motor_GetAngle(&motor_joint_13pin_3);
    float vel1_rpm = Motor_GetVelocity(&motor_joint_13pin);
    float vel2_rpm = Motor_GetVelocity(&motor_joint_13pin_2);
    float vel3_rpm = Motor_GetVelocity(&motor_joint_13pin_3);
    
    // 轉換單位：度 -> 弧度，RPM -> rad/s
    joint_state_msg.position.data[0] = theta1_deg * 0.0174533;  // deg to rad
    joint_state_msg.position.data[1] = theta2_deg * 0.0174533;
    joint_state_msg.position.data[2] = theta3_deg * 0.0174533;
    joint_state_msg.velocity.data[0] = vel1_rpm * 0.10472;      // rpm to rad/s
    joint_state_msg.velocity.data[1] = vel2_rpm * 0.10472;
    joint_state_msg.velocity.data[2] = vel3_rpm * 0.10472;
    
    // 更新時間戳（簡化版，使用計數器）
    joint_state_msg.header.stamp.sec = publish_counter / 50;  // 假設50Hz
    joint_state_msg.header.stamp.nanosec = (publish_counter % 50) * 20000000;
    
    // 發布訊息
    rcl_ret_t ret = rcl_publish(&joint_state_publisher, &joint_state_msg, NULL);
    if (ret != RCL_RET_OK) {
        printf("[micro-ROS] WARN: 發布失敗 (ret=%d)\r\n", (int)ret);
    }
    
    publish_counter++;
    
    // 每秒輸出一次診斷（假設50Hz發布）
    if (publish_counter % 50 == 0) {
        printf("[micro-ROS] 發布狀態: θ1=%.2f°, θ2=%.2f°, θ3=%.2f°\r\n", 
               theta1_deg, theta2_deg, theta3_deg);
    }
}

// ==========================================================
// micro-ROS Spin 函式
// ==========================================================
void MicroROS_Spin(void)
{
    if (!microros_initialized) {
        return;
    }
    
    // 執行 Executor（處理 Subscription 回呼）
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));  // 1ms timeout
}

// ==========================================================
// micro-ROS 清理函式
// ==========================================================
void MicroROS_Fini(void)
{
    if (!microros_initialized) {
        return;
    }
    
    printf("[micro-ROS] 開始清理...\r\n");
    
    // 清理 Executor
    rclc_executor_fini(&executor);
    
    // 清理 Publisher & Subscriber
    rcl_publisher_fini(&joint_state_publisher, &node);
    rcl_subscription_fini(&motor_cmd_subscriber, &node);
    
    // 清理 Node & Support
    rcl_node_fini(&node);
    rclc_support_fini(&support);
    
    // 釋放訊息記憶體
    sensor_msgs__msg__JointState__fini(&joint_state_msg);
    std_msgs__msg__Float32MultiArray__fini(&motor_cmd_msg);
    
    microros_initialized = false;
    printf("[micro-ROS] 清理完成\r\n");
}

// ==========================================================
// 狀態查詢函式
// ==========================================================
bool MicroROS_IsInitialized(void)
{
    return microros_initialized;
}
