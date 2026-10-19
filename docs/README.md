# DogHead 文件導覽

本目錄的文件橫跨早期韌體稽核、現行 ROS 2 HIL 實作與馬達帶電啟動條件。初學者請依下列順序閱讀，避免把歷史 Phase 編號和現行運動控制 HIL 的 Phase 混在一起。

## ROS 2、Docker、FreeRTOS 初學者完整教材

請由 [`beginner_tutorial/README.md`](beginner_tutorial/README.md) 開始。這套分章教材以本書法手臂的實際 package、topic、micro-ROS transport、FreeRTOS task 與安全閘門為主線，包含觀念、CLI、原始碼導讀、shadow-mode HIL 實驗、除錯表與練習。

所有說明文字以繁體中文撰寫；ROS 2、FreeRTOS、micro-ROS、QoS 等既有技術名稱，以及程式碼、topic、message、CLI 指令與 API 識別字，保留原文以利對照實際系統。

## 建議閱讀順序

1. [`project_management/05_motion_control_hil_learning_report.md`](project_management/05_motion_control_hil_learning_report.md)
   ：初學者主教材，說明 ROS 2、FreeRTOS、micro-ROS、QoS、安全協定與除錯。
2. [`project_management/04_motion_control_hil_execution_plan.md`](project_management/04_motion_control_hil_execution_plan.md)
   ：canonical interface、工作清單、Phase gates 與目前勾選狀態。
3. [`ros2_integration/05_ros2_hil_packages_architecture.md`](ros2_integration/05_ros2_hil_packages_architecture.md)
   ：五個 ROS 2 套件的責任邊界。
4. [`ros2_integration/01_microros_architecture_and_state_machine.md`](ros2_integration/01_microros_architecture_and_state_machine.md)
   ：MCU micro-ROS state machine、transport 與 entity lifecycle。
5. [`ros2_integration/02_wsl2_usb_and_agent_setup_guide.md`](ros2_integration/02_wsl2_usb_and_agent_setup_guide.md)
   ：Windows COM6 bridge、Docker Agent 與操作方式。
6. [`ros2_integration/micro_ros_header_generation.md`](ros2_integration/micro_ros_header_generation.md)
   ：自訂訊息與靜態函式庫重建。

## 現行基線

- Target：NUCLEO-F446RE / STM32F446RET6。
- RTOS：FreeRTOS；ControlTask 1 kHz、CommTask 約 50 Hz。
- Host：Ubuntu 22.04 / ROS 2 Humble container。
- Transport：USART2 460800 8N1，Windows pyserial TCP bridge，RXNE IRQ ring。
- 測試：5 個 ROS 套件、31 個測試結果、0 個失敗。
- HIL：Phase 0–2 輸出關閉通過；馬達帶電的 Phase 3 未授權。
- Safety：actuator hardware gate=0，未綁定 session 時 command fail-closed。

## 文件分類

### `project_management/`

系統架構、執行計畫、Master 文件、學習教材與 agent 工作紀錄。若不同文件狀態
衝突，以 `04_motion_control_hil_execution_plan.md` 和
`05_motion_control_hil_learning_report.md` 的 2026-07-31 狀態為準。

### `ros2_integration/`

ROS 2 套件、micro-ROS、Windows/WSL2/Docker 序列埠路徑、靜態函式庫產生
與除錯事後分析。

### `firmware_audit/`

早期 bare-metal → FreeRTOS 重構與 code review 歷史。Phase 編號屬於早期 firmware
roadmap，不等於現行 motion HIL Phase。

### `firmware_bringup_audit/`

Powered motor 測試的風險與 Conditional GO 條件。這些文件不是目前的通電授權；
必須先取得硬體參數、E-stop 驗證與另一次 gate=1 approval。
