# 書法手臂：ROS 2、Docker、FreeRTOS 專案式入門

這套教材不是把三種技術分開背名詞，而是沿著本專案的一筆命令走完整條路：

```text
末端座標目標
  -> ROS 2 motion node（IK、軌跡）
  -> ROS 2 driver node（session、TTL、限制、安全閘門）
  -> DDS
  -> micro-ROS Agent
  -> USB/UART
  -> STM32 CommTask（解包、mailbox）
  -> STM32 ControlTask（1 kHz、插補、PID、安全檢查）
  -> 馬達輸出
```

目標讀者是「懂一點 STM32，但剛接觸 ROS 2、Docker 與 FreeRTOS」的開發者。讀完後，你應該能回答：每一層為何存在、資料如何流動、即時性放在哪裡、安全機制如何失效關閉，以及發生問題時應從哪一層開始查。

## 建議學習路線

| 順序 | 章節 | 完成後能做到 |
|---|---|---|
| 0 | [學習地圖與環境](00_learning_map.md) | 分清楚 Windows、容器、ROS graph、MCU 四個執行世界 |
| 1 | [系統架構與一筆命令的旅程](01_system_architecture.md) | 說明每個 package、node、task 與 transport 的責任 |
| 2 | [Docker 從零到能開發](02_docker.md) | 看懂 image/container/volume/network，能進容器編譯 |
| 3 | [ROS 2 核心觀念與 CLI](03_ros2_fundamentals.md) | 操作 node、topic、service、parameter、QoS、launch |
| 4 | [逐檔閱讀本專案 ROS 2 程式](04_ros2_project.md) | 看懂 interfaces、driver、motion、HIL、bringup |
| 5 | [FreeRTOS 與即時控制](05_freertos.md) | 理解排程、優先權、週期、queue、critical section、ISR |
| 6 | [micro-ROS 與跨系統通訊](06_microros.md) | 解釋 DDS-XRCE、Agent、UART、狀態機與斷線重建 |
| 7 | [安全的端到端實驗](07_labs.md) | 從純軟體測試進到 shadow-mode HIL，不驅動馬達 |
| 8 | [除錯、測試與常見陷阱](08_debugging.md) | 用分層方法定位 Docker、DDS、Agent、UART、RTOS 問題 |
| 9 | [詞彙、練習與下一步](09_glossary.md) | 用題目驗證理解，規劃後續實作 |

## 三條不可混淆的邊界

1. **Docker 不等於 ROS 2**：Docker 只提供可重現的 Linux 執行環境；ROS 2 節點仍透過 DDS 通訊。
2. **ROS 2 不等於即時控制**：Linux 上做規劃、協調與監控；硬即時性較強的 1 kHz 迴圈放在 STM32/FreeRTOS。
3. **連得上不等於能動**：通訊成功、driver armed、host gate、firmware compile-time gate、MCU safety state 與 axis mask 都是不同條件。

## 專案基準與安全聲明

- ROS 發行版：Humble；容器映像為 `osrf/ros:humble-desktop-full`。
- MCU：STM32F446RE；FreeRTOS tick 為 1 kHz。
- 通訊：USART2，460800 baud，8N1；Windows 可透過 TCP 8888 橋接。
- `ControlTask`：1 ms 週期；`CommTask`：約每 5 ms 執行通訊工作，狀態 50 Hz、診斷 1 Hz。
- 新手實驗一律保持：ROS driver 的 `shadow_mode=true`、`actuator_enabled=false`，以及韌體 `ACTUATOR_HARDWARE_OUTPUT_ENABLED=0`。
- 軟體 E-stop 或 stop service 不能取代實體斷電、急停回路、驅動器 fault 線與機械限位。

文中的「目前實作」均以倉庫內原始碼為準；「建議」代表教學或工程改善方向，不表示專案已具備該功能。

