# 09：詞彙、練習與下一步

## 快速詞彙表

| 詞彙 | 白話定義 |
|---|---|
| ROS graph | 所有 nodes 與 endpoints 的動態拓樸 |
| DDS / RMW | ROS 2 常用 middleware / middleware 抽象層 |
| topic | 有型別、非同步、多對多資料流 |
| QoS | reliability、history、durability 等傳輸契約 |
| package / workspace | 發布建置單位 / colcon 多 package 工作目錄 |
| underlay / overlay | 官方 ROS 與疊加的專案安裝空間 |
| micro-ROS / Agent | MCU client 軟體棧 / XRCE 與 DDS 間的 proxy |
| executor | 等待事件並派送 ROS callback |
| RTOS task / ISR | scheduler task / 中斷服務 context |
| WCET / jitter | 最壞執行時間 / 時序偏移的變化 |
| critical section | 短時間保護 shared state 的區段 |
| mailbox | 本案保存最新 command 的長度 1 queue |
| TTL / session / sequence | 新鮮度 / arm 世代 / 順序防護 |
| fail-closed | 錯誤或不確定時回到不輸出的狀態 |
| shadow mode | 執行計算與資料路徑但禁止 actuator |
| HIL | 真硬體參與的整合測試 |
| IK/FK / PVA | 逆/正運動學 / 位置速度加速度目標 |

## 綜合自我測驗

1. Docker image、container、bind mount 各是什麼？
2. 為什麼新 shell 需要重新 source？
3. package 與 node 有何不同？
4. Best Effort state stream 掉一 frame 為何未必需重送？
5. `.msg` 修改後，host 和 MCU 各要重建什麼？
6. motion node 在未 armed 時如何保持 shadow-safe？
7. driver 收到 NaN telemetry 時做什麼？
8. YAML 有 key 為何不代表程式已使用？
9. ControlTask 比 CommTask priority 高的理由？
10. `vTaskDelayUntil` 為何適合固定週期？
11. ISR 為什麼應使用 `...FromISR()`？
12. 長度 1 overwrite queue 為何適合 setpoint？
13. reconnect 時為何要換 session、清 command、reset PID/interpolator？
14. 列出 actuator output 的至少六個 AND 條件。
15. 如何證明 1 kHz deadline，而不是只證明 50 Hz telemetry？

## 實作作業

### A：畫資料流

從 EF reference 畫到 motor gate，再畫 encoder 回 `/joint_states`；每條箭頭標 type、頻率、QoS 與 execution context。

### B：新增 safety test

測試 arm session A、disarm、arm session B；驗證 B 不等於 A，並驗證 A 的 command 被拒絕。

### C：參數追蹤表

列出 YAML 每個 key，對照 `declare_parameter`、`get_parameter` 與 consumer，分類「已使用」「宣告但未注入」「未宣告」。

### D：頻寬預算

估算 McuState payload、CDR alignment、XRCE overhead，乘 50 Hz，與 460800 8N1 約 46 kB/s raw capacity 比較。

### E：時序量測

設計 ControlTask entry/exit GPIO 或 DWT cycle 量測，記錄最大值/deadline miss；說明 overhead，避免 1 kHz path `printf`。

## 答案方向

- 第 6 題：session 0、mask 0，但仍可做 IK/status。
- 第 10 題：固定 release 基準避免 execution time 累積 drift。
- 第 13 題：避免舊世代命令和動態狀態跨 reconnect 延續。
- session 防跨 arm 世代，sequence 防亂序，TTL 防太老。
- 1 kHz deadline 要在 MCU 量 execution/release/deadline counters。

## 下一階段

1. 完成 unit tests 與 Lab 0–4。
2. 不接馬達電源完成 telemetry-only Lab 5–8。
3. 實作參數真正注入 validator 並加測試。
4. 為 motion 補 timer-based trajectory 與 cancel/preempt。
5. 建自動 HIL 報告：版本、設定、rosbag、diagnostics、pass/fail。
6. 最後才進行有 checklist 與實體急停的 powered single-axis bring-up。

能可靠解釋錯誤路徑，才算真正理解系統。對機器人控制而言，「為何它沒有動」和「為何它會動」同樣重要。

