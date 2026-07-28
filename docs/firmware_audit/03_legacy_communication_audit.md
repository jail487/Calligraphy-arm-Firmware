# 舊通訊程式健檢

## 實際結論

沒有找到 rosserial、ROS 1、`ros_lib`、`cmd_vel` 實作、odometry message 或舊 `V_X/V_Y/V_W` callback。找到的是一套 **未完成且已被停用的 micro-ROS Humble 嘗試**，以及多份比程式現況更樂觀的舊文件。

## 分類

| 項目 | 現況 | 分類 | 建議 |
|---|---|---|---|
| `Middleware/MicroROS/Inc/microros_app.h` | 宣告 init/spin/joint state API | 可保留介面概念 | 重設為 connection state machine + command/snapshot API |
| `Middleware/MicroROS/Src/microros_app.c` | 所有函式是 stub，永遠 false | 目前有效但只表示 disabled | Phase 3 才替換；baseline 前勿刪 |
| `Middleware/FreeRTOS/Src/freertos.c:404-462` | CommTask source存在 | 未完成 / 不參與 runtime | 重新設計後啟用，不直接解除註解 |
| 同檔 `480-487` | CommTask `xTaskCreate` 被註解 | 明確停用 | 先修 heap/stack/transport/lifecycle |
| `Core/Src/main.c:27-33` | rcl/rclc/rmw headers註解 | 遺留 | Phase 2 移至 communication module，main 不應 include |
| `it_transport.c` | 整檔 `#if 0` | 舊 IT transport | 不直接復活；ring buffer 與 TX wait 有缺陷 |
| `microros_allocators.c` / `custom_memory_manager.c` | `#if 0` | 舊 allocator | 依 utility對應版本重建，不手動解除 |
| `microros_time.c` | 自製 rcutils/time symbols | 潛在衝突 | 和實際 libmicroros symbols比對後只保留必要 shim |
| `microros_stubs.c` | 自製 TLS/64-bit atomics | 潛在衝突 | 只在 link error有證據時採用；確認 task/ISR context |
| `libmicroros.a` | 檔案存在、humble branch | 未參與 link | 用相同 GCC flags重新產生並正式加 linker |
| `.cproject` | include path存在 | 半完成 | library `-L/-l` 仍空 |
| USART2 | init有效 | 可重用硬體 | 透過 CubeMX加 IRQ/DMA |
| repo agent script | serial agent 115200 | 可作 host baseline | ROS distro/version要和 firmware library鎖定 |

## 舊 transport 的具體問題

`Middleware/MicroROS/Src/it_transport.c` 若移除 `#if 0`：

- RX head/tail 在 ISR/task共享卻不是 volatile/atomic。
- 沒有 full-buffer detection；producer會覆寫未讀資料。
- callback沒有篩選 `huart`，未來多 UART 會互相污染。
- 每 byte 重啟 `HAL_UART_Receive_IT`，interrupt負載高。
- TX呼叫 `HAL_UART_Transmit_IT` 後以 `osDelay(1)` busy-style等待完成，會把非同步傳輸重新變成 blocking。
- `timeout` 和 `err` 參數未實作。
- USART2 global IRQ目前沒開、`USART2_IRQHandler`也不存在，因此即使編譯也不能運作。
- callback ownership 未集中；未來 DMA/IT transport只能有一個 HAL UART callback dispatcher。

建議不直接修補此檔，而以 utility clone中與同一 commit、同一 library build配套的 DMA transport為基準，外面包一層 project-owned `uros_transport`。

## 舊 topic / message

舊文件聲稱：

- publisher `/joint_states`：`sensor_msgs/msg/JointState` @ 50 Hz
- subscriber `/motor_cmd`：`std_msgs/msg/Float32MultiArray`
- 有些測試文件又假設 `/cmd_vel` 或 `/chatter`

實際 ELF沒有任何 rcl/rmw/uxr symbol，以上 topic都不存在。`/cmd_vel` 是移動底盤語意，不適合三軸機械臂；`Float32MultiArray` 缺 joint name、單位與 mode 語意，也不應直接通往 PWM。

可重用的是「joint state feedback」方向，不是舊文件宣稱的完成狀態。

## 文件與現況衝突

`Project_Documentation_v3.md`、repo `docs/MICROROS_INTEGRATION.md`、`ROS2_INTEGRATION_TEST.md` 和 roadmap宣稱 micro-ROS publisher/subscriber、USART IRQ甚至 DMA已完成；實際則是 stub + disabled task + no IRQ/DMA + no library link。部分文件還引用不存在的 `Core/Src/microros_app.c` 路徑。

處理順序：

1. 保留舊文件作歷史參考，但在標頭加「historical / not current」。
2. 以本 audit 作 current truth。
3. Phase 3 通過實機 agent測試後，才重寫 integration guide。
4. 不以舊文件的預期 console output當成測試證據。

## UART reuse / conflict

USART2 PA2/PA3 可作 micro-ROS transport，但需單一 ownership：

```text
USART2 byte stream
└─ XRCE-DDS framing only

debug text
├─ SWO/ITM（開發期首選）
├─ 另一 UART
└─ ROS diagnostics topic（低頻、結構化）
```

禁止同一 USART2同時輸出 `printf` 和 XRCE-DDS。現有 `printf` 也沒有 backend，若未來讓 CommTask可連結且直接呼叫 weak undefined `__io_putchar`，可能 fault。

## 遷移決策

- 保留：USART2 hardware init、FreeRTOS task separation概念、joint state方向、utility clone作參考。
- 重寫：micro-ROS application、connection/reconnect state machine、command buffer、telemetry snapshot、transport wrapper。
- 淘汰候選：舊 IT transport、雙 heap allocator副本、無證據的 atomic/TLS stubs、`/motor_cmd Float32MultiArray`。
- 暫不刪除：所有上述檔案；Phase 2先標記、建立可回退 baseline，再逐一從 build排除。

