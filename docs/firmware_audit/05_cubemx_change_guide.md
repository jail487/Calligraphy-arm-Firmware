# CubeMX 修改指南

本文件是建議，不是已執行變更。開始前先保存可 build commit/tag、備份 `.ioc`、`.cproject` 與所有 USER CODE 外的可疑手改；Generate 後必須 review diff 並雙 config clean build。不要直接改 `MX_USART2_UART_Init`、`MX_TIMx_Init`、`SystemClock_Config` 或 MSP 生成區。

---

## 1. 取消與 Disable motor enable GPIO

* **項目**：三軸 Enable pin 取消與 Disable
* **目前設定**：CubeMX 原先在 PC4 設定 Output（M1/M2 共用 Enable），程式中註記 M3 為 PC0；`main.c` 亦將 PC5 設為 Output。現因硬體與控制需求變更，**Motor Enable 腳位已不再需要**。
* **建議設定**：
  1. **`PC4`**：在 CubeMX 中將 Mode 切換回 **Reset / Unconfigured (Free)**，取消 GPIO Output 初始化。
  2. **`PC0`**：保持 Unconfigured (Free)，程式碼中移除 `M3_ENABLE_PIN` 關聯。
  3. **`PC5`**：目前應用無引用，在 CubeMX 中切換回 **Reset / Unconfigured (Free)**。
  4. **保留腳位**：`PC1` (M1 DIR)、`PC2` (M2 DIR)、`PC3` (M3 DIR) 需繼續保留為 GPIO Output。
* **CubeMX 路徑**：Pinout & Configuration → System Core → GPIO → 點選 GPIOC (PC4, PC5)。
* **修改步驟**：選擇 PC4/PC5 → 將 Pin Configuration 改為 Reset / Disable Output → 清除 User Label → Generate Code。
* **改動目的與原因**：
  馬達啟用改由硬體全時接通或軟體 PWM / Direction 控制，不再需要 MCU GPIO 手動切換 Enable。將多餘腳位關閉可釋放 PC4/PC0/PC5 GPIO 資源，同時消除原先 PC4 多軸共用 Enable 及 PC0 未初始化可能導致的硬體狀態懸空疑慮。
* **若維持原樣不改會怎樣**：
  * **會造成的具體問題**：
    1. 開機時 MCU 會強行將 PC4/PC5 初始化為 Output 低電位，若驅動板已改為硬體 Enable，極可能導致馬達驅動晶片被誤鎖死、無法回應控制指令運轉。
    2. PC0 在 CubeMX 中未初始化為 GPIO，但若程式碼中繼續對其進行 GPIO 寫入，該操作將無效並產生狀態混淆。
    3. 增加開機時無謂的 GPIO 暫存器配置開銷。
  * **為什麼會發生（底層原因）**：
    目前硬體驅動板已被設計為硬體 Enable（拉高）或由 PWM 防護。CubeMX 生成的 `MX_GPIO_Init` 若仍將 PC4/PC5 拉低，MCU 輸出的低電位會與驅動板 Enable 端的電位保護發生衝突，鎖死驅動晶片；且 PC0 未初始化時暫存器處於浮動狀態，寫入無效。
* **受影響程式**：`motor_config.h/.c` 移除 Enable 定義與寫入、`main.c` 中的 `MX_GPIO_Init` 移除 PC4/PC5 Output 初始化。
* **Generate Code 風險**：極低；請確認驅動器硬體 Enable 端已正確拉高/拉低至工作電位。
* **驗證方式**：重新 Generate Code 後，確認 `MX_GPIO_Init` 僅初始化 PC1/PC2/PC3 做 Output，且馬達在 PWM/DIR 控制下可正常運動。

---

## 2. TIM6 IRQ priority

* **項目**：Control tick 可安全呼叫 FreeRTOS FromISR API
* **目前設定**：CubeMX 產生的 `.ioc` 優先級為 **10**；USER code runtime override 手動覆蓋為 **5**。
* **建議設定**：Preemption priority 改為 **5**、subpriority 改為 **0**。
* **CubeMX 路徑**：Pinout & Configuration → System Core → NVIC → NVIC tab → TIM6 global interrupt and DAC underrun。
* **修改步驟**：勾選 Enable → 優先級設為 5/0；Generate Code 後確認手寫覆蓋處可安全移除。
* **改動目的與原因**：
  TIM6 是觸發 1kHz 馬達 PID 控制任務 (`ControlTask`) 的關鍵計時器，中斷內部會呼叫 `vTaskNotifyGiveFromISR()`。依 FreeRTOS 規則（`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`），呼叫 `FromISR` API 的中斷優先級數值**不能小於 5**（在 ARM 中數字越小優先級越高）。設定為 **5** 能提供 FreeRTOS 許可的最強搶佔能力，大幅降低 1kHz PID 控制的時間抖動 (Jitter)；同時將 CubeMX 生成碼直接設為 5，可消除 CubeMX 生成碼 (10) 與手寫覆蓋 (5) 的不一致衝突，避免重新生成程式碼時設定意外失效。
* **若維持原樣不改會怎樣**：
  * **會造成的具體問題**：
    1. 1kHz 馬達 PID 控制的時間間隔發生非預期抖動 (Jitter)，導致馬達運轉時產生微幅異音、運轉不平順甚至額外發熱。
    2. 未來維護、重構或重新 Generate Code 時，手寫的覆蓋程式碼（USER CODE 區塊）若意外被刪除或未執行，TIM6 優先級會悄悄降回 10。
  * **為什麼會發生（底層原因）**：
    TIM6 的 `.ioc` 預設優先級為 10，小於 FreeRTOS 可呼叫 API 的最大優先級 5。當優先級為 10 時，優先級為 6~9 的其他週邊中斷（如 UART 通訊中斷）將能搶佔 TIM6，導致 1kHz 控制中斷被無預期延遲與打斷，產生控制時間間隔失真。
* **受影響程式**：`stm32f4xx_hal_msp.c`、`main.c`、控制時脈抖動 (Jitter)。
* **Generate Code 風險**：若手寫覆蓋程式碼未同步清理仍會重複執行，建議獨立提交修改。
* **驗證方式**：讀取 NVIC priority 暫存器、啟用 `configASSERT`、執行 1 kHz 中斷與控制通知壓力測試。

---

## 3. USART2 serial baseline

* **項目**：micro-ROS UART framing 通訊基準設定
* **目前設定**：USART2 asynchronous、PA2/PA3、115200 8N1、no flow control。
* **建議設定**：第一輪保持 115200 8N1；通過 baseline 測試後，再獨立測試 460800 等更高鮑率。
* **CubeMX 路徑**：Pinout & Configuration → Connectivity → USART2 → Parameter Settings。
* **修改步驟**：Mode 設為 Asynchronous；Baud Rate 115200；8 Bits；Parity None；Stop Bits 1；TX/RX；Hardware Flow Control None；Oversampling 16。
* **改動目的與原因**：
  在 micro-ROS 初步整合與除錯階段，維護標準 115200 鮑率能確保物理層 (Physical Layer) 訊號傳輸的極高可靠性與相容性。將「通訊協定/DMA搬移驗證」與「高鮑率訊號品質驗證」解耦，避免除錯時無法釐清是通訊軟體錯誤還是 UART 線路噪聲導致的封包遺失。
* **若維持原樣不改會怎樣**：
  * **會造成的具體問題**：
    1. 若在未確立硬體實體層與軟體通訊穩定前冒然拉高鮑率（如 460800），容易引發 high bit error rate、CRC 驗證失敗與封包遺失。
    2. micro-ROS Agent 發生頻繁斷線或重連，且開發團隊無法確定是軟體 Task/DMA 配置問題還是實體線路訊號噪聲問題。
  * **為什麼會發生（底層原因）**：
    高鮑率傳輸對 PCB 佈線抗噪能力、阻抗匹配與晶片時脈誤差極為敏感。若直接從高鮑率開始除錯，硬體訊號失真與軟體邏輯 Bug 會互相疊加，無法做到除錯上的「變數隔離 (Isolate Variables)」。
* **受影響程式**：micro-ROS Agent 啟動參數、UART 通訊超時與吞吐量評估。
* **Generate Code 風險**：會重新產生 `MX_USART2_UART_Init`；不得在函式本體中手動修改鮑率。
* **驗證方式**：與 Agent 建立連線、監測封包 sequence/CRC 與 24 小時長脈衝測試；通過後再比較更高鮑率。

---

## 4. USART2 RX/TX DMA

* **項目**：低 CPU 負擔、可持續的 Serial Transport
* **目前設定**：無 DMA request/handle/IRQ 配置。
* **建議設定**：USART2_RX 設為 **DMA Circular** 模式、USART2_TX 設為 **DMA Normal** 模式；Data width 設為 Byte、Memory increment 啟用、Peripheral increment 停用；DMA 優先級設為 **Very High**。
* **CubeMX 路徑**：Connectivity → USART2 → DMA Settings → Add USART2_RX / USART2_TX。
* **修改步驟**：新增 RX Mode Circular；TX Mode Normal；方向與資料寬度設定為 Byte/Byte、Memory Increment、Very High。
* **改動目的與原因**：
  micro-ROS 採用高頻率的 XRCE-DDS 序列化封包傳送。若使用傳統的 UART Per-byte 中斷（IT 模式），每接收一個 Byte 就會中斷 CPU 一次，將產生極高的 CPU Overrun 負擔並擾動 1kHz PID 控制；使用 Blocking 模式則會阻塞任務。啟用 DMA 後，封包搬移全由硬體 DMA 控制器處理，大幅降低 CPU 佔用率，確保高頻通訊與姿態控制兼得。
* **若維持原樣不改會怎樣**：
  * **會造成的具體問題**：
    1. CPU 算力被通訊大量吃滿（CPU 佔用率飆升至近 100%），無法確保 1kHz 馬達姿態 PID 控制的實時性與精確間隔。
    2. 通訊任務在傳送/接收大封包時發生阻塞 (Blocking)，導致 micro-ROS 封包延遲飆高甚至逾時丟包。
  * **為什麼會發生（底層原因）**：
    micro-ROS 採用高頻 XRCE-DDS 協定發送 JointState 等資料。在無 DMA 的情況下，系統只能使用 Blocking 模式或逐字中斷 (Per-byte IT) 模式。每秒產生上萬次 UART 接收/發送中斷，CPU 必須頻繁執行 ISR 保存與恢復 Context Switch，產生極高 Overrun 負擔，直接擠佔 PID 控制運算資源。
* **受影響程式**：新增 DMA handles、`stm32f4xx_hal_msp.c`、DMA IRQ handlers、micro-ROS transport 封裝。
* **Generate Code 風險**：可能重排 `stm32f4xx_it.c/.h` 與 MSP 初始化；注意 UART Callback 不得在多個檔案重複定義。
* **驗證方式**：執行 Loopback 測試、Buffer Wrap 環形邊界測試、大流量 Burst 傳輸與 Agent 重新啟動測試；檢查無 Overrun/Error 方為合格。

---

## 5. USART2 / DMA NVIC

* **項目**：Transport 通訊中斷優先級管理
* **目前設定**：USART2 global interrupt 停用；無 DMA IRQ。
* **建議設定**：USART2 全域中斷與其 RX/TX DMA IRQ 的 Preemption priority 設為 **6**、subpriority 設為 **0**（比 TIM6 控制中斷的 5 稍微低一階）。
* **CubeMX 路徑**：Connectivity → USART2 → NVIC Settings，以及 System Core → NVIC。
* **修改步驟**：勾選 Enable USART2 global、RX DMA stream、TX DMA stream IRQ；優先級統一填寫 6/0。
* **改動目的與原因**：
  通訊中斷需要處理 HAL UART/DMA 傳送完成、接收環形緩衝區更新以及 Overrun/Noise 錯誤復原。將優先級設為 **6** 有兩大原因：(1) 低於 TIM6 控制中斷 (5)，確保即使通訊發生大量資料傳輸，也永遠不會搶佔或延遲 1kHz 的馬達姿態控制；(2) 位於 FreeRTOS API 可呼叫範圍 (5~15)，允許 DMA 中斷安全地使用 FreeRTOS `FromISR` API 喚醒通訊任務。
* **若維持原樣不改會怎樣**：
  * **會造成的具體問題**：
    1. 一旦 UART 出現線路雜訊或 Overrun Error，通訊中斷會永久鎖死卡死，無法再次收發任何 micro-ROS 封包。
    2. 若將優先級錯設為 0~4（高於 `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`），中斷內一旦呼叫 FreeRTOS API 就會引發 `configASSERT` 造成 MCU 立即崩潰死機。
    3. 若優先級高於 TIM6 (5)，大流量通訊爆發時則會搶佔馬達控制中斷。
  * **為什麼會發生（底層原因）**：
    STM32 HAL 庫要求 UART/DMA 必須有相對應的全域 NVIC 中斷常式來處理 Transfer Complete 與 Error Flags（如 ORE, FE, NE）。沒有開啟 NVIC 中斷，Overrun Error Flag 無法被清除，UART 硬體暫存器會永遠停留在錯誤鎖死狀態。此外，ARM Cortex-M4 的 NVIC 優先級制度極嚴格，數值小於 5 的中斷絕對禁止呼叫 FreeRTOS `FromISR` API。
* **受影響程式**：`stm32f4xx_it.c/.h`、micro-ROS transport callbacks。
* **Generate Code 風險**：若舊 `it_transport.c` 中的 ISR Callback 未清除會造成符號重複；正式啟用新 transport 前舊檔需保持 `#if 0`。
* **驗證方式**：確認每個 IRQ Handler 正確 Dispatch 到對應 HAL handle；人工注入通訊錯誤後確認系統能自動恢復。

---

## 6. FreeRTOS tasks / heap

* **項目**：Task Ownership 整理與 micro-ROS RAM 記憶體規劃
* **目前設定**：`defaultTask` 分配 3000 words (12 KB) 但僅作 delay；`ControlTask` 手動建立 512 words (2 KB)；FreeRTOS `TOTAL_HEAP_SIZE` 為 20 KB；`CommTask` 註解停用且原規劃僅 4 KB。
* **建議設定**：
  1. 將 `defaultTask` 改為 micro-ROS `CommTask`（Stack 設為 3072 words / 12 KB），或重新規劃刪除多餘任務。
  2. 將 FreeRTOS 總 Heap (`TOTAL_HEAP_SIZE`) 初步調大至 **48 KB**（`49152` Bytes）。
  3. 將 `ControlTask` Priority 設為 Native 4、`CommTask` Priority 設為 Native 2，由 CubeMX 統一明確管理。
* **CubeMX 路徑**：Middleware and Software Packs → FREERTOS → Tasks and Queues；Config parameters → Memory Management settings。
* **修改步驟**：規劃 Task Table → 設定各 Task Stack 與 Priority → 調整 Heap 為 49152 → 保持 CMSIS_V2 與 Newlib Reentrant。
* **改動目的與原因**：
  micro-ROS 官方規格明確指出 micro-ROS Task **Stack 必須 >10 KB**，且初始化 Support/Node/Publisher 實體需動態分配大容量記憶體。現有的 20 KB Heap 與 4 KB Task 分配會直接導致記憶體分配失敗 (Allocation Failure) 或 Stack 溢位引發 `HardFault`。調大 Heap 並重組 Task 結構能確保 micro-ROS 具備足夠的記憶體空間安全運作。
* **若維持原樣不改會怎樣**：
  * **會造成的具體問題**：
    1. 當韌體執行 micro-ROS 初始化 (`rclc_support_init` 或建立 Publisher) 時，會立刻因 20 KB Heap 空間不足而回傳宣告失敗，永遠無法與 PC Agent 建立連線。
    2. 若硬切換啟動僅 4 KB 的 `CommTask`，系統會在訊息序列化收發時突發 `HardFault` 崩潰重啟。
  * **為什麼會發生（底層原因）**：
    micro-ROS 的 XRCE-DDS 協議堆疊需要動態配置實體物件（Support, Node, Publisher, Memory Allocator），現有 20 KB 的 FreeRTOS 總 Heap 會在開機初始化時瞬間耗盡。同時，micro-ROS 官方明確規定 Task Stack 必須大於 10 KB；現有的 4 KB Stack 在進行深度函式呼叫與 CDR 封包序列化時，會瞬間踩爆 Task 記憶體邊界 (Stack Overflow)，引發記憶體存取違規與 `HardFault` 死機。
* **受影響程式**：`main.c` 任務屬性、`FreeRTOSConfig.h`、自訂 `MX_FREERTOS_Init`。
* **Generate Code 風險**：CubeMX 會重新產生 Task 入口與 `freertos.c`；業務邏輯必須留在獨立 Module 中，僅在 USER 區呼叫。
* **驗證方式**：檢查所有 Task/Entity 建立時的 Return code、記錄系統最小剩餘 Heap (`xPortGetMinimumEverFreeHeapSize`) 與每任務 Stack High-water mark。

---

## 7. Encoder input filter

* **項目**：TIM1 / TIM4 / TIM5 馬達編碼器雜訊抑制
* **目前設定**：Input Capture IC1/IC2 Filter 均為 0 (未啟用濾波)。
* **建議設定**：根據實測馬達邊界脈衝頻率與 Scope 雜訊量測，設定合適的硬體數位濾波器數值 (Input Filter)。
* **CubeMX 路徑**：Timers → TIM1/TIM4/TIM5 → Parameter Settings → Encoder → Input Filter CH1/CH2。
* **修改步驟**：計算最大 Edge 頻率（Max RPM × PPR × 4 / 60）→ 用示波器量測高頻開關雜訊脈寬 → 選擇能抑制雜訊且不漏抓脈衝的最小 Filter 值 → Generate Code。
* **改動目的與原因**：
  機械臂馬達編碼器線路長且鄰近大電流 PWM 驅動線，容易受到高頻電磁干擾 (EMI) 與開關噪聲影響產生毛刺。在 TIM 硬體層級啟用數位濾波器 (Input Filter) 可以直接在硬體採樣階段濾除高頻波彈，防止錯誤的假 Count 造成角度估算漂移與控制發抖。
* **若維持原樣不改會怎樣**：
  * **會造成的具體問題**：
    1. 馬達靜止時編碼器讀值（Encoder Count）仍會無故跳動或緩慢漂移。
    2. 馬達以 PWM 大電流驅動運轉時，機械臂姿態估算出現嚴重誤差，導致 PID 控制劇烈抖動甚至系統失控暴衝。
  * **為什麼會發生（底層原因）**：
    編碼器 A/B 相訊號線鄰近 PWM 驅動線與大電流馬達線。在 Filter=0（未開啟數位濾波器）時，Timer 硬體會將 PWM 切換產生的高頻電磁干擾 (EMI) 毛刺與開關彈跳誤判為有效的脈衝邊緣 (Edge Count)，導致位置回授出現「假 Count 累積」、角度估計發生漂移。
* **受影響程式**：`MX_TIM1/4/5_Init`，編碼器角度與速度回授。
* **Generate Code 風險**：手動修改生成碼會在下一次生成時被覆蓋，必須在 CubeMX 設定中指定。
* **驗證方式**：馬達靜止時編碼器 Count 不發生任何漂移；最高速正反轉時數值與外部獨立計數器一致。

---

## 8. IWDG（安全鏈完成後）

* **項目**：獨立看門狗硬體安全防護
* **目前設定**：未啟用。
* **建議設定**：在 Phase 4 安全鏈與 Fault 紀錄機制完成後才啟用 IWDG；Timeout 初始設定約 500 ms，僅由系統健康監督點進行刷新餵狗。
* **CubeMX 路徑**：Pinout & Configuration → System Core → IWDG → 勾選 Activated → Parameter Settings。
* **修改步驟**：選擇 LSI 時脈、設定約 500 ms 超時 → 在開機診斷中保存 Reset 原因 → 僅在 Control/Comm Task 健康標誌均成立時刷看門狗。
* **改動目的與原因**：
  當系統因不可預期的軟體 Bug、任務死鎖 (Deadlock)、無窮迴圈或嚴重電磁干擾導致 CPU 失控時，獨立看門狗 (IWDG) 可以在 500ms 內自動強制 MCU 硬體重置 (Reset)，將馬達輸出立即拉回開機安全狀態，防止機械臂失控造成機構損壞或人身危險。
* **若維持原樣不改會怎樣**：
  * **會造成的具體問題**：
    1. 當軟體因非預期 Bug、Task 死鎖或強電磁干擾致使 CPU 卡死時，馬達驅動器會持續維持最後的 PWM/DIR 輸出，導致機械臂暴衝旋轉。
    2. 無法自動救回或強制煞車，極易引發機械碰撞、齒輪損壞或馬達驅動器過熱燒毀事故。
  * **為什麼會發生（底層原因）**：
    沒有啟用獨立看門狗 (IWDG)，系統便缺乏硬體級別的「最後一道防線」。當主晶片 CPU 陷入無窮迴圈或死鎖時，GPIO 腳位輸出電位會保持凍結（保持最後狀態），若此時馬達處於致能運轉狀態，外部硬體無法得知 MCU 已死機，會無限期繼續驅動馬達。
* **受影響程式**：開機診斷 (Startup Diagnostics)、Safety Manager、開機馬達預設狀態。
* **Generate Code 風險**：一經啟用若未及時餵狗會導致無窮重置；在除錯中斷時也會觸發，開發期需建立專用開關。
* **驗證方式**：故意阻塞 `ControlTask` 或 `CommTask`；確認馬達安全切斷、記錄到看門狗 Reset 原因且未發生無窮重置迴圈。

---

## 9. 清理未使用 TIM / oscillator pins（低優先）

* **項目**：清理未引用的周邊與時脈預留腳位
* **目前設定**：TIM8、TIM13、PC5 等週邊在 CubeMX 中已初始化但應用程式無任何引用；SYSCLK 仍使用內部 HSI。
* **建議設定**：在硬體圖紙 (Schematic) 確認無預留用途後，在 CubeMX 中將未使用的 TIM8、TIM13 設為 Disable，釋放相關資源。
* **CubeMX 路徑**：Timers → TIM8 / TIM13 → Disable；System Core → RCC。
* **修改步驟**：確認硬體圖紙 → 每次停用一組週邊 → Generate Code → 進行完整功能迴歸測試。
* **改動目的與原因**：
  清理無用的週邊初始化可以：(1) 減少系統開機時不必要的 HAL 暫存器配置時間；(2) 釋放被無謂佔用的 MCU 腳位與內部 Timer 資源；(3) 簡化 generated code，避免未來開發者對硬體配置產生誤解或發生週邊中斷衝突。
* **若維持原樣不改會怎樣**：
  * **會造成的具體問題**：
    1. 增加 MCU 開機時不必要的 HAL 初始化時間與 Flash 記憶體佔用。
    2. 被佔用卻未使用的腳位可能在未來新功能擴充時造成 Pin Multiplexing (復用) 衝突，或讓後續維護者對硬體架構產生誤解。
  * **為什麼會發生（底層原因）**：
    CubeMX 產生的 `MX_TIM8_Init` 與 `MX_TIM13_Init` 會無謂地寫入週邊暫存器並配置外設時脈。這些死程式碼 (Dead Code) 佔據 Flash 空間，且佔用了周邊暫存器與記憶體 Mapping，增加維護複雜度。
* **受影響程式**：`main.c` 初始化流程、MSP 設定、Pin Mux 配置。
* **Generate Code 風險**：原本自動生成的初始化函式與 Handle 會消失，若 USER 程式碼中有殘留引用會導致編譯失敗。
* **驗證方式**：執行 Debug 與 Release 雙組別 Clean Build，並進行全軸運動與 homing 回歸測試。
