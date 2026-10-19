# micro-ROS 通訊底層開發除錯血淚史 (Post-Mortem & Lessons Learned)

> [!IMPORTANT]
> 本 post-mortem 已由後續 T15–T27 實機工作補充：ABI static-library rebuild、
> Windows VCP bridge、RXNE IRQ ring、QoS/baud 調整與 fail-closed session binding。

**專案**：Calligraphy-arm-Firmware  
**文件位置**：`docs/ros2_integration/03_microros_debug_post_mortem.md`  
**建立日期**：2026-07-30  
**目標**：紀錄我們在建立 STM32F446RE 與 WSL2 micro-ROS Agent 通訊時，所遭遇的 4 大史詩級 Bug，以及我們是如何一步步抽絲剝繭解決它們的。這份文件是未來除錯的重要避雷針。

---

## 📅 開發歷程摘要

在微控制器上運行 ROS 2 (micro-ROS) 是一項極具挑戰的任務，因為它橫跨了**硬體底層 (DMA/UART)**、**RTOS (作業系統與中斷)**、以及**電腦端的虛擬化環境 (WSL2/Docker/USB 轉接)**。

我們在開發過程中的症狀始終只有一個：「**Agent 沒有收到資料，連線超時**」。
但為了解決這個單一症狀，我們實際上打敗了隱藏在不同層級的 4 個終極大魔王。

---

## 👾 魔王一：FPU 浮點運算引發的「開機即猝死」(HardFault)

* **發生場景**：當我們好不容易寫好狀態機，把程式燒進板子後，發現板子連第一行 Log 都印不出來，甚至連 1kHz 的定時器都不會閃燈了。
* **診斷過程**：
  透過 STM32CubeIDE 的 Debugger 暫停程式，我們發現程式死在了 `HardFault_Handler`（硬體錯誤中斷）。查看 Call Stack 發現，當程式呼叫 micro-ROS 的時間同步或數學運算時，晶片就崩潰了。
* **根本原因 (Root Cause)**：
  micro-ROS 官方編譯的靜態函式庫 (`libmicroros.a`) 是使用 `softfp` (軟硬體浮點混合 ABI) 編譯的，但我們的 STM32CubeIDE 專案預設卻沒有對齊這個標準，導致晶片在處理浮點指令時發生暫存器存取違規。
* **解決方案**：
  我們深入修改了 `.cproject` 與 `cflags.mk`，強制將編譯參數統一為 `-mfloat-abi=softfp` 與 `-mfpu=fpv4-sp-d16`。
* **學習點**：整合第三方編譯好的 C/C++ Library 時，ABI (Application Binary Interface) 必須做到位元組級別的絕對匹配。

---

## 👾 魔王二：DMA 接收緩衝區的「暴力洗白」 (Ring Buffer Amnesia)

* **發生場景**：晶片不再當機了，也會定時送出 Ping 封包。但 WSL2 的 Agent 雖然顯示有收到，卻遲遲無法建立 Session (`session established`)，最終超時。
* **診斷過程**：
  我們深入追蹤了 `dma_transport.c` 中的 `cubemx_transport_read` 函式。我們發現，只要實體 UART 線路上有一丁點雜訊引發硬體警告（例如 Framing Error 或 Noise Error），舊的程式碼就會執行 `HAL_UART_DMAStop()`，並把緩衝區指針 `dma_head` 和 `dma_tail` 歸零。
* **根本原因 (Root Cause)**：
  我們的 RX 是採用 **Circular DMA (環形持續接收模式)**。如果隨意歸零指標，那些「已經收到、但還沒被 micro-ROS 處理」的合法封包，就會被當成垃圾丟棄或被後續資料覆寫。
* **解決方案**：
  修改錯誤復原邏輯：遇到 ORE/NE/FE 錯誤時，**只清除硬體警告旗標 (`__HAL_UART_CLEAR_FLAG`)**，絕對不暫停 DMA 也絕不修改指標。
* **學習點**：在 Circular 模式下，硬體會自己照顧好資料流，軟體千萬不要自作聰明去打斷或重置狀態。

---

## 👾 魔王三：殭屍容器大軍的「封包瓜分」 (Zombie Docker Contention)

* **發生場景**：修好 DMA 後，我們滿懷期待地測試，結果 Agent 畫面上完全沒有印出任何東西！甚至加了 `-v6` (最高詳細日誌) 也像死水一樣。
* **診斷過程**：
  我們用 Debugger 確認 MCU 發送端 `HAL_UART_Transmit_DMA` 完美回傳 `HAL_OK`。然後我們使用 Linux 底層指令 `stty` 與 `xxd` 去傾聽 `/dev/ttyACM0`，發現實體層確實有收到 `0x7e` 開頭的封包。為什麼 Agent 看不到？
  執行 `docker ps` 後，真相大白：**背景居然同時活著 6 個 micro-ROS Agent！**
* **根本原因 (Root Cause)**：
  在 Linux 系統中，如果有多個行程 (Process) 同時讀取同一個 TTY (序列埠)，Linux 不會複製資料給所有人，而是會把字元「隨機分配」給它們。這導致 6 個 Agent 每個都只拿到破碎的片段，沒有人能解碼出完整的封包。
* **解決方案**：
  執行 `docker rm -f $(docker ps -aq)` 殺死所有背景殭屍容器。並在後續的啟動指令加上 `--rm` 確保容器結束後自動銷毀。
* **學習點**：Linux 的 TTY 設備是獨佔性質的。當通訊沒有回應時，永遠要先檢查是不是有其他背景程式（或 Zombie Container）在偷聽串口。

---

## 👾 魔王四：Windows 偵錯器的「物理截胡」 (The VCP Hijacking Illusion)

* **發生場景**：為了解決前述問題，我們在 Windows 的 STM32CubeIDE 按下了 Debug (小蟲子) 按鈕來監看變數。卻發現 MCU 的程式會卡死在等待 Agent 回應的地方，以為是 MCU 發生了死鎖。
* **診斷過程**：
  在 Debug 模式下，我們發現 `osDelay(1)` 迴圈順利結束了，發送完全正常，但就是收不到回音。
* **根本原因 (Root Cause)**：
  這是硬體直通 (USB Passthrough) 的盲點。當 STM32CubeIDE 啟動 Debug 時，它會強行將 ST-LINK 的 USB 控制權**從 WSL2 搶回 Windows**。因此，MCU 送出的 Ping 封包全部流向了 Windows，WSL2 裡的 Agent 自然收不到，MCU 等不到回音就會判定超時。
* **解決方案**：
  建立雙向切換標準作業 SOP：
  - **從 Windows 切換給 WSL2 (測試 micro-ROS 通訊時)**：先結束 IDE Debug，並在 PowerShell 執行：
    ```powershell
    usbipd attach --wsl --busid <BUSID>
    ```
  - **從 WSL2 切回 Windows (要在 CubeIDE 中除錯或燒錄時)**：在 PowerShell 執行：
    ```powershell
    usbipd detach --busid <BUSID>
    usbipd unbind --busid <BUSID>
    ```
    *(必做！`usbipd unbind` 能解開 stub 轉接驅動並還原原生的 Windows ST-LINK 驅動，否則 CubeIDE 會報錯看不到 ST-LINK)*。
* **學習點**：在跨虛擬機 (WSL2) 的硬體開發中，除了看程式碼，更要隨時意識到「物理線路現在到底接在哪個作業系統上」。

---

## 💡 最終心得總結

這次除錯之旅完美詮釋了「嵌入式系統除錯的立體性」。我們從：
1. **編譯器層 (ABI)**
2. **驅動層 (DMA Ring Buffer)**
3. **作業系統層 (Linux TTY 獨佔性)**
4. **虛擬化層 (usbipd 裝置綁定)**

一路過關斬將。這些經驗不僅證明了目前的底層架構非常穩固，也為未來團隊在擴展 ROS 2 功能時，提供了最寶貴的避坑指南。
