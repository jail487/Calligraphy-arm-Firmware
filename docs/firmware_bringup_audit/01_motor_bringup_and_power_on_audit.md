# 馬達 Bringup 實機通電與驗收綜合審查報告

> [!WARNING]
> 這份文件只描述馬達帶電啟動的放行條件，不代表目前已允許通電運轉。
> 現行映像為 gate=0；Phase 3 前仍需確認 motor/driver、encoder、E-stop 與機械限制。

**最後更新**：2026-07-29  
**審查對象**：三軸驅動介面 (`mainpp.cpp` / `robot_interface.h` / `safety_manager.c`)  
**審查結論**：**通過單軸無負載極低速 Conditional GO 驗收**

---

## 1. 馬達通電前與 Bringup 審查記錄

### 1.1 介面與資料結構驗收
- `RobotCommand_t` 消除歷史重複欄位與 padding 不對齊風險，明確區分 Normal 模式與 Single-Axis Test 模式。
- `Mock_TestCommand_Producer()` 具備 C linkage 封裝，寫入經逾時 (250ms) 管理的 Static Queue，Producer 停止更新後 250ms 馬達會自動停機。
- `ApplyCommand()` 嚴格限定 Test RPM 範圍在 ±100 RPM 內，任何非法的 axis 或超速命令將觸發 `ROBOT_FAULT_INVALID_COMMAND`。

### 1.2 物理隔離與硬體控制驗收
- 測試模式下採用 `enable_mask` 控制。未被選定的軸不僅轉速歸零，亦透過 `Motor_Stop()` 或將 Enable PIN 拉低進行物理硬體隔離。
- 致動器最終輸出前一列均強制執行 **Ultimate Safety Gate**（`std::isfinite` 有限性檢查、150 RPM 限幅與 `RobotSafety_CanDrive()` 檢查）。

---

## 2. 通電測試放行範圍與前置條件

### 2.1 嚴格限制範圍
- **僅允許**：單一馬達、無連桿/皮帶/負載、10～20 RPM 極低速起步之短時間 Bench Test。
- **嚴禁放行**：多軸同時旋轉、帶負載運轉、全自動軌跡/IK 測試或無人監看測試。

### 2.2 第一次通電必要清單 (Checklist)
1. 目標 ARM toolchain clean build 成功，0 Error, 0 Warning。
2. 馬達與連桿、皮帶完全分離。
3. 僅對被測軸驅動器供電，其餘兩軸驅動器實體斷電。
4. 使用可限流電源供應器，設有保守電流上限。
5. 實體 E-stop 緊急斷電開關必須在操作人員手邊。
