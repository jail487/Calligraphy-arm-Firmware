# DogHead 機械手臂 - ROS2 整合測試流程

## 🎯 測試目標
驗證 STM32 (micro-ROS) ↔ ROS2 Agent ↔ ROS2 Host 的完整通訊鏈路

---

## 📋 測試前準備清單

### 硬體準備
- [x] STM32F446RET6 開發板
- [x] USB 傳輸線（Type-C 或 Micro-USB）
- [ ] 馬達驅動電路（可選，僅測試通訊時不需要）

### 軟體準備
- [x] ROS2 Humble 已安裝
- [x] micro-ROS Agent Docker Image 已下載
- [x] STM32 韌體已包含 micro-ROS 初始化程式碼

---

## 🧪 測試步驟

### 階段 1：驗證 Docker 環境

#### 1.1 測試 Agent 啟動（無 STM32）
```bash
cd /home/jail487/doghead/dogArm-project

# 應該會看到錯誤（因為沒有連接裝置），這是正常的
docker run -it --rm --net=host \
    microros/micro-ros-agent:humble \
    udp4 --port 8888 -v6
```
**預期輸出**：Agent 啟動並監聽 UDP 8888 埠

#### 1.2 測試 ROS2 環境
```bash
# Terminal 1
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 2
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /chatter
```
**預期輸出**：能看到 `/chatter` topic 並接收訊息

---

### 階段 2：連接 STM32 並驗證串列埠

#### 2.1 連接 STM32
1. 將 STM32 通過 USB 連接到電腦
2. 確認串列埠出現：
```bash
ls -l /dev/tty* | grep -E "USB|ACM"
# 應該會看到 /dev/ttyACM0 或類似裝置
```

#### 2.2 測試串列埠通訊
```bash
# 安裝 screen
sudo apt install screen

# 測試連線（115200 是 UART2 的鮑率）
screen /dev/ttyACM0 115200

# 如果 STM32 有 printf 輸出，會在這裡顯示
# 按 Ctrl+A 然後按 K 退出
```

#### 2.3 設定權限
```bash
# 暫時授權
sudo chmod 666 /dev/ttyACM0

# 或永久加入群組（需登出重登）
sudo usermod -aG dialout $USER
```

---

### 階段 3：啟動 micro-ROS Agent

#### 3.1 使用快速腳本
```bash
cd /home/jail487/doghead/dogArm-project
./scripts/start_microros_agent.sh /dev/ttyACM0 115200
```

#### 3.2 觀察 Agent 日誌
**成功連線的跡象**：
```
[1641234567.123456] info     | Root.cpp           | ...
[1641234567.234567] info     | SessionManager.cpp | create_session
[1641234567.345678] info     | ProxyClient.cpp    | create_client
```

**連線失敗的跡象**：
```
[ERROR] Cannot open serial port
```
→ 檢查串列埠名稱和權限

---

### 階段 4：驗證 ROS2 通訊

#### 4.1 列出 Topic（在另一個終端機）
```bash
source /opt/ros/humble/setup.bash
ros2 topic list

# 預期看到（取決於 STM32 實作）：
# /joint_states
# /parameter_events
# /rosout
```

#### 4.2 監聽關節狀態
```bash
ros2 topic echo /joint_states

# 預期輸出（如果 STM32 有發布）：
# header:
#   stamp:
#     sec: 1641234567
#     nanosec: 123456789
# name:
# - joint1
# - joint2
# position:
# - 0.0
# - 0.0
# velocity:
# - 0.0
# - 0.0
```

#### 4.3 發送測試指令
```bash
# 發布速度指令（假設 STM32 有訂閱 /cmd_vel）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 觀察 STM32 反應（馬達應該會動）
```

---

### 階段 5：使用 Rviz 視覺化

#### 5.1 啟動 Rviz
```bash
source /opt/ros/humble/setup.bash
rviz2
```

#### 5.2 添加顯示項目
1. 點擊左下角 `Add`
2. 選擇 `By topic` → `/joint_states` → `JointState`
3. 觀察關節數據更新

---

## 🐛 故障排除指南

### 問題 1：找不到 /dev/ttyACM0
**原因**：STM32 未正確連接或驅動問題

**解決**：
```bash
# 檢查 USB 裝置
lsusb | grep -i stm

# 查看 kernel 訊息
dmesg | tail -20

# 如果出現 /dev/ttyUSB0 而非 ttyACM0
./scripts/start_microros_agent.sh /dev/ttyUSB0 115200
```

### 問題 2：Agent 啟動但 `ros2 topic list` 沒有東西
**原因**：STM32 韌體可能沒有正確初始化 micro-ROS

**檢查項目**：
1. STM32 韌體中是否呼叫了 `rmw_uros_set_custom_transport()`
2. FreeRTOS 任務是否正確啟動
3. UART 鮑率是否匹配（STM32: 115200, Agent: 115200）

**除錯方法**：
```bash
# 在 STM32 的 CommTask 加入診斷輸出
printf("micro-ROS init start\n");
// ... 初始化程式碼 ...
printf("micro-ROS init done\n");
```

### 問題 3：Topic 有出現但沒有資料
**原因**：Publisher 頻率過低或未正確發布

**檢查**：
```bash
# 查看 Topic 資訊
ros2 topic info /joint_states

# 查看 Topic 頻率
ros2 topic hz /joint_states

# 預期：10-100 Hz
```

### 問題 4：Docker 容器無法存取串列埠
**解決**：
```bash
# 確認裝置映射正確
docker run -it --rm --net=host \
    --device=/dev/ttyACM0:/dev/ttyACM0:rw \  # 注意 :rw
    microros/micro-ros-agent:humble \
    serial --dev /dev/ttyACM0 -b 115200 -v6
```

---

## 📊 效能基準

### 通訊延遲測試
```bash
# 測量 Round-Trip Time (RTT)
ros2 run demo_nodes_cpp listener &
ros2 topic pub /chatter std_msgs/msg/String "data: 'test'" -r 100

# 使用 ros2 topic hz 測量實際頻率
ros2 topic hz /joint_states
```

**預期效能**：
- UART @ 115200 bps：約 20-50 Hz 更新率
- UART @ 921600 bps：約 100-200 Hz 更新率
- 延遲：5-20ms（取決於訊息大小）

---

## ✅ 測試成功標準

- [ ] Agent 能正常啟動並連接 STM32
- [ ] `ros2 topic list` 能看到 `/joint_states` 或自訂 topic
- [ ] `ros2 topic echo` 能接收到即時資料
- [ ] 從 ROS2 發送指令後，STM32 能正確接收並執行
- [ ] 通訊延遲 < 50ms
- [ ] 無資料遺失或亂碼

---

## 🎓 下一步

測試通過後，可以進行：
1. **實作自訂訊息**：定義 `doghead_msgs` 套件
2. **增加 Service 呼叫**：如 `/set_position`
3. **整合 Action Server**：實作 `/write_character` 動作
4. **搭配 Rviz**：視覺化機械手臂狀態
5. **開發上層控制節點**：Python/C++ 控制邏輯

---

## 📁 相關檔案位置

- Agent 腳本：`/home/jail487/doghead/dogArm-project/scripts/start_microros_agent.sh`
- Docker Compose：`/home/jail487/doghead/dogArm-project/docker-compose.yml`
- STM32 韌體：`/home/jail487/doghead/dogArm-project/Firmware/doghead_arm/`
- 測試指南：本檔案
