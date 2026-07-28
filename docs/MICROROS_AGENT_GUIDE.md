# micro-ROS Agent 使用指南

## 🚀 快速啟動

### 方法 1：使用腳本（推薦）
```bash
cd /home/jail487/doghead/dogArm-project
./scripts/start_microros_agent.sh [串列埠] [鮑率]

# 範例
./scripts/start_microros_agent.sh /dev/ttyACM0 115200
```

### 方法 2：使用 Docker Compose
```bash
cd /home/jail487/doghead/dogArm-project

# 啟動 Agent
docker-compose up microros-agent

# 背景運行
docker-compose up -d microros-agent

# 停止
docker-compose down
```

### 方法 3：手動運行 Docker 命令
```bash
docker run -it --rm \
    --net=host \
    --device=/dev/ttyACM0:/dev/ttySTM32:rw \
    microros/micro-ros-agent:humble \
    serial --dev /dev/ttySTM32 -b 115200 -v6
```

## 📋 前置檢查

### 1. 確認串列埠
```bash
# 列出所有 USB 串列埠
ls -l /dev/tty* | grep -E "USB|ACM"

# 常見埠：
# - /dev/ttyACM0  (STM32 Virtual COM Port)
# - /dev/ttyUSB0  (USB-UART 轉接器)
```

### 2. 設定串列埠權限
```bash
# 將使用者加入 dialout 群組（永久）
sudo usermod -aG dialout $USER
# 登出後重新登入生效

# 暫時授權（重開機失效）
sudo chmod 666 /dev/ttyACM0
```

### 3. 測試 STM32 連線
```bash
# 安裝 minicom
sudo apt install minicom

# 測試串列埠
minicom -D /dev/ttyACM0 -b 115200
```

## 🔧 Agent 參數說明

### UART/Serial 模式
```bash
docker run -it --rm --net=host \
    --device=/dev/ttyACM0 \
    microros/micro-ros-agent:humble \
    serial --dev /dev/ttyACM0 \
           -b 115200 \        # 鮑率
           -v6                # 詳細日誌（1-6，越大越詳細）
```

### UDP 模式（若使用網路傳輸）
```bash
docker run -it --rm --net=host \
    microros/micro-ros-agent:humble \
    udp4 --port 8888 -v6
```

## 🧪 驗證 Agent 運作

### 在另一個終端機檢查 ROS2 Topic
```bash
# 確保已 source ROS2 環境
source /opt/ros/humble/setup.bash

# 列出所有 Topic（應該會看到 STM32 發布的 Topic）
ros2 topic list

# 監聽特定 Topic
ros2 topic echo /joint_states

# 查看節點資訊
ros2 node list
ros2 node info /micro_ros_agent
```

## 🐛 常見問題排除

### 問題 1：找不到串列埠
```bash
# 檢查 USB 裝置
lsusb | grep STM

# 檢查 dmesg 日誌
dmesg | grep tty
```

### 問題 2：權限被拒
```bash
# 方法 1：暫時授權
sudo chmod 666 /dev/ttyACM0

# 方法 2：加入群組（需重新登入）
sudo usermod -aG dialout $USER
```

### 問題 3：Agent 連接但無數據
- 確認 STM32 韌體已正確初始化 micro-ROS
- 檢查 UART 鮑率是否一致（STM32 與 Agent 都要 115200）
- 查看 Agent 日誌（-v6 模式）

### 問題 4：ROS2 節點無法發現
- 確認使用 `--net=host`（Docker 需要主機網路模式）
- 檢查防火牆設定
- 嘗試設定 ROS_DOMAIN_ID：
  ```bash
  export ROS_DOMAIN_ID=0
  ```

## 📊 進階使用

### 搭配 Rviz 視覺化
```bash
# Terminal 1: 啟動 Agent
./scripts/start_microros_agent.sh

# Terminal 2: 啟動 Rviz
source /opt/ros/humble/setup.bash
rviz2
```

### 錄製數據（ROS2 Bag）
```bash
# 錄製所有 Topic
ros2 bag record -a

# 錄製特定 Topic
ros2 bag record /joint_states /motor_cmd
```

### 自訂 Agent 配置
編輯 `docker-compose.yml` 中的 `command` 欄位：
```yaml
command: serial --dev /dev/ttySTM32 -b 921600 -v6  # 更高鮑率
```

## 🔗 相關資源
- [micro-ROS 官方文檔](https://micro.ros.org/)
- [STM32 + micro-ROS 教學](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils)
- [ROS2 Humble 文檔](https://docs.ros.org/en/humble/)
