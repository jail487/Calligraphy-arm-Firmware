#!/bin/bash
# micro-ROS Agent 啟動腳本
# 用途：在 Docker 中運行 micro-ROS Agent，連接 STM32 與 ROS2

set -e

# 顏色輸出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  micro-ROS Agent Docker 啟動工具${NC}"
echo -e "${GREEN}========================================${NC}"

# 預設參數
MODE_OR_PORT="${1:-/dev/ttyACM0}"

# Bridge 模式處理 (TCP 轉接模式)
if [ "$MODE_OR_PORT" = "bridge" ] || [ "$MODE_OR_PORT" = "--bridge" ]; then
    TCP_HOST="${2:-host.docker.internal}"
    TCP_PORT="${3:-8888}"
    BAUDRATE="${4:-460800}"

    echo -e "${GREEN}[INFO] 啟動 micro-ROS Agent (TCP Bridge 模式)...${NC}"
    echo -e "${YELLOW}[INFO] TCP Bridge 目標: ${TCP_HOST}:${TCP_PORT}${NC}"
    echo -e "${YELLOW}[INFO] 鮑率: ${BAUDRATE}${NC}"
    echo -e "${YELLOW}提示：按 Ctrl+C 停止${NC}"
    echo ""

    docker run -i --rm \
        --name doghead_microros_agent_bridge \
        --net=host \
        --add-host host.docker.internal:host-gateway \
        --entrypoint /bin/sh \
        microros/micro-ros-agent:humble \
        -c "
            which socat > /dev/null || (apt-get update && apt-get install -y socat)
            echo '[INFO] 建立 PTY /dev/ttySTM32 -> tcp:${TCP_HOST}:${TCP_PORT}...'
            socat pty,link=/dev/ttySTM32,raw,echo=0 tcp:${TCP_HOST}:${TCP_PORT} &
            SOCAT_PID=\$!

            for i in \$(seq 1 10); do
                if [ -e /dev/ttySTM32 ] && kill -0 \$SOCAT_PID 2>/dev/null; then
                    break
                fi
                sleep 0.5
            done

            if [ ! -e /dev/ttySTM32 ] || ! kill -0 \$SOCAT_PID 2>/dev/null; then
                echo '[ERROR] socat 建立 PTY 連線失敗 (tcp:${TCP_HOST}:${TCP_PORT})' >&2
                exit 1
            fi

            echo '[INFO] 啟動 micro_ros_agent 可執行檔...'
            . /opt/ros/humble/setup.sh 2>/dev/null || true
            . /uros_ws/install/setup.sh 2>/dev/null || true
            exec /uros_ws/install/micro_ros_agent/lib/micro_ros_agent/micro_ros_agent serial --dev /dev/ttySTM32 -b ${BAUDRATE} -v6
        "
    echo -e "${GREEN}[INFO] micro-ROS Agent (Bridge 模式) 已停止${NC}"
    exit 0
fi

# 直連硬體模式 (Direct Hardware Serial Mode)
SERIAL_PORT="${MODE_OR_PORT}"
BAUDRATE="${2:-460800}"

echo -e "${YELLOW}[INFO] 串列埠: ${SERIAL_PORT}${NC}"
echo -e "${YELLOW}[INFO] 鮑率: ${BAUDRATE}${NC}"

# 檢查串列埠是否存在
if [ ! -e "$SERIAL_PORT" ]; then
    echo -e "${RED}[ERROR] 串列埠 ${SERIAL_PORT} 不存在！${NC}"
    echo -e "${YELLOW}可用的串列埠：${NC}"
    ls -l /dev/tty* | grep -E "USB|ACM" || echo "未找到 USB 串列埠"
    exit 1
fi

# 檢查權限
if [ ! -r "$SERIAL_PORT" ] || [ ! -w "$SERIAL_PORT" ]; then
    echo -e "${YELLOW}[WARN] 沒有串列埠權限，嘗試加入 dialout 群組...${NC}"
    echo "請執行: sudo usermod -aG dialout $USER"
    echo "然後登出重新登入"
    echo ""
    echo -e "${YELLOW}暫時解決：${NC}"
    echo "sudo chmod 666 $SERIAL_PORT"
fi

echo -e "${GREEN}[INFO] 啟動 micro-ROS Agent (直連 UART 模式)...${NC}"
echo -e "${YELLOW}提示：按 Ctrl+C 停止${NC}"
echo ""

# 運行 Docker 容器
# --rm: 容器停止後自動刪除
# --net=host: 使用主機網路（讓 ROS2 可以發現）
# --device: 映射串列埠到容器內
docker run -i --rm \
    --name doghead_microros_agent \
    --net=host \
    --device=${SERIAL_PORT}:/dev/ttySTM32:rw \
    microros/micro-ros-agent:humble \
    serial --dev /dev/ttySTM32 -b ${BAUDRATE} -v6

echo -e "${GREEN}[INFO] micro-ROS Agent 已停止${NC}"
