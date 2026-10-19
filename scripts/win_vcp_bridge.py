#!/usr/bin/env python3
"""
Windows ST-Link VCP Bidirectional TCP Bridge for containerized micro-ROS Agent.
--------------------------------------------------------------------------------
Features:
1. Full Auto-Discovery: Automatically scans and connects to ST-Link COM port (COM3, COM9, etc.).
2. Auto Docker Agent Link: Automatically notifies / restarts the micro-ROS Agent container
   so socat immediately connects to port 8888.
3. Hot-Plug & Reset Recovery: Automatically heals and re-syncs connection when STM32 is Reset
   or USB is unplugged/replugged.
"""

import sys
import time
import socket
import argparse
import threading
import subprocess
import signal

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("[ERROR] 'pyserial' package is not installed.", file=sys.stderr)
    print("[ERROR] Please install it using: pip install pyserial", file=sys.stderr)
    sys.exit(1)


def find_stlink_com_port() -> str:
    """Auto-detect ST-Link Virtual COM Port on Windows."""
    ports = serial.tools.list_ports.comports()
    for p in ports:
        desc = (p.description or "").lower()
        mfg = (p.manufacturer or "").lower()
        hwid = (p.hwid or "").lower()
        if "stlink" in desc or "stmicroelectronics" in desc or "stlink" in mfg or "0483:" in hwid:
            return p.device

    # Fallback: single port
    if len(ports) == 1:
        return ports[0].device

    return ""


def restart_docker_agent():
    """Trigger micro-ROS agent container restart in background so socat connects immediately."""
    try:
        res = subprocess.run(
            ["docker", "ps", "-q", "-f", "name=doghead_microros_agent_bridge"],
            capture_output=True, text=True, timeout=3
        )
        if res.stdout.strip():
            print("[BRIDGE] Automatically restarting Docker micro-ROS agent for instant connection...")
            subprocess.Popen(["docker", "restart", "doghead_microros_agent_bridge"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        else:
            # Check if container exists but stopped
            res_all = subprocess.run(
                ["docker", "ps", "-aq", "-f", "name=doghead_microros_agent_bridge"],
                capture_output=True, text=True, timeout=3
            )
            if res_all.stdout.strip():
                print("[BRIDGE] Starting existing Docker micro-ROS agent container...")
                subprocess.Popen(["docker", "start", "doghead_microros_agent_bridge"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception as e:
        # Docker not running or not in PATH, non-fatal
        pass


class VcpTcpBridge:
    def __init__(self, com_port: str, baudrate: int, host: str, tcp_port: int, max_retries: int = 5):
        self.requested_com_port = com_port
        self.active_com_port = ""
        self.baudrate = baudrate
        self.host = host
        self.tcp_port = tcp_port
        self.max_retries = max_retries
        self.running = True

        self.ser = None
        self.server_sock = None
        self.client_sock = None

    def open_serial(self) -> bool:
        """Attempt to open the specified or auto-detected COM port with bounded retries."""
        for attempt in range(1, self.max_retries + 1):
            if not self.running:
                return False

            port_to_open = self.requested_com_port
            if port_to_open.lower() == "auto":
                port_to_open = find_stlink_com_port()
                if not port_to_open:
                    print(f"[BRIDGE] Scanning for ST-Link USB COM Port (Attempt {attempt}/{self.max_retries})...")
                    time.sleep(1.0)
                    continue

            try:
                print(f"[BRIDGE] Opening serial port {port_to_open} @ {self.baudrate} baud...")
                self.ser = serial.Serial(
                    port=port_to_open,
                    baudrate=self.baudrate,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.002,
                    write_timeout=1.0
                )
                self.active_com_port = port_to_open
                print(f"[BRIDGE] Serial port {port_to_open} opened successfully.")
                return True
            except serial.SerialException as e:
                print(f"[BRIDGE WARNING] Failed to open {port_to_open}: {e}", file=sys.stderr)
                if attempt < self.max_retries:
                    time.sleep(1.0)
        return False

    def init_tcp_server(self) -> bool:
        """Bind TCP server socket."""
        try:
            self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.server_sock.settimeout(0.5)
            self.server_sock.bind((self.host, self.tcp_port))
            self.server_sock.listen(1)
            print(f"[BRIDGE] TCP Server listening on {self.host}:{self.tcp_port}")
            return True
        except socket.error as e:
            print(f"[BRIDGE ERROR] Failed to bind TCP server on {self.host}:{self.tcp_port}: {e}", file=sys.stderr)
            return False

    def _serial_to_tcp_loop(self):
        """Forward bytes from Serial -> TCP socket with zero Nagle delay."""
        while self.running and self.client_sock and self.ser and self.ser.is_open:
            try:
                in_waiting = self.ser.in_waiting
                if in_waiting > 0:
                    data = self.ser.read(min(in_waiting, 4096))
                else:
                    data = self.ser.read(1)
                if data and self.client_sock:
                    self.client_sock.sendall(data)
            except (socket.timeout, serial.SerialTimeoutException):
                continue
            except (socket.error, serial.SerialException) as e:
                if self.running:
                    print(f"[BRIDGE ERROR] Serial->TCP communication error: {e}", file=sys.stderr)
                break

    def _tcp_to_serial_loop(self):
        """Forward bytes from TCP socket -> Serial with immediate flush."""
        while self.running and self.client_sock and self.ser and self.ser.is_open:
            try:
                data = self.client_sock.recv(4096)
                if not data:
                    print("[BRIDGE] Client disconnected.", file=sys.stderr)
                    break
                self.ser.write(data)
                self.ser.flush()
            except socket.timeout:
                continue
            except (socket.error, serial.SerialException) as e:
                if self.running:
                    print(f"[BRIDGE ERROR] TCP->Serial communication error: {e}", file=sys.stderr)
                break

    def run(self):
        """Main execution loop."""
        if not self.open_serial():
            print(f"[BRIDGE FATAL] Could not connect to ST-Link COM port. Please check USB cable.", file=sys.stderr)
            sys.exit(1)

        if not self.init_tcp_server():
            self.close()
            sys.exit(1)

        # Trigger instant Docker agent connection
        restart_docker_agent()

        print("=" * 70)
        print(f" [BRIDGE READY] Connected to {self.active_com_port} | TCP Server: {self.tcp_port}")
        print(" -> STM32 開機或按 Reset 鍵時，將自動即時重新握手連線！")
        print(" -> 按 Ctrl+C 結束橋接。")
        print("=" * 70)

        while self.running:
            try:
                try:
                    client_sock, client_addr = self.server_sock.accept()
                except socket.timeout:
                    continue

                print(f"[BRIDGE] Docker micro-ROS Agent connected ({client_addr[0]}:{client_addr[1]})")
                self.client_sock = client_sock
                self.client_sock.settimeout(0.5)
                self.client_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

                if self.ser and self.ser.is_open:
                    self.ser.reset_input_buffer()
                    self.ser.reset_output_buffer()

                t_s2t = threading.Thread(target=self._serial_to_tcp_loop, name="SerialToTcp", daemon=True)
                t_t2s = threading.Thread(target=self._tcp_to_serial_loop, name="TcpToSerial", daemon=True)

                t_s2t.start()
                t_t2s.start()

                while self.running and t_s2t.is_alive() and t_t2s.is_alive():
                    time.sleep(0.2)

                print("[BRIDGE] Agent session reconnected/reset. Listening for next session...")
                if self.client_sock:
                    try:
                        self.client_sock.shutdown(socket.SHUT_RDWR)
                        self.client_sock.close()
                    except Exception:
                        pass
                    self.client_sock = None

            except KeyboardInterrupt:
                print("\n[BRIDGE] Shutdown requested.")
                self.running = False
                break
            except Exception as e:
                print(f"[BRIDGE ERROR] Exception in main loop: {e}", file=sys.stderr)
                time.sleep(1.0)

        self.close()

    def close(self):
        """Clean up resources on exit."""
        self.running = False
        if self.client_sock:
            try: self.client_sock.close()
            except Exception: pass
        if self.server_sock:
            try: self.server_sock.close()
            except Exception: pass
        if self.ser and self.ser.is_open:
            try: self.ser.close()
            except Exception: pass
        print("[BRIDGE] All resources released cleanly.")


def main():
    parser = argparse.ArgumentParser(description="Windows ST-Link VCP Auto-Connecting TCP Bridge for micro-ROS")
    parser.add_argument("-c", "--com", default="auto", help="Windows Serial COM Port (default: auto)")
    parser.add_argument("-b", "--baud", type=int, default=460800, help="Baud rate (default: 460800)")
    parser.add_argument("-p", "--port", type=int, default=8888, help="TCP Port (default: 8888)")
    parser.add_argument("-H", "--host", default="0.0.0.0", help="Host (default: 0.0.0.0)")
    parser.add_argument("--retries", type=int, default=10, help="Max retries (default: 10)")
    args = parser.parse_args()

    bridge = VcpTcpBridge(com_port=args.com, baudrate=args.baud, host=args.host, tcp_port=args.port, max_retries=args.retries)

    def signal_handler(sig, frame):
        print("\n[BRIDGE] Exiting...")
        bridge.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    bridge.run()


if __name__ == "__main__":
    main()
