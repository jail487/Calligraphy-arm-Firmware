#!/usr/bin/env python3
"""
Doghead Arm - MCU 即時狀態、安全閘門與通訊診斷監控儀表板 (MCU Live Monitor)
----------------------------------------------------------------------
提供全方位的底層韌體除錯與診斷視圖：
- 狀態機 (Safety State): SAFE / READY / RUNNING / FAULT
- 控制模式 (Control Mode): DISABLED / MOTOR_TEST / JOINT_PVA / HOMING
- 輸出遮罩: 邏輯 enable_mask vs 實體 physical_enable_mask (檢查硬體閘門是否放行)
- 指令生命週期: command_age_ms, last_accepted_seq, 拒絕與超時統計
- 診斷錯誤計數: session_mismatch, invalid_cmd, stale_seq, timeout
- 各軸實測角度、PID 輸出轉速 (commanded_motor_rpm)、編碼器計數
"""

import sys
import time
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from doghead_interfaces.msg import McuState, McuDiagnostics, McuSessionControl

SAFETY_STATE_MAP = {
    0: "0 (ROBOT_STATE_SAFE) [安全停機]",
    1: "1 (ROBOT_STATE_HOMING) [歸零校正中]",
    2: "2 (ROBOT_STATE_READY) [就緒]",
    3: "3 (ROBOT_STATE_RUNNING) [運轉中 ✅]",
    4: "4 (ROBOT_STATE_FAULT) [故障鎖定 ⚠️]",
    5: "5 (ROBOT_STATE_ESTOP) [緊急停止 🛑]"
}

FAULT_CODE_MAP = {
    0: "0 (NONE) [無故障]",
    1: "1 (HOMING_TIMEOUT) [歸零逾時]",
    2: "2 (HOMING_TRAVEL_LIMIT) [歸零行程超限]",
    3: "3 (POSITIVE_LIMIT) [正向限位觸發 ⚠️]",
    4: "4 (NEGATIVE_LIMIT) [反向限位觸發 ⚠️]",
    5: "5 (INVALID_COMMAND) [非法指令]",
    6: "6 (TRAJECTORY) [軌跡異常]",
    7: "7 (SOFTWARE_LIMIT) [軟體限位超限]",
    8: "8 (DEADLINE_MISS) [週期逾時]"
}

CONTROL_MODE_MAP = {
    0: "0 (DISABLED)",
    1: "1 (MOTOR_VELOCITY_TEST)",
    2: "2 (JOINT_PVA)",
    3: "3 (HOMING)",
    4: "4 (STOP)"
}

class McuLiveMonitor(Node):
    def __init__(self):
        super().__init__('mcu_live_monitor')
        
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.state_sub = self.create_subscription(
            McuState, '/doghead/mcu/state', self._on_mcu_state, qos
        )
        self.diag_sub = self.create_subscription(
            McuDiagnostics, '/doghead/mcu/diagnostics', self._on_mcu_diag, 10
        )
        
        self.latest_state = None
        self.latest_diag = None
        self.last_state_time = 0.0
        self.last_diag_time = 0.0
        
        # 5 Hz 刷新終端畫面
        self.timer = self.create_timer(0.2, self._render_dashboard)

    def _on_mcu_state(self, msg: McuState):
        self.latest_state = msg
        self.last_state_time = time.monotonic()

    def _on_mcu_diag(self, msg: McuDiagnostics):
        self.latest_diag = msg
        self.last_diag_time = time.monotonic()

    def _render_dashboard(self):
        # 清除終端機畫面
        print("\033[H\033[J", end="")
        
        print("=" * 80)
        print("        🐶 DOGHEAD ARM - MCU 底層韌體即時監控與除錯診斷儀表板")
        print("=" * 80)
        
        now = time.monotonic()
        state_alive = (now - self.last_state_time < 1.0) if self.last_state_time > 0 else False
        diag_alive = (now - self.last_diag_time < 2.0) if self.last_diag_time > 0 else False
        
        state_status = "🟢 正常接收 (50Hz)" if state_alive else "🔴 通訊中斷"
        diag_status = "🟢 正常接收 (10Hz)" if diag_alive else "🔴 通訊中斷"
        
        print(f"【通訊鏈路狀態】: /doghead/mcu/state: {state_status}  |  /doghead/mcu/diagnostics: {diag_status}")
        print("-" * 80)
        
        if not self.latest_state:
            print("⏳ 正在等待 STM32 McuState 封包...")
            return
            
        s = self.latest_state
        d = self.latest_diag
        
        # 1. 核心安全與狀態機
        safety_str = SAFETY_STATE_MAP.get(s.safety_state, f"未知 ({s.safety_state})")
        fault_str = FAULT_CODE_MAP.get(s.fault_code, f"未知 ({s.fault_code})")
        mode_str = CONTROL_MODE_MAP.get(s.control_mode, f"未知 ({s.control_mode})")
        print("【MCU 核心狀態機】:")
        print(f"  • 安全狀態 (safety_state) : {safety_str}")
        print(f"  • 故障代碼 (fault_code)   : {fault_str}")
        print(f"  • 控制模式 (control_mode) : {mode_str}")
        print(f"  • 開機辨識碼 (boot_id)    : {s.boot_id} (0x{s.boot_id:04X}) | Uptime: {s.uptime_ms/1000.0:6.1f} s")
        print("-" * 80)
        
        # 2. 硬體輸出安全閘門 (硬體放行關鍵指標)
        req_mask = f"0x{s.requested_enable_mask:02X} (b'{s.requested_enable_mask:03b}')"
        app_mask = f"0x{s.enable_mask:02X} (b'{s.enable_mask:03b}')"
        phy_mask = f"0x{s.physical_enable_mask:02X} (b'{s.physical_enable_mask:03b}')"
        
        gate_status = "🟢 馬達硬體輸出放行中 (Can Drive)" if s.physical_enable_mask != 0 else "🔴 硬體閘門阻斷 (Outputs Suppressed)"
        print("【致能遮罩與硬體閘門】:")
        print(f"  • 請求遮罩 (requested) : {req_mask}")
        print(f"  • 邏輯遮罩 (validated) : {app_mask}")
        print(f"  • 實體輸出 (physical)  : {phy_mask} -> {gate_status}")
        print("-" * 80)
        
        # 3. 指令生命週期與拒絕統計
        cmd_freshness = f"{s.command_age_ms} ms" if s.command_age_ms < 1000 else f"{s.command_age_ms/1000.0:.1f} s (已逾時 ⚠️)"
        print("【指令通訊與拒絕統計】:")
        print(f"  • 指令年齡 (command_age_ms) : {cmd_freshness}")
        print(f"  • 最新接收序號 (last_seq)   : {s.last_accepted_command_sequence}")
        print(f"  • 總拒絕指令數 (rejected)   : {s.rejected_command_total}")
        
        if d:
            print("  • 異常統計 (Diagnostics)  : "
                  f"Session不符={d.session_mismatch_total}, "
                  f"非法指令={d.invalid_command_total}, "
                  f"過期序號={d.stale_sequence_total}, "
                  f"超時停機={d.command_timeout_total}")
            print(f"  • 系統資源 (Free Heap)     : {d.free_heap_bytes} bytes (歷史最低: {d.minimum_ever_free_heap_bytes} bytes)")
        print("-" * 80)
        
        # 4. 三軸實測數值與輸出轉速
        q1_deg = math.degrees(s.measured_position_rad[0]) if len(s.measured_position_rad) > 0 else 0.0
        q2_deg = math.degrees(s.measured_position_rad[1]) if len(s.measured_position_rad) > 1 else 0.0
        q3_deg = math.degrees(s.measured_position_rad[2]) if len(s.measured_position_rad) > 2 else 0.0
        
        t1_deg = math.degrees(s.target_position_rad[0]) if len(s.target_position_rad) > 0 else 0.0
        t2_deg = math.degrees(s.target_position_rad[1]) if len(s.target_position_rad) > 1 else 0.0
        t3_deg = math.degrees(s.target_position_rad[2]) if len(s.target_position_rad) > 2 else 0.0
        
        print("【關節軸詳細狀態】:")
        print(f"  軸 1 (J1 大臂) : 實測={q1_deg:6.2f}° | 目標={t1_deg:6.2f}° | 指令RPM={s.commanded_motor_rpm[0]:6.1f} | 編碼器={s.encoder_count[0]}")
        print(f"  軸 2 (J2 小臂) : 實測={q2_deg:6.2f}° | 目標={t2_deg:6.2f}° | 指令RPM={s.commanded_motor_rpm[1]:6.1f} | 編碼器={s.encoder_count[1]}")
        print(f"  軸 3 (J3 基座) : 實測={q3_deg:6.2f}° | 目標={t3_deg:6.2f}° | 指令RPM={s.commanded_motor_rpm[2]:6.1f} | 編碼器={s.encoder_count[2]}")
        print("=" * 80)
        print("按 Ctrl+C 可退出監控儀表板")

def main():
    rclpy.init()
    node = McuLiveMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
