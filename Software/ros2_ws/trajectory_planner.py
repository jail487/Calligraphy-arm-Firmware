import sys
import os
import math
import matplotlib.pyplot as plt

# 確保能引用到同目錄下的 mapper 與 kinematics
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from trajectory_mapper import TrajectoryMapper
from robot_constants import SOFTWARE_SAFE_VEL_RAD_S, SOFTWARE_SAFE_ACCEL_RAD_S2

class TrajectoryPlanner:
    def __init__(self, dt=0.02):
        self.dt = dt  # 控制週期 (預設 50Hz = 0.02s)

    def generate_trapezoidal_profile(self, total_dist, vmax, amax):
        """生成一維的梯形速度/位置/加速度曲線"""
        if total_dist <= 0:
            return []

        # 判斷是否達不到 vmax 就得減速 (三角形速度曲線)
        if (vmax**2 / amax) > total_dist:
            v_peak = math.sqrt(total_dist * amax)
            t_acc = v_peak / amax
            t_const = 0.0
            d_acc = 0.5 * amax * t_acc**2
            d_const = 0.0
        else:
            t_acc = vmax / amax
            d_acc = 0.5 * amax * t_acc**2
            d_const = total_dist - 2 * d_acc
            t_const = d_const / vmax

        t_total = 2 * t_acc + t_const
        N = int(math.ceil(t_total / self.dt))
        
        profile = []
        for i in range(N + 1):
            t = i * self.dt
            if t <= t_acc:
                v = amax * t
                s = 0.5 * amax * t**2
                a = amax
            elif t <= t_acc + t_const:
                v = vmax
                s = d_acc + vmax * (t - t_acc)
                a = 0.0
            else:
                t_dec = t - t_acc - t_const
                v = vmax - amax * t_dec
                s = d_acc + d_const + vmax * t_dec - 0.5 * amax * t_dec**2
                a = -amax
                
                # 防止浮點數誤差導致過衝
                if v < 0: v = 0.0
                if s > total_dist: s = total_dist
                
            profile.append({'t': t, 's': s, 'v': v, 'a': a})
        return profile

    def _interpolate_spatial(self, pts, target_s):
        """給定累積距離 target_s，在折線 pts 中內插出 X, Y, Z"""
        if target_s <= 0: return pts[0]
        
        # 建立累積距離表
        cum_s = [0.0]
        for i in range(1, len(pts)):
            dx = pts[i]['x_cm'] - pts[i-1]['x_cm']
            dy = pts[i]['y_cm'] - pts[i-1]['y_cm']
            dz = pts[i]['z_cm'] - pts[i-1]['z_cm']
            dist = math.sqrt(dx**2 + dy**2 + dz**2)
            cum_s.append(cum_s[-1] + dist)
            
        if target_s >= cum_s[-1]: return pts[-1]
        
        # 尋找所在的線段
        for i in range(1, len(cum_s)):
            if cum_s[i-1] <= target_s <= cum_s[i]:
                ratio = (target_s - cum_s[i-1]) / (cum_s[i] - cum_s[i-1])
                x = pts[i-1]['x_cm'] + ratio * (pts[i]['x_cm'] - pts[i-1]['x_cm'])
                y = pts[i-1]['y_cm'] + ratio * (pts[i]['y_cm'] - pts[i-1]['y_cm'])
                z = pts[i-1]['z_cm'] + ratio * (pts[i]['z_cm'] - pts[i-1]['z_cm'])
                return {'x_cm': x, 'y_cm': y, 'z_cm': z, 'type': pts[i]['type']}
        return pts[-1]

    def plan_segment(self, segment_pts, vmax, amax, kin_solver):
        """對單一連續筆劃進行 50Hz PVA 規劃"""
        # 計算總距離
        total_dist = 0.0
        for i in range(1, len(segment_pts)):
            dx = segment_pts[i]['x_cm'] - segment_pts[i-1]['x_cm']
            dy = segment_pts[i]['y_cm'] - segment_pts[i-1]['y_cm']
            dz = segment_pts[i]['z_cm'] - segment_pts[i-1]['z_cm']
            total_dist += math.sqrt(dx**2 + dy**2 + dz**2)

        # 產生一維時間曲線
        profile = self.generate_trapezoidal_profile(total_dist, vmax, amax)
        
        # 重採樣並逆解算
        trajectory_50hz = []
        for p in profile:
            # 空間內插
            spat = self._interpolate_spatial(segment_pts, p['s'])
            
            # IK 解算 (需要公尺)
            ok, joints, _ = kin_solver.inverse_kinematics(spat['x_cm']/100.0, spat['y_cm']/100.0, spat['z_cm']/100.0)
            if not ok: continue
                
            trajectory_50hz.append({
                't': p['t'],
                'x': spat['x_cm'], 'y': spat['y_cm'], 'z': spat['z_cm'],
                'q1': joints[0], 'q2': joints[1], 'q3': joints[2]
            })
            
        # 數值微積分求 V 與 A
        for i in range(len(trajectory_50hz)):
            if i == 0:
                for ax in ('q1', 'q2', 'q3'):
                    trajectory_50hz[i][f'v_{ax}'] = 0.0
                    trajectory_50hz[i][f'a_{ax}'] = 0.0
            else:
                for ax in ('q1', 'q2', 'q3'):
                    v = (trajectory_50hz[i][ax] - trajectory_50hz[i-1][ax]) / self.dt
                    trajectory_50hz[i][f'v_{ax}'] = v
                    
                    if i == 1:
                        trajectory_50hz[i][f'a_{ax}'] = 0.0
                    else:
                        a = (trajectory_50hz[i][f'v_{ax}'] - trajectory_50hz[i-1][f'v_{ax}']) / self.dt
                        trajectory_50hz[i][f'a_{ax}'] = max(-SOFTWARE_SAFE_ACCEL_RAD_S2, min(SOFTWARE_SAFE_ACCEL_RAD_S2, a))

        return trajectory_50hz

    def plan_full_trajectory(self, physical_trajectory, kin_solver, v_write=2.0, v_travel=4.0, a_write=8.0, a_travel=12.0):
        """
        將路徑切分為連續段落：段落銜接處空間位置嚴格連續，
        確保在原點垂直抬升至空中高度後才進行水平移動。
        """
        if not physical_trajectory:
            return []

        # 1. 空間連續分段：每個段落頭尾與前後段相接
        segments = []
        curr_seg = [physical_trajectory[0]]
        for pt in physical_trajectory[1:]:
            if pt['type'] == curr_seg[-1]['type']:
                curr_seg.append(pt)
            else:
                # 銜接點：當前段以該轉折點作為終點
                curr_seg.append(pt)
                segments.append(curr_seg)
                # 下一段以該轉折點作為起點
                curr_seg = [pt]
        if curr_seg:
            segments.append(curr_seg)

        full_pva = []
        total_t = 0.0

        for i, seg in enumerate(segments):
            if len(seg) < 2:
                continue

            seg_type = seg[0]['type']
            vmax = v_write if seg_type == 'write' else v_travel
            amax = a_write if seg_type == 'write' else a_travel

            pva = self.plan_segment(seg, vmax=vmax, amax=amax, kin_solver=kin_solver)
            if not pva:
                continue

            # 每一段段內各自平滑，絕不跨段平滑 (保證起落筆交界處速度與水平位移嚴格為 0)
            pva_smoothed = self._smooth_pva_trajectory(pva, window_len=5)
            for p in pva_smoothed:
                p_copy = dict(p)
                p_copy['t'] += total_t
                p_copy['seg_type'] = seg_type
                for ax in ('q1', 'q2', 'q3'):
                    p_copy[f'v_{ax}'] = max(-SOFTWARE_SAFE_VEL_RAD_S, min(SOFTWARE_SAFE_VEL_RAD_S, p_copy[f'v_{ax}']))
                    p_copy[f'a_{ax}'] = max(-SOFTWARE_SAFE_ACCEL_RAD_S2, min(SOFTWARE_SAFE_ACCEL_RAD_S2, p_copy[f'a_{ax}']))
                full_pva.append(p_copy)

            total_t = full_pva[-1]['t']

        return full_pva

    def _smooth_pva_trajectory(self, full_pva, window_len=15):
        """對全段 PVA 進行 Hanning 濾波平滑，消除轉折點與筆劃交界的離散階躍 (消除 Jerk)"""
        if len(full_pva) < window_len:
            return full_pva
            
        import numpy as np
        w = np.hanning(window_len)
        w /= np.sum(w)
        pad_len = window_len // 2
        
        for ax in ('q1', 'q2', 'q3'):
            v_raw = np.array([p[f'v_{ax}'] for p in full_pva])
            v_smooth = np.convolve(np.pad(v_raw, (pad_len, pad_len), mode='edge'), w, mode='valid')
            
            # 重新從平滑後的速度算出加速度，保證 C1 連續無突刺
            a_smooth = np.gradient(v_smooth, self.dt)
            
            for idx, p in enumerate(full_pva):
                p[f'v_{ax}'] = float(max(-SOFTWARE_SAFE_VEL_RAD_S, min(SOFTWARE_SAFE_VEL_RAD_S, v_smooth[idx])))
                p[f'a_{ax}'] = float(max(-SOFTWARE_SAFE_ACCEL_RAD_S2, min(SOFTWARE_SAFE_ACCEL_RAD_S2, a_smooth[idx])))
                
        return full_pva

def plot_validation(segment_data, title="Trajectory Validation"):
    """彈出圖表，顯示 P, V, A 曲線 (這是未來比對真實資料的基礎)"""
    t = [p['t'] for p in segment_data]
    q1 = [math.degrees(p['q1']) for p in segment_data]
    v1 = [math.degrees(p['v_q1']) for p in segment_data]
    a1 = [math.degrees(p['a_q1']) for p in segment_data]
    
    q2 = [math.degrees(p['q2']) for p in segment_data]
    v2 = [math.degrees(p['v_q2']) for p in segment_data]
    a2 = [math.degrees(p['a_q2']) for p in segment_data]

    fig, axs = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    fig.suptitle(title, fontsize=16)

    # Position
    axs[0].plot(t, q1, label='J1 Angle (deg)', color='b', linewidth=2)
    axs[0].plot(t, q2, label='J2 Angle (deg)', color='r', linewidth=2)
    axs[0].set_ylabel('Position (deg)')
    axs[0].grid(True)
    axs[0].legend()

    # Velocity
    axs[1].plot(t, v1, label='J1 Velocity Target (deg/s)', color='b', linestyle='--')
    axs[1].plot(t, v2, label='J2 Velocity Target (deg/s)', color='r', linestyle='--')
    axs[1].set_ylabel('Velocity (deg/s)')
    axs[1].grid(True)
    axs[1].legend()

    # Acceleration
    axs[2].plot(t, a1, label='J1 Accel Target (deg/s^2)', color='b', linestyle=':')
    axs[2].plot(t, a2, label='J2 Accel Target (deg/s^2)', color='r', linestyle=':')
    axs[2].set_ylabel('Acceleration (deg/s^2)')
    axs[2].set_xlabel('Time (sec)')
    axs[2].grid(True)
    axs[2].legend()

    # 儲存圖表到同目錄
    save_path = os.path.join(os.path.dirname(__file__), 'pva_validation_plot.png')
    plt.savefig(save_path)
    print(f"\n[Plot] 驗證圖表已儲存至: {save_path}")
    
    # 關閉視窗，避免阻塞指令列 (若有桌面環境可改為 plt.show())
    plt.close(fig)

if __name__ == '__main__':
    test_json = r"C:\Users\jacks\Documents\project\GraduationProject\BBSMG_ws\samples\waypoint_path\robot_path.json"
    if not os.path.exists(test_json):
        print("找不到 JSON 檔案")
        sys.exit(1)
        
    print("[1] 載入並映射座標...")
    mapper = TrajectoryMapper(z_write_cm=3.5, z_travel_cm=8.0, margin_cm=1.5)
    raw_points = mapper.map_trajectory(test_json)
    
    # 抓取第一段 Write 筆劃來測試
    write_segment = []
    for pt in raw_points:
        if pt['type'] == 'write':
            write_segment.append(pt)
        elif len(write_segment) > 0:
            break # 抓到第一筆劃結束就跳出
            
    if not write_segment:
        print("找不到 Write 筆劃！")
        sys.exit(1)
        
    print(f"[2] 萃取第一筆劃，共 {len(write_segment)} 個點，開始進行 PVA 規劃...")
    planner = TrajectoryPlanner(dt=0.02)
    
    # 設定寫字速度 5 cm/s，加速度 20 cm/s^2
    pva_50hz = planner.plan_segment(write_segment, vmax=5.0, amax=20.0, kin_solver=mapper.kin)
    
    print(f"[3] 規劃完成！成功生成 {len(pva_50hz)} 個時間序列點 (50Hz)。")
    print(f"    預計書寫時間: {pva_50hz[-1]['t']:.2f} 秒")
    
    print("[4] 產生並儲存 PVA 驗證圖表...")
    plot_validation(pva_50hz, title="First Stroke PVA Target Profile")

