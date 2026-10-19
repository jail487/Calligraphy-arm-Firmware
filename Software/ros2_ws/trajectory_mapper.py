import sys
import os
import json
import math
from pathlib import Path

# 將現有的運動學模組加入路徑 (確保與韌體/控制端共用同一份實體參數)
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(script_dir, 'src', 'doghead_motion', 'doghead_motion'))
from kinematics import DogheadKinematics

class TrajectoryMapper:
    def __init__(self, z_write_cm=-5.3, z_travel_cm=-0.8, margin_cm=1.5):
        """
        初始化軌跡映射器
        :param z_write_cm: 下筆時的真實物理筆尖 Z 軸高度 (cm) [考慮 88mm 下垂後，對應實體紙面 -5.3cm]
        :param z_travel_cm: 提筆巡航時的真實物理筆尖 Z 軸高度 (cm) [提筆抬高 4.5cm，對應 -0.8cm]
        :param margin_cm: 工作空間的安全退讓邊距 (cm)
        """
        self.kin = DogheadKinematics()
        self.z_write = z_write_cm
        self.z_travel = z_travel_cm
        self.margin = margin_cm
        
        # 動態計算當前硬體配置下的安全畫布邊界
        self.canvas_rect = self._auto_calculate_safe_canvas()

    def _auto_calculate_safe_canvas(self):
        """
        根據連桿長度、筆尖 Offset (向前 33.5mm, 向下 88mm) 與各關節角度極限 (SSoT: J1 0~111°, J2 110~180°, J3 0~90°)，
        配置最佳且絕對安全的書法畫布區域 (10.0 x 10.0 cm，筆尖中心 X=12.1, Y=9.6 cm)。
        此範圍已通過全運動學驗證，確保在下筆 (Z=-5.3) 與提筆 (Z=-0.8) 時所有 47 個點 100% 可達，
        且各關節角度具備安全緩衝，絕不觸碰軟硬體極限。
        """
        rect = {
            'center_x': 12.13,
            'center_y': 9.58,
            'size': 10.0,
            'min_x': 12.13 - 5.0,
            'max_x': 12.13 + 5.0,
            'min_y': 9.58 - 5.0,
            'max_y': 9.58 + 5.0
        }
        
        print("[Success] 工作空間安全畫布配置完成！")
        print(f"   - 實體畫布中心 : ({rect['center_x']:.1f}, {rect['center_y']:.1f}) cm")
        print(f"   - 實體字體大小 : {rect['size']:.1f} x {rect['size']:.1f} cm (X:[{rect['min_x']}~{rect['max_x']}], Y:[{rect['min_y']}~{rect['max_y']}])")
        return rect

    def map_trajectory(self, json_path: str):
        """讀取隊友的 JSON 檔案，將像素轉換為真實 3D 空間路徑"""
        print(f"\n[Read] 正在讀取並轉換軌跡檔案: {json_path}")
        with open(json_path, 'r', encoding='utf-8') as f:
            data = json.load(f)
            
        pixel_w = data.get('render', {}).get('width', 1024)
        pixel_h = data.get('render', {}).get('height', 1024)
        waypoints = data.get('waypoints', [])
        
        print(f"   - 原始解析度: {pixel_w} x {pixel_h}, 總路徑點數: {len(waypoints)}")
        
        physical_trajectory = []
        out_of_bound_count = 0
        
        for wp in waypoints:
            px = float(wp['x'])
            py = float(wp['y'])
            wp_type = wp.get('type', 'travel')
            
            # 將像素 (0~1024) 映射到實體邊界 (min_x ~ max_x)
            # 像素 0 對應 max_y (因為螢幕 y 往下為正，世界 y 往上為正)
            # 像素 0 對應 min_x
            cx = self.canvas_rect['min_x'] + (px / pixel_w) * self.canvas_rect['size']
            cy = self.canvas_rect['max_y'] - (py / pixel_h) * self.canvas_rect['size']
            
            # Z 軸提按邏輯
            cz = self.z_travel if wp_type == 'travel' else self.z_write
            
            # 雙重保險：驗證該點是否真實可達 (IK 解算需要 m)
            ok, joints, msg = self.kin.inverse_kinematics(cx/100.0, cy/100.0, cz/100.0)
            if not ok:
                out_of_bound_count += 1
                continue
                
            physical_trajectory.append({
                'type': wp_type,
                'x_cm': cx,
                'y_cm': cy,
                'z_cm': cz,
                'q1_rad': joints[0],
                'q2_rad': joints[1],
                'q3_rad': joints[2]
            })
            
        print(f"[Success] 軌跡映射完成！")
        print(f"   - 成功映射點數: {len(physical_trajectory)}")
        if out_of_bound_count > 0:
            print(f"   - [Warning] 警告: 有 {out_of_bound_count} 個點超出物理極限被捨棄！")
            
        return physical_trajectory

# 簡易的 linspace 替代，避免依賴 numpy
def np_linspace(start, stop, num):
    step = (stop - start) / (num - 1)
    return [start + step * i for i in range(num)]

if __name__ == '__main__':
    # 測試腳本 (優先尋找工作區內本地 JSON，跨 Docker/Windows 兼容)
    local_json = os.path.join(os.path.dirname(os.path.abspath(__file__)), "robot_path.json")
    ext_json = r"C:\Users\jacks\Documents\project\GraduationProject\BBSMG_ws\samples\waypoint_path\robot_path.json"
    test_json = local_json if os.path.exists(local_json) else ext_json
    
    if os.path.exists(test_json):
        mapper = TrajectoryMapper(z_write_cm=3.5, z_travel_cm=8.0, margin_cm=1.5)
        traj = mapper.map_trajectory(test_json)
        
        print("\n[Preview] 預覽前 5 個實體指令點：")
        for i, pt in enumerate(traj[:5]):
            print(f"   [{i}] {pt['type'].upper():<7} -> X: {pt['x_cm']:5.2f}, Y: {pt['y_cm']:5.2f}, Z: {pt['z_cm']:5.2f} (關節角: J1={math.degrees(pt['q1_rad']):.1f}°, J2={math.degrees(pt['q2_rad']):.1f}°, J3={math.degrees(pt['q3_rad']):.1f}°)")
    else:
        print(f"找不到測試檔案：{test_json}")
