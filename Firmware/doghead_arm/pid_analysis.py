"""
PID 調參數據分析工具
用於分析從 STM32 輸出的測試數據
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
import pandas as pd

class PIDDataAnalyzer:
    """PID 測試數據分析器"""
    
    def __init__(self, csv_file):
        """載入 CSV 數據"""
        self.df = pd.read_csv(csv_file)
        self.time = self.df['Time_ms'].values / 1000.0  # 轉換為秒
        self.target = self.df['Target_deg'].values
        self.actual = self.df['Actual_deg'].values
        self.error = self.df['Error_deg'].values
        self.control = self.df['Control_RPM'].values
        self.velocity = self.df['Velocity_RPM'].values
        
    def plot_overview(self):
        """繪製總覽圖"""
        fig, axes = plt.subplots(4, 1, figsize=(12, 10))
        
        # 1. 位置跟隨
        axes[0].plot(self.time, self.target, 'r--', label='目標位置', linewidth=2)
        axes[0].plot(self.time, self.actual, 'b-', label='實際位置', linewidth=1.5)
        axes[0].set_ylabel('位置 (deg)')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title('位置跟隨曲線', fontsize=14, fontweight='bold')
        
        # 2. 跟隨誤差
        axes[1].plot(self.time, self.error, 'r-', linewidth=1.5)
        axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.5)
        axes[1].fill_between(self.time, -2, 2, alpha=0.2, color='green', label='±2° 穩定區')
        axes[1].set_ylabel('誤差 (deg)')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        axes[1].set_title('跟隨誤差', fontsize=14, fontweight='bold')
        
        # 3. 控制輸出
        axes[2].plot(self.time, self.control, 'g-', linewidth=1.5)
        axes[2].set_ylabel('控制輸出 (RPM)')
        axes[2].grid(True, alpha=0.3)
        axes[2].set_title('控制信號', fontsize=14, fontweight='bold')
        
        # 4. 實際速度
        axes[3].plot(self.time, self.velocity, 'm-', linewidth=1.5)
        axes[3].set_ylabel('速度 (RPM)')
        axes[3].set_xlabel('時間 (s)')
        axes[3].grid(True, alpha=0.3)
        axes[3].set_title('實際速度', fontsize=14, fontweight='bold')
        
        plt.tight_layout()
        plt.savefig('pid_analysis_overview.png', dpi=150)
        print("✓ 總覽圖已儲存: pid_analysis_overview.png")
        plt.show()
        
    def calculate_metrics(self):
        """計算性能指標"""
        dt = np.mean(np.diff(self.time))
        
        metrics = {
            'IAE': np.trapz(np.abs(self.error), self.time),
            'ISE': np.trapz(self.error**2, self.time),
            'ITAE': np.trapz(self.time * np.abs(self.error), self.time),
            'max_error': np.max(np.abs(self.error)),
            'steady_state_error': np.mean(np.abs(self.error[-10:])),
        }
        
        # 階躍響應分析（如果是階躍）
        if np.std(self.target) > 1.0:  # 有明顯變化
            step_size = np.max(self.target) - np.min(self.target)
            peak_value = np.max(self.actual)
            final_value = np.mean(self.actual[-20:])
            
            overshoot = ((peak_value - final_value) / step_size) * 100
            metrics['overshoot_percent'] = overshoot
            
            # 上升時間
            threshold_10 = np.min(self.actual) + step_size * 0.1
            threshold_90 = np.min(self.actual) + step_size * 0.9
            
            idx_10 = np.where(self.actual > threshold_10)[0]
            idx_90 = np.where(self.actual > threshold_90)[0]
            
            if len(idx_10) > 0 and len(idx_90) > 0:
                rise_time = self.time[idx_90[0]] - self.time[idx_10[0]]
                metrics['rise_time_s'] = rise_time
            
            # 穩定時間
            settling_band = step_size * 0.02  # 2%
            stable_mask = np.abs(self.actual - final_value) < settling_band
            
            # 從後往前找第一個不穩定點
            for i in range(len(stable_mask)-1, -1, -1):
                if not stable_mask[i]:
                    metrics['settling_time_s'] = self.time[i]
                    break
        
        return metrics
    
    def print_metrics(self):
        """打印性能指標"""
        metrics = self.calculate_metrics()
        
        print("\n" + "="*50)
        print("          性能指標分析報告")
        print("="*50)
        
        print("\n【積分型指標】")
        print(f"  IAE  (絕對誤差積分)    : {metrics['IAE']:.2f}")
        print(f"  ISE  (誤差平方積分)    : {metrics['ISE']:.2f}")
        print(f"  ITAE (時間加權誤差)    : {metrics['ITAE']:.2f}")
        print(f"  最大誤差               : {metrics['max_error']:.2f} deg")
        print(f"  穩態誤差               : {metrics['steady_state_error']:.2f} deg")
        
        if 'overshoot_percent' in metrics:
            print("\n【階躍響應特性】")
            print(f"  超調量                 : {metrics.get('overshoot_percent', 0):.1f} %")
            print(f"  上升時間               : {metrics.get('rise_time_s', 0)*1000:.0f} ms")
            print(f"  穩定時間               : {metrics.get('settling_time_s', 0)*1000:.0f} ms")
        
        # 綜合評分
        score = 100 - metrics['IAE'] * 0.5 - metrics['steady_state_error'] * 2
        score = max(0, min(100, score))
        
        print("\n【綜合評分】")
        print(f"  分數: {score:.1f} / 100", end=" ")
        if score > 80:
            print("(優秀 ⭐⭐⭐)")
        elif score > 60:
            print("(良好 ⭐⭐)")
        elif score > 40:
            print("(可接受 ⭐)")
        else:
            print("(需改進)")
        
        print("="*50 + "\n")
        
        return metrics
    
    def frequency_analysis(self):
        """頻域分析（用於判斷震盪頻率）"""
        # FFT 分析誤差信號
        fft_error = np.fft.fft(self.error)
        freq = np.fft.fftfreq(len(self.error), np.mean(np.diff(self.time)))
        
        # 只取正頻率
        pos_mask = freq > 0
        freq_pos = freq[pos_mask]
        fft_pos = np.abs(fft_error[pos_mask])
        
        # 找主要頻率分量
        dominant_idx = np.argsort(fft_pos)[-5:]  # 前5個
        
        plt.figure(figsize=(10, 6))
        plt.semilogy(freq_pos, fft_pos)
        plt.xlabel('頻率 (Hz)')
        plt.ylabel('幅度')
        plt.title('誤差信號頻譜分析')
        plt.grid(True, alpha=0.3)
        
        for idx in dominant_idx:
            if freq_pos[idx] < 10:  # 只標註低頻
                plt.axvline(freq_pos[idx], color='r', linestyle='--', alpha=0.5)
                plt.text(freq_pos[idx], fft_pos[idx], f'{freq_pos[idx]:.2f} Hz')
        
        plt.savefig('pid_frequency_analysis.png', dpi=150)
        print("✓ 頻譜圖已儲存: pid_frequency_analysis.png")
        plt.show()
        
        print("\n主要震盪頻率:")
        for idx in dominant_idx[::-1]:
            if freq_pos[idx] < 10:
                print(f"  {freq_pos[idx]:.2f} Hz (幅度: {fft_pos[idx]:.1f})")
    
    def compare_parameters(self, other_analyzer, label1="參數組1", label2="參數組2"):
        """比較兩組參數的性能"""
        metrics1 = self.calculate_metrics()
        metrics2 = other_analyzer.calculate_metrics()
        
        # 創建比較表
        comparison = pd.DataFrame({
            label1: [
                metrics1['IAE'],
                metrics1['ISE'],
                metrics1['max_error'],
                metrics1['steady_state_error'],
                metrics1.get('overshoot_percent', 0),
                metrics1.get('rise_time_s', 0) * 1000
            ],
            label2: [
                metrics2['IAE'],
                metrics2['ISE'],
                metrics2['max_error'],
                metrics2['steady_state_error'],
                metrics2.get('overshoot_percent', 0),
                metrics2.get('rise_time_s', 0) * 1000
            ]
        }, index=['IAE', 'ISE', '最大誤差', '穩態誤差', '超調量(%)', '上升時間(ms)'])
        
        print("\n" + "="*60)
        print("                 參數比較")
        print("="*60)
        print(comparison)
        print("="*60)
        
        # 視覺化比較
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        
        axes[0].plot(self.time, self.actual, 'b-', label=label1, linewidth=2)
        axes[0].plot(other_analyzer.time, other_analyzer.actual, 'r--', label=label2, linewidth=2)
        axes[0].plot(self.time, self.target, 'k:', label='目標', alpha=0.5)
        axes[0].set_ylabel('位置 (deg)')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        axes[0].set_title('位置跟隨比較')
        
        axes[1].plot(self.time, self.error, 'b-', label=label1, linewidth=2)
        axes[1].plot(other_analyzer.time, other_analyzer.error, 'r--', label=label2, linewidth=2)
        axes[1].axhline(y=0, color='k', linestyle='--', alpha=0.5)
        axes[1].set_ylabel('誤差 (deg)')
        axes[1].set_xlabel('時間 (s)')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        axes[1].set_title('誤差比較')
        
        plt.tight_layout()
        plt.savefig('pid_parameter_comparison.png', dpi=150)
        print("\n✓ 比較圖已儲存: pid_parameter_comparison.png")
        plt.show()


def batch_analyze(csv_files, labels):
    """批量分析多組數據"""
    results = []
    
    for csv_file, label in zip(csv_files, labels):
        print(f"\n處理: {label}")
        analyzer = PIDDataAnalyzer(csv_file)
        metrics = analyzer.calculate_metrics()
        metrics['label'] = label
        results.append(metrics)
    
    # 創建對比表
    df_results = pd.DataFrame(results)
    df_results = df_results.set_index('label')
    
    print("\n" + "="*80)
    print("                      批量分析結果對比")
    print("="*80)
    print(df_results[['IAE', 'ISE', 'max_error', 'steady_state_error']])
    print("="*80)
    
    # 繪製對比圖
    fig, ax = plt.subplots(figsize=(10, 6))
    df_results[['IAE', 'ISE']].plot(kind='bar', ax=ax)
    ax.set_ylabel('誤差指標')
    ax.set_title('不同參數組性能對比')
    ax.legend(['IAE', 'ISE'])
    plt.xticks(rotation=45)
    plt.tight_layout()
    plt.savefig('pid_batch_comparison.png', dpi=150)
    print("\n✓ 批量對比圖已儲存: pid_batch_comparison.png")
    plt.show()


# ==================== 使用範例 ====================

if __name__ == "__main__":
    import sys
    
    # 設定中文字體（Windows）
    plt.rcParams['font.sans-serif'] = ['Microsoft JhengHei', 'Arial Unicode MS']
    plt.rcParams['axes.unicode_minus'] = False
    
    print("="*60)
    print("       PID 調參數據分析工具")
    print("="*60)
    
    # 單個文件分析
    if len(sys.argv) > 1:
        csv_file = sys.argv[1]
        print(f"\n載入數據: {csv_file}")
        
        analyzer = PIDDataAnalyzer(csv_file)
        
        # 繪製總覽圖
        analyzer.plot_overview()
        
        # 打印性能指標
        analyzer.print_metrics()
        
        # 頻域分析
        analyzer.frequency_analysis()
        
    else:
        print("\n使用方法:")
        print("  python pid_analysis.py <csv檔案路徑>")
        print("\n範例:")
        print("  python pid_analysis.py test_data.csv")
        print("\n或在 Python 程式中使用:")
        print("  analyzer = PIDDataAnalyzer('test_data.csv')")
        print("  analyzer.plot_overview()")
        print("  analyzer.print_metrics()")
        
        # 批量分析範例
        print("\n批量分析範例:")
        print("  files = ['kp5.csv', 'kp8.csv', 'kp10.csv']")
        print("  labels = ['Kp=5', 'Kp=8', 'Kp=10']")
        print("  batch_analyze(files, labels)")
