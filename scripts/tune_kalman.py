#!/usr/bin/env python3
"""
ARV_V1 Kalman滤波器调参与评估工具
实时显示滤波效果，评估增益K是否在合理范围
用法: python3 tune_kalman.py
"""
import subprocess
import sys
import time
import signal

NODE = "/torque_controller_action_server"

# Kalman增益判断标准 (来自 TODO_KDL.md)
K_LOW = 0.05   # K < 0.05: 过度平滑，增大Q_vel
K_GOOD_MIN = 0.1
K_GOOD_MAX = 0.3
K_HIGH = 0.5   # K > 0.5: 过度信任测量，减小Q_vel

class KalmanTuner:
    def __init__(self):
        self.running = True
        signal.signal(signal.SIGINT, lambda s,f: self._exit())

    def _exit(self):
        self.running = False
        print("\n退出")
        sys.exit(0)

    def get_param(self, name: str):
        try:
            r = subprocess.run(["ros2", "param", "get", NODE, name],
                             capture_output=True, text=True, timeout=3)
            if r.returncode == 0 and "value is:" in r.stdout:
                return float(r.stdout.split(":")[-1].strip())
        except:
            pass
        return None

    def set_param(self, name: str, value: float) -> bool:
        try:
            r = subprocess.run(["ros2", "param", "set", NODE, name, str(value)],
                             capture_output=True, text=True, timeout=3)
            return r.returncode == 0
        except:
            return False

    def get_kalman_params(self) -> dict:
        return {
            "enabled": self.get_param("kalman.enabled"),
            "Q_pos": self.get_param("kalman.Q_pos"),
            "Q_vel": self.get_param("kalman.Q_vel"),
            "R_pos": self.get_param("kalman.R_pos"),
            "R_vel": self.get_param("kalman.R_vel"),
        }

    def print_status(self):
        p = self.get_kalman_params()
        print("\n" + "="*60)
        print("  Kalman 滤波器状态")
        print("="*60)
        enabled = "启用" if p["enabled"] else "禁用"
        print(f"  状态: {enabled}")
        print(f"\n  过程噪声 (Q):")
        print(f"    Q_pos = {p['Q_pos']:.1e}  (位置不确定性)")
        print(f"    Q_vel = {p['Q_vel']:.1e}  (速度不确定性) <- 主要调参项")
        print(f"\n  测量噪声 (R):")
        print(f"    R_pos = {p['R_pos']:.1e}  (编码器噪声)")
        print(f"    R_vel = {p['R_vel']:.1e}  (速度测量噪声)")
        print("-"*60)
        print("  增益K判断标准:")
        print(f"    K < {K_LOW}: 过度平滑 -> 增大 Q_vel")
        print(f"    K ∈ [{K_GOOD_MIN}, {K_GOOD_MAX}]: 平衡 ✓")
        print(f"    K > {K_HIGH}: 过度信任测量 -> 减小 Q_vel")
        print("="*60)

    def menu(self):
        print("\n" + "-"*40)
        print("  [1] 查看当前参数")
        print("  [2] 调节 Q_vel (速度过程噪声)")
        print("  [3] 调节 R_vel (速度测量噪声)")
        print("  [4] 快速预设")
        print("  [5] 开/关滤波器")
        print("  [6] 查看增益K (需看控制器日志)")
        print("  [0] 退出")
        print("-"*40)

    def run(self):
        while self.running:
            self.menu()
            c = input("选择: ").strip()

            if c == "1":
                self.print_status()
            elif c == "2":
                self._tune_q_vel()
            elif c == "3":
                self._tune_r_vel()
            elif c == "4":
                self._presets()
            elif c == "5":
                self._toggle()
            elif c == "6":
                self._show_gain_hint()
            elif c == "0":
                self._exit()

    def _tune_q_vel(self):
        """调节Q_vel"""
        current = self.get_param("kalman.Q_vel")
        print(f"\n当前 Q_vel = {current:.1e}")
        print("建议范围: 1e-7 ~ 1e-3")
        print("  增大 -> K增大 -> 响应更快，噪声更多")
        print("  减小 -> K减小 -> 更平滑，响应变慢")
        try:
            v = input("新值 (如 1e-5): ").strip()
            if v:
                val = float(v)
                if self.set_param("kalman.Q_vel", val):
                    print(f"[OK] Q_vel = {val:.1e}")
                else:
                    print("[失败]")
        except ValueError:
            print("[输入错误]")

    def _tune_r_vel(self):
        """调节R_vel"""
        current = self.get_param("kalman.R_vel")
        print(f"\n当前 R_vel = {current:.1e}")
        print("建议范围: 1e-3 ~ 0.1")
        try:
            v = input("新值: ").strip()
            if v:
                val = float(v)
                if self.set_param("kalman.R_vel", val):
                    print(f"[OK] R_vel = {val:.1e}")
                else:
                    print("[失败]")
        except ValueError:
            print("[输入错误]")

    def _presets(self):
        """预设模板"""
        presets = {
            "1": ("响应优先", 1e-4, 1e-2),
            "2": ("平衡(默认)", 1e-5, 2.5e-2),
            "3": ("平滑优先", 1e-6, 5e-2),
        }
        print("\n预设:")
        for k, (name, q, r) in presets.items():
            print(f"  [{k}] {name}: Q_vel={q:.0e}, R_vel={r:.0e}")
        c = input("选择 [1-3]: ").strip()
        if c in presets:
            _, q, r = presets[c]
            self.set_param("kalman.Q_vel", q)
            self.set_param("kalman.R_vel", r)
            print(f"[OK] 已应用")

    def _toggle(self):
        """开关滤波器"""
        current = self.get_param("kalman.enabled")
        new_val = not current
        if self.set_param("kalman.enabled", new_val):
            print(f"[OK] 滤波器已{'启用' if new_val else '禁用'}")

    def _show_gain_hint(self):
        """显示如何查看增益"""
        print("\n" + "="*60)
        print("查看 Kalman 增益 K:")
        print("-"*60)
        print("方法1: 查看控制器终端输出")
        print("  每5秒自动打印: 'Kalman Gain Observation'")
        print("")
        print("方法2: 设置打印间隔")
        print("  ros2 param set /torque_controller_action_server \\")
        print("       kalman.print_interval 200  # 1秒打印一次")
        print("")
        print("判断标准:")
        print(f"  K[1,1] (速度增益) 应在 [{K_GOOD_MIN}, {K_GOOD_MAX}] 范围内")
        print("="*60)

if __name__ == "__main__":
    print("检查节点...")
    r = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True)
    if "torque_controller" not in r.stdout:
        print("[警告] 控制器节点未运行!")
    KalmanTuner().run()
