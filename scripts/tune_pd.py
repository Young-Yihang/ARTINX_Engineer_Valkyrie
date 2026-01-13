#!/usr/bin/env python3
"""
ARV_V1 PD/PID 调参工具 - 交互式六关节调参 + 性能评估
用法: python3 tune_pd.py
"""
import subprocess
import sys
import time
import threading
import signal
from dataclasses import dataclass
from typing import List, Optional
import json

NODE = "/torque_controller_action_server"

@dataclass
class JointMetrics:
    """单关节性能指标"""
    rmse: float = 0.0          # 位置RMSE (rad)
    max_error: float = 0.0     # 最大位置误差
    overshoot: float = 0.0     # 超调量 (%)
    settling_time: float = 0.0 # 调节时间 (s)
    samples: int = 0

class TunerCLI:
    def __init__(self):
        self.running = True
        self.recording = False
        self.data_buffer = []
        self.metrics = [JointMetrics() for _ in range(6)]
        signal.signal(signal.SIGINT, lambda s,f: self._exit())

    def _exit(self):
        self.running = False
        print("\n退出调参工具")
        sys.exit(0)

    def ros2_param_get(self, param: str) -> Optional[float]:
        """获取ROS2参数"""
        try:
            result = subprocess.run(
                ["ros2", "param", "get", NODE, param],
                capture_output=True, text=True, timeout=3
            )
            if result.returncode == 0:
                # 解析输出: "Double value is: 30.0"
                line = result.stdout.strip()
                if "value is:" in line:
                    return float(line.split(":")[-1].strip())
        except Exception as e:
            print(f"[错误] 获取参数失败: {e}")
        return None

    def ros2_param_set(self, param: str, value: float) -> bool:
        """设置ROS2参数"""
        try:
            result = subprocess.run(
                ["ros2", "param", "set", NODE, param, str(value)],
                capture_output=True, text=True, timeout=3
            )
            return result.returncode == 0
        except Exception as e:
            print(f"[错误] 设置参数失败: {e}")
            return False

    def get_all_gains(self) -> dict:
        """获取所有PD增益"""
        gains = {"Kp": [], "Kd": []}
        for i in range(1, 7):
            kp = self.ros2_param_get(f"Kp.joint_{i}")
            kd = self.ros2_param_get(f"Kd.joint_{i}")
            gains["Kp"].append(kp if kp else 0.0)
            gains["Kd"].append(kd if kd else 0.0)
        return gains

    def print_current_gains(self):
        """显示当前增益"""
        gains = self.get_all_gains()
        print("\n" + "="*60)
        print("当前 PD 增益:")
        print("-"*60)
        print(f"  关节:  J1      J2      J3      J4      J5      J6")
        print(f"  Kp: {gains['Kp'][0]:7.1f} {gains['Kp'][1]:7.1f} {gains['Kp'][2]:7.1f} "
              f"{gains['Kp'][3]:7.1f} {gains['Kp'][4]:7.1f} {gains['Kp'][5]:7.1f}")
        print(f"  Kd: {gains['Kd'][0]:7.2f} {gains['Kd'][1]:7.2f} {gains['Kd'][2]:7.2f} "
              f"{gains['Kd'][3]:7.2f} {gains['Kd'][4]:7.2f} {gains['Kd'][5]:7.2f}")
        print("="*60)

    def set_joint_gain(self, joint: int, kp: float = None, kd: float = None):
        """设置单关节增益"""
        if kp is not None:
            if self.ros2_param_set(f"Kp.joint_{joint}", kp):
                print(f"  [OK] Kp.joint_{joint} = {kp}")
            else:
                print(f"  [失败] Kp.joint_{joint}")
        if kd is not None:
            if self.ros2_param_set(f"Kd.joint_{joint}", kd):
                print(f"  [OK] Kd.joint_{joint} = {kd}")
            else:
                print(f"  [失败] Kd.joint_{joint}")

    def set_all_gains(self, kp_list: List[float], kd_list: List[float]):
        """批量设置所有增益"""
        print("\n批量设置增益...")
        for i in range(6):
            self.set_joint_gain(i+1, kp_list[i], kd_list[i])
        print("完成!")

    def menu(self):
        """主菜单"""
        print("\n" + "="*50)
        print("  ARV_V1 PD 调参工具")
        print("="*50)
        print("  [1] 查看当前增益")
        print("  [2] 调单关节 Kp/Kd")
        print("  [3] 批量调所有关节")
        print("  [4] 快速增益模板")
        print("  [5] 启动性能记录 (需另开终端)")
        print("  [0] 退出")
        print("-"*50)

    def run(self):
        while self.running:
            self.menu()
            choice = input("选择 [0-5]: ").strip()

            if choice == "1":
                self.print_current_gains()
            elif choice == "2":
                self._tune_single_joint()
            elif choice == "3":
                self._tune_all_joints()
            elif choice == "4":
                self._apply_template()
            elif choice == "5":
                self._start_recording_hint()
            elif choice == "0":
                self._exit()
            else:
                print("[无效选择]")

    def _tune_single_joint(self):
        """调单关节"""
        self.print_current_gains()
        try:
            j = int(input("关节编号 [1-6]: "))
            if j < 1 or j > 6:
                print("[无效关节]")
                return
            kp_str = input(f"新 Kp (当前值回车跳过): ").strip()
            kd_str = input(f"新 Kd (当前值回车跳过): ").strip()
            kp = float(kp_str) if kp_str else None
            kd = float(kd_str) if kd_str else None
            self.set_joint_gain(j, kp, kd)
        except ValueError:
            print("[输入错误]")

    def _tune_all_joints(self):
        """批量调参"""
        self.print_current_gains()
        print("\n输入6个值，空格分隔 (回车跳过该行):")
        try:
            kp_str = input("Kp [J1-J6]: ").strip()
            kd_str = input("Kd [J1-J6]: ").strip()
            if kp_str:
                kp_list = [float(x) for x in kp_str.split()]
                if len(kp_list) == 6:
                    for i, kp in enumerate(kp_list):
                        self.set_joint_gain(i+1, kp=kp)
            if kd_str:
                kd_list = [float(x) for x in kd_str.split()]
                if len(kd_list) == 6:
                    for i, kd in enumerate(kd_list):
                        self.set_joint_gain(i+1, kd=kd)
        except ValueError:
            print("[输入格式错误]")

    def _apply_template(self):
        """预设模板"""
        templates = {
            "1": ("保守(低增益)", [20,30,20,8,0.5,0.5], [0.5,0.5,0.5,0.2,0.5,0.5]),
            "2": ("标准(当前默认)", [30,50,30,10,1,1], [1,1,1,0.3,1,1]),
            "3": ("激进(高增益)", [50,80,50,15,2,2], [1.5,1.5,1.5,0.5,1.5,1.5]),
        }
        print("\n增益模板:")
        for k, (name, kp, kd) in templates.items():
            print(f"  [{k}] {name}")
            print(f"      Kp: {kp}")
            print(f"      Kd: {kd}")
        choice = input("选择模板 [1-3]: ").strip()
        if choice in templates:
            _, kp, kd = templates[choice]
            self.set_all_gains(kp, kd)

    def _start_recording_hint(self):
        """性能记录提示"""
        print("\n" + "="*60)
        print("性能记录方法:")
        print("-"*60)
        print("1. 新开终端运行:")
        print("   python3 scripts/record_metrics.py")
        print("")
        print("2. 在 RViz 中规划执行一段轨迹")
        print("")
        print("3. 脚本会自动计算并显示:")
        print("   - 各关节位置 RMSE")
        print("   - 超调量 / 调节时间")
        print("   - 力矩饱和次数")
        print("="*60)

if __name__ == "__main__":
    print("检查 ROS2 节点...")
    result = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True)
    if NODE.replace("/","") not in result.stdout:
        print(f"[警告] 节点 {NODE} 未运行!")
        print("请先启动: ./start_mujoco_system.sh")
    TunerCLI().run()
