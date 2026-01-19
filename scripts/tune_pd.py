#!/usr/bin/env python3
"""
ARV_V1 Cascade P+PI 调参工具 - 交互式六关节调参 + 性能评估
用法: python3 tune_pd.py (兼容旧名称，实际调整cascade PID)
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
        self._node_check_cache = None  # 缓存节点在线状态
        signal.signal(signal.SIGINT, lambda s,f: self._exit())

    def _exit(self):
        self.running = False
        print("\n退出调参工具")
        sys.exit(0)
    
    def _check_node_online(self) -> bool:
        """检查节点是否在线（带缓存）"""
        if self._node_check_cache is not None:
            return self._node_check_cache
        try:
            result = subprocess.run(
                ["ros2", "node", "list"], 
                capture_output=True, text=True, timeout=1
            )
            self._node_check_cache = NODE.replace("/","") in result.stdout
            return self._node_check_cache
        except:
            return False

    def ros2_param_get(self, param: str) -> Optional[float]:
        """获取ROS2参数（快速版）"""
        if not self._check_node_online():
            print(f"[错误] 节点 {NODE} 未运行")
            return None
        try:
            result = subprocess.run(
                ["ros2", "param", "get", NODE, param],
                capture_output=True, text=True, timeout=3.0  # ros2命令本身需要2秒左右
            )
            if result.returncode == 0:
                # 解析输出: "Double value is: 30.0"
                line = result.stdout.strip()
                if "value is:" in line:
                    return float(line.split(":")[-1].strip())
        except subprocess.TimeoutExpired:
            print(f"[超时] 参数 {param} 获取超时")
            self._node_check_cache = None  # 清除缓存，下次重新检查
        except Exception as e:
            print(f"[错误] 获取参数 {param} 失败: {e}")
        return None

    def ros2_param_set(self, param: str, value: float) -> bool:
        """设置ROS2参数（快速版）"""
        if not self._check_node_online():
            print(f"[错误] 节点 {NODE} 未运行")
            return False
        try:
            result = subprocess.run(
                ["ros2", "param", "set", NODE, param, str(value)],
                capture_output=True, text=True, timeout=3.0  # ros2命令本身需要2秒左右
            )
            return result.returncode == 0
        except subprocess.TimeoutExpired:
            print(f"[超时] 参数 {param} 设置超时")
            self._node_check_cache = None  # 清除缓存
            return False
        except Exception as e:
            print(f"[错误] 设置参数 {param} 失败: {e}")
            return False

    def get_all_gains(self) -> dict:
        """获取所有Cascade P+PI增益（优化版：显示进度）"""
        gains = {"pos_Kp": [], "vel_Kp": [], "vel_Ki": [], "vel_limit": []}
        print("读取参数中", end="", flush=True)
        for i in range(1, 7):
            print(".", end="", flush=True)
            pos_kp = self.ros2_param_get(f"cascade_pid.joint_{i}.pos_Kp")
            vel_kp = self.ros2_param_get(f"cascade_pid.joint_{i}.vel_Kp")
            vel_ki = self.ros2_param_get(f"cascade_pid.joint_{i}.vel_Ki")
            vel_limit = self.ros2_param_get(f"cascade_pid.joint_{i}.vel_limit")
            gains["pos_Kp"].append(pos_kp if pos_kp else 0.0)
            gains["vel_Kp"].append(vel_kp if vel_kp else 0.0)
            gains["vel_Ki"].append(vel_ki if vel_ki else 0.0)
            gains["vel_limit"].append(vel_limit if vel_limit else 0.0)
        print(" 完成")
        return gains

    def print_current_gains(self):
        """显示当前增益"""
        gains = self.get_all_gains()
        print("\n" + "="*70)
        print("当前 Cascade P+PI 增益 (外环P → 内环PI):")
        print("-"*70)
        print(f"  关节:    J1      J2      J3      J4      J5      J6")
        print(f"  pos_Kp: {gains['pos_Kp'][0]:6.1f}  {gains['pos_Kp'][1]:6.1f}  {gains['pos_Kp'][2]:6.1f}  "
              f"{gains['pos_Kp'][3]:6.1f}  {gains['pos_Kp'][4]:6.1f}  {gains['pos_Kp'][5]:6.1f}")
        print(f"  vel_Kp: {gains['vel_Kp'][0]:6.2f}  {gains['vel_Kp'][1]:6.2f}  {gains['vel_Kp'][2]:6.2f}  "
              f"{gains['vel_Kp'][3]:6.2f}  {gains['vel_Kp'][4]:6.2f}  {gains['vel_Kp'][5]:6.2f}")
        print(f"  vel_Ki: {gains['vel_Ki'][0]:6.2f}  {gains['vel_Ki'][1]:6.2f}  {gains['vel_Ki'][2]:6.2f}  "
              f"{gains['vel_Ki'][3]:6.2f}  {gains['vel_Ki'][4]:6.2f}  {gains['vel_Ki'][5]:6.2f}")
        print("="*70)

    def set_joint_gain(self, joint: int, pos_kp: float = None, vel_kp: float = None, vel_ki: float = None):
        """设置单关节Cascade PID增益"""
        if pos_kp is not None:
            param = f"cascade_pid.joint_{joint}.pos_Kp"
            if self.ros2_param_set(param, pos_kp):
                print(f"  [OK] {param} = {pos_kp}")
            else:
                print(f"  [失败] {param}")
        if vel_kp is not None:
            param = f"cascade_pid.joint_{joint}.vel_Kp"
            if self.ros2_param_set(param, vel_kp):
                print(f"  [OK] {param} = {vel_kp}")
            else:
                print(f"  [失败] {param}")
        if vel_ki is not None:
            param = f"cascade_pid.joint_{joint}.vel_Ki"
            if self.ros2_param_set(param, vel_ki):
                print(f"  [OK] {param} = {vel_ki}")
                print(f"  [失败] {param}")

    def set_all_gains(self, pos_kp_list: List[float], vel_kp_list: List[float], vel_ki_list: List[float] = None):
        """批量设置所有增益"""
        print("\n批量设置增益...")
        if vel_ki_list is None:
            vel_ki_list = [0.0] * 6  # 默认不使用积分
        for i in range(6):
            self.set_joint_gain(i+1, pos_kp_list[i], vel_kp_list[i], vel_ki_list[i])
        print("完成!")

    def menu(self):
        """主菜单"""
        print("\n" + "="*50)
        print("  ARV_V1 Cascade P+PI 调参工具")
        print("="*50)
        print("  [1] 查看当前增益")
        print("  [2] 调单关节 pos_Kp/vel_Kp/vel_Ki")
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
            kp_str = input(f"新 pos_Kp 外环位置增益 (回车跳过): ").strip()
            vel_kp_str = input(f"新 vel_Kp 内环速度增益 (回车跳过): ").strip()
            vel_ki_str = input(f"新 vel_Ki 内环积分增益 (回车跳过): ").strip()
            pos_kp = float(kp_str) if kp_str else None
            vel_kp = float(vel_kp_str) if vel_kp_str else None
            vel_ki = float(vel_ki_str) if vel_ki_str else None
            self.set_joint_gain(j, pos_kp, vel_kp, vel_ki)
        except ValueError:
            print("[输入错误]")

    def _tune_all_joints(self):
        """批量调参"""
        self.print_current_gains()
        print("\n输入6个值，空格分隔 (回车跳过该行):")
        try:
            pos_kp_str = input("pos_Kp [J1-J6]: ").strip()
            vel_kp_str = input("vel_Kp [J1-J6]: ").strip()
            vel_ki_str = input("vel_Ki [J1-J6]: ").strip()
            if pos_kp_str:
                pos_kp_list = [float(x) for x in pos_kp_str.split()]
                if len(pos_kp_list) == 6:
                    for i, kp in enumerate(pos_kp_list):
                        self.set_joint_gain(i+1, pos_kp=kp)
            if vel_kp_str:
                vel_kp_list = [float(x) for x in vel_kp_str.split()]
                if len(vel_kp_list) == 6:
                    for i, kp in enumerate(vel_kp_list):
                        self.set_joint_gain(i+1, vel_kp=kp)
            if vel_ki_str:
                vel_ki_list = [float(x) for x in vel_ki_str.split()]
                if len(vel_ki_list) == 6:
                    for i, ki in enumerate(vel_ki_list):
                        self.set_joint_gain(i+1, vel_ki=ki)
        except ValueError:
            print("[输入格式错误]")

    def _apply_template(self):
        """预设模板"""
        templates = {
            "1": ("超保守(极低增益)", [0.5,1.0,0.5,0.2,0.1,0.1], [1.0,1.0,1.0,0.3,1.0,1.0], [0]*6),
            "2": ("保守(当前)", [3.0,5.0,3.0,0.5,0.2,0.2], [1.0,1.0,1.0,0.3,1.0,1.0], [0]*6),
            "3": ("中等(PD等效)", [30,50,30,33.3,1,1], [1.0,1.0,1.0,0.3,1.0,1.0], [0]*6),
        }
        print("\n增益模板 (Cascade P+PI):")
        for k, (name, pos_kp, vel_kp, vel_ki) in templates.items():
            print(f"  [{k}] {name}")
            print(f"      pos_Kp: {pos_kp}")
            print(f"      vel_Kp: {vel_kp}")
            print(f"      vel_Ki: {vel_ki}")
        choice = input("选择模板 [1-3]: ").strip()
        if choice in templates:
            _, pos_kp, vel_kp, vel_ki = templates[choice]
            self.set_all_gains(pos_kp, vel_kp, vel_ki)

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
    result = subprocess.run(["ros2", "node", "list"], capture_output=True, text=True, timeout=2)
    if NODE.replace("/","") not in result.stdout:
        print(f"[警告] 节点 {NODE} 未运行!")
        print("请先启动: ./start_mujoco_system.sh")
        sys.exit(1)
    else:
        print(f"[OK] 节点 {NODE} 在线")
    TunerCLI().run()
