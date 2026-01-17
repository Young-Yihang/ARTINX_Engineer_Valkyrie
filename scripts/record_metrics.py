#!/usr/bin/env python3
"""
ARV_V1 性能记录与评估工具 (Cascade P+PI)
订阅 /joint_states 和 action feedback，实时计算控制性能指标
用法: python3 record_metrics.py [--duration 10] [--show-params]
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
import argparse
from dataclasses import dataclass, field
from typing import List
import csv
from datetime import datetime
import subprocess

@dataclass
class JointData:
    """单关节数据缓冲"""
    positions: List[float] = field(default_factory=list)
    velocities: List[float] = field(default_factory=list)
    desired_pos: List[float] = field(default_factory=list)
    desired_vel: List[float] = field(default_factory=list)
    timestamps: List[float] = field(default_factory=list)

@dataclass
class Metrics:
    """性能指标"""
    rmse_pos: float = 0.0
    rmse_vel: float = 0.0
    max_pos_error: float = 0.0
    max_vel_error: float = 0.0
    mean_pos_error: float = 0.0

class MetricsRecorder(Node):
    def __init__(self, duration: float = 10.0, show_params: bool = False):
        super().__init__('metrics_recorder')
        self.duration = duration
        self.show_params = show_params
        self.start_time = None
        self.joint_data = [JointData() for _ in range(6)]
        self.torques = [[] for _ in range(6)]
        self.torque_saturated = [0 for _ in range(6)]
        self.TORQUE_LIMIT = 20.0  # N·m 饱和阈值

        # 订阅
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.js_callback, 10)
        self.torque_sub = self.create_subscription(
            Float64MultiArray, '/effort_controller/commands', self.torque_callback, 10)

        self.get_logger().info(f'开始记录 {duration}s ...')
        self.get_logger().info('在 RViz 中执行轨迹以采集数据')
        self.start_time = time.time()

    def js_callback(self, msg: JointState):
        if len(msg.position) < 6:
            return
        t = time.time() - self.start_time
        for i in range(6):
            self.joint_data[i].positions.append(msg.position[i])
            self.joint_data[i].velocities.append(msg.velocity[i] if len(msg.velocity)>i else 0)
            self.joint_data[i].timestamps.append(t)

    def torque_callback(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return
        for i in range(6):
            tau = msg.data[i]
            self.torques[i].append(tau)
            if abs(tau) >= self.TORQUE_LIMIT * 0.95:
                self.torque_saturated[i] += 1

    def compute_metrics(self) -> List[Metrics]:
        """计算各关节指标"""
        results = []
        for i in range(6):
            jd = self.joint_data[i]
            m = Metrics()
            if len(jd.positions) < 10:
                results.append(m)
                continue

            pos = np.array(jd.positions)
            vel = np.array(jd.velocities)

            # 用第一个位置作为参考(保持模式)或计算相对误差
            if len(jd.desired_pos) > 0:
                des_pos = np.array(jd.desired_pos[:len(pos)])
                pos_err = pos[:len(des_pos)] - des_pos
            else:
                # 无期望轨迹时，用位置变化量评估稳定性
                pos_err = pos - pos[0]

            m.rmse_pos = np.sqrt(np.mean(pos_err**2))
            m.max_pos_error = np.max(np.abs(pos_err))
            m.mean_pos_error = np.mean(np.abs(pos_err))
            m.rmse_vel = np.sqrt(np.mean(vel**2))
            m.max_vel_error = np.max(np.abs(vel))
            results.append(m)
        return results

    def print_report(self):
        """打印性能报告"""
        metrics = self.compute_metrics()
        samples = len(self.joint_data[0].positions)

        print("\n" + "="*70)
        print(f"  ARV_V1 控制性能报告 (Cascade P+PI) | 采样: {samples} | 时长: {self.duration:.1f}s")
        print("="*70)
        
        # 如果需要显示参数
        if self.show_params:
            self._print_cascade_params()
            print("-"*70)
        
        print(f"{'关节':<6} {'RMSE位置(rad)':<14} {'最大误差':<12} {'RMSE速度':<12} {'力矩饱和':<10}")
        print("-"*70)

        for i in range(6):
            m = metrics[i]
            sat = self.torque_saturated[i]
            sat_str = f"{sat}" if sat == 0 else f"\033[91m{sat}\033[0m"  # 红色警告
            print(f"J{i+1:<5} {m.rmse_pos:<14.6f} {m.max_pos_error:<12.6f} "
                  f"{m.rmse_vel:<12.4f} {sat_str:<10}")

        print("-"*70)
        # 总体评估
        avg_rmse = np.mean([m.rmse_pos for m in metrics])
        total_sat = sum(self.torque_saturated)

        print(f"\n综合评估:")
        print(f"  平均位置RMSE: {avg_rmse:.6f} rad ({np.degrees(avg_rmse):.4f}°)")
        if total_sat > 0:
            print(f"  \033[91m[警告]\033[0m 力矩饱和 {total_sat} 次，考虑降低增益")
        if avg_rmse > 0.01:
            print(f"  \033[93m[建议]\033[0m RMSE较大，可尝试增大 pos_Kp 或 vel_Kp")
        elif avg_rmse < 0.001:
            print(f"  \033[92m[良好]\033[0m 跟踪精度优秀")
        print("="*70)

    def _print_cascade_params(self):
        """打印当前级联PID参数"""
        print("\n当前Cascade P+PI参数:")
        print(f"  {'关节':<8} {'pos_Kp':<10} {'vel_Kp':<10} {'vel_Ki':<10} {'vel_limit':<10}")
        
        for i in range(1, 7):
            pos_kp = self._ros2_param_get(f"cascade_pid.joint_{i}.pos_Kp")
            vel_kp = self._ros2_param_get(f"cascade_pid.joint_{i}.vel_Kp")
            vel_ki = self._ros2_param_get(f"cascade_pid.joint_{i}.vel_Ki")
            vel_limit = self._ros2_param_get(f"cascade_pid.joint_{i}.vel_limit")
            
            print(f"  J{i:<7} {pos_kp:<10.2f} {vel_kp:<10.2f} {vel_ki:<10.4f} {vel_limit:<10.2f}")
    
    def _ros2_param_get(self, param_name: str) -> float:
        """获取ROS2参数值"""
        try:
            result = subprocess.run(
                ['ros2', 'param', 'get', '/torque_controller_action_server', param_name],
                capture_output=True, text=True, timeout=2
            )
            if result.returncode == 0:
                # 输出格式: "Double value is: 3.0"
                value_str = result.stdout.strip().split(':')[-1].strip()
                return float(value_str)
        except:
            pass
        return 0.0

    def save_csv(self):
        """保存原始数据到CSV"""
        filename = f"metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        with open(filename, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time'] + [f'j{i+1}_pos' for i in range(6)] +
                          [f'j{i+1}_vel' for i in range(6)])

            n = min(len(self.joint_data[i].positions) for i in range(6))
            for k in range(n):
                row = [self.joint_data[0].timestamps[k]]
                row += [self.joint_data[i].positions[k] for i in range(6)]
                row += [self.joint_data[i].velocities[k] for i in range(6)]
                writer.writerow(row)
        print(f"\n数据已保存: {filename}")

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--duration', '-d', type=float, default=10.0, help='记录时长(秒)')
    parser.add_argument('--save', '-s', action='store_true', help='保存CSV')
    parser.add_argument('--show-params', '-p', action='store_true', help='显示Cascade P+PI参数')
    args = parser.parse_args()

    rclpy.init()
    node = MetricsRecorder(args.duration, args.show_params)

    end_time = time.time() + args.duration
    while time.time() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.print_report()
    if args.save:
        node.save_csv()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
