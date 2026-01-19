#!/usr/bin/env python3
"""
ARV_V1 重力标定工具 - 静态多姿态采集 + 最小二乘拟合
原理: 在多个静止姿态下，保持力矩 = 重力矩 τ_hold = G(q)
用法: python3 calibrate_gravity.py
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
import json
from datetime import datetime

class GravityCalibrator(Node):
    def __init__(self):
        super().__init__('gravity_calibrator')

        # 数据存储
        self.samples = []  # [(q, tau), ...]
        self.current_q = None
        self.current_tau = None
        self.collecting = False
        self.collect_buffer = []

        # 订阅
        self.js_sub = self.create_subscription(
            JointState, '/joint_states', self.js_cb, 10)
        self.tau_sub = self.create_subscription(
            Float64MultiArray, '/effort_controller/commands', self.tau_cb, 10)

        self.get_logger().info('重力标定工具已启动')
        self.get_logger().info('确保机械臂处于力矩控制模式（保持静止）')

    def js_cb(self, msg):
        if len(msg.position) >= 6:
            self.current_q = np.array(msg.position[:6])
            if self.collecting:
                self.collect_buffer.append(('q', self.current_q.copy()))

    def tau_cb(self, msg):
        if len(msg.data) >= 6:
            self.current_tau = np.array(msg.data[:6])
            if self.collecting:
                self.collect_buffer.append(('tau', self.current_tau.copy()))

    def collect_sample(self, duration=2.0):
        """采集当前姿态的稳态数据"""
        self.collect_buffer = []
        self.collecting = True

        print(f"  采集中 ({duration}s)...", end='', flush=True)
        time.sleep(duration)

        self.collecting = False

        # 提取数据
        qs = [d[1] for d in self.collect_buffer if d[0] == 'q']
        taus = [d[1] for d in self.collect_buffer if d[0] == 'tau']

        if len(qs) < 10 or len(taus) < 10:
            print(" 数据不足!")
            return False

        # 计算均值（滤除噪声）
        q_mean = np.mean(qs, axis=0)
        tau_mean = np.mean(taus, axis=0)

        # 计算标准差（检查是否静止）
        q_std = np.std(qs, axis=0)
        tau_std = np.std(taus, axis=0)

        if np.max(q_std) > 0.01:  # 位置波动 > 0.01 rad
            print(f" 警告: 位置不稳定 (std={np.max(q_std):.4f})")
            return False

        self.samples.append({
            'q': q_mean.tolist(),
            'tau': tau_mean.tolist(),
            'q_std': q_std.tolist(),
            'tau_std': tau_std.tolist()
        })

        print(f" OK (采集 {len(qs)} 帧)")
        return True

    def print_current_state(self):
        """显示当前状态"""
        if self.current_q is None:
            print("  [等待数据...]")
            return

        print("\n当前状态:")
        print(f"  位置 q  = [{', '.join([f'{x:7.3f}' for x in self.current_q])}] rad")
        if self.current_tau is not None:
            print(f"  力矩 τ  = [{', '.join([f'{x:7.3f}' for x in self.current_tau])}] N·m")
        print(f"  已采集: {len(self.samples)} 个姿态")

    def save_data(self):
        """保存采集数据"""
        if not self.samples:
            print("无数据可保存")
            return None

        filename = f"gravity_calib_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
        data = {
            'timestamp': datetime.now().isoformat(),
            'num_samples': len(self.samples),
            'samples': self.samples
        }

        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)

        print(f"\n数据已保存: {filename}")
        return filename

    def analyze(self):
        """分析采集的数据"""
        if len(self.samples) < 3:
            print("至少需要3个姿态才能分析")
            return

        print("\n" + "="*60)
        print("  重力标定分析")
        print("="*60)

        # 提取数据
        Q = np.array([s['q'] for s in self.samples])    # (N, 6)
        Tau = np.array([s['tau'] for s in self.samples]) # (N, 6)

        print(f"\n采集姿态数: {len(self.samples)}")

        # 对每个关节分析
        for j in range(6):
            q_j = Q[:, j]
            tau_j = Tau[:, j]

            # 简单分析：力矩与sin(q)的相关性（重力主要表现）
            sin_q = np.sin(q_j)
            cos_q = np.cos(q_j)

            # 线性回归: tau = a*sin(q) + b*cos(q) + c
            # 这是简化模型，实际应考虑耦合
            A = np.column_stack([sin_q, cos_q, np.ones_like(q_j)])
            coeffs, residuals, _, _ = np.linalg.lstsq(A, tau_j, rcond=None)

            # 预测
            tau_pred = A @ coeffs
            rmse = np.sqrt(np.mean((tau_j - tau_pred)**2))

            print(f"\nJoint {j+1}:")
            print(f"  力矩范围: [{tau_j.min():.3f}, {tau_j.max():.3f}] N·m")
            print(f"  拟合系数: a={coeffs[0]:.3f}, b={coeffs[1]:.3f}, c={coeffs[2]:.3f}")
            print(f"  拟合RMSE: {rmse:.4f} N·m")

            if rmse > 0.5:
                print(f"  [!] RMSE较大，可能需要更多姿态或检查模型")

        print("\n" + "-"*60)
        print("说明:")
        print("  - a*sin(q) + b*cos(q) 近似重力矩")
        print("  - c 是偏置（摩擦/零漂）")
        print("  - RMSE < 0.1 说明拟合良好")
        print("  - 后续可用这些系数修正URDF或前馈")
        print("="*60)


def main():
    rclpy.init()
    node = GravityCalibrator()

    # 等待数据
    print("等待话题数据...")
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.current_q is not None:
            break

    if node.current_q is None:
        print("[错误] 未收到 /joint_states，请检查系统")
        return

    print("\n" + "="*50)
    print("  重力标定 - 多姿态采集")
    print("="*50)
    print("\n操作流程:")
    print("  1. 用示教器/RViz将机械臂移到某个姿态")
    print("  2. 确保机械臂静止（力矩保持模式）")
    print("  3. 按回车采集当前姿态")
    print("  4. 重复上述步骤，采集至少5个不同姿态")
    print("  5. 输入 'done' 完成采集并分析")
    print("\n建议姿态:")
    print("  - 各关节分别在 0°, 45°, 90°, -45° 等位置")
    print("  - 覆盖实际工作范围")

    while True:
        node.print_current_state()

        # 处理ROS消息
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)

        cmd = input("\n[回车=采集 / done=完成 / q=退出]: ").strip().lower()

        if cmd == 'q':
            break
        elif cmd == 'done':
            node.save_data()
            node.analyze()
            break
        else:
            # 采集当前姿态
            print(f"\n采集姿态 #{len(node.samples)+1}:")

            # 持续spin以采集数据
            start = time.time()
            while time.time() - start < 2.0:
                rclpy.spin_once(node, timeout_sec=0.01)

            node.collect_sample(duration=0.1)  # 已经spin过了，这里只处理buffer

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
