#!/usr/bin/env python3
"""
力矩方向验证脚本 — 逐关节发 1Nm，观察编码器响应
用法: python3 torque_test.py
前提: 只启动 hardware_interface_node (force_zero_torque:=false)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import time
import sys
import signal

NUM_JOINTS = 7  # 6 arm + 1 gripper
TORQUE = 1.0    # Nm
DURATION = 1.0  # 秒

URDF_AXIS = ['+Z', '+Z', '-Z', '+Z', '-Z', '+Z']
JOINT_NAMES = ['J1(腰)', 'J2(肩)', 'J3(肘)', 'J4(腕旋)', 'J5(腕摆)', 'J6(末端)']


class TorqueTestNode(Node):
    def __init__(self):
        super().__init__('torque_test')
        self.pub = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 10)
        self.latest_js = None
        self.sub = self.create_subscription(JointState, '/joint_states', self.js_cb, 10)
        signal.signal(signal.SIGINT, self.emergency_stop)

    def js_cb(self, msg):
        self.latest_js = msg

    def send_zero(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * NUM_JOINTS
        for _ in range(10):
            self.pub.publish(msg)
            time.sleep(0.005)

    def emergency_stop(self, *args):
        self.send_zero()
        print('\n[EMERGENCY STOP] All zeros sent.')
        sys.exit(0)

    def read_position(self):
        rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_js is None:
            return None
        return list(self.latest_js.position)

    def test_joint(self, joint_idx):
        name = JOINT_NAMES[joint_idx]
        axis = URDF_AXIS[joint_idx]

        pos_before = self.read_position()
        if pos_before is None:
            self.get_logger().error('No /joint_states!')
            return

        print(f'  Pos before: {pos_before[joint_idx]:+.4f} rad')
        print(f'  Sending +{TORQUE} Nm for {DURATION}s ...', flush=True)

        msg = Float64MultiArray()
        msg.data = [0.0] * NUM_JOINTS
        msg.data[joint_idx] = TORQUE

        t0 = time.time()
        while time.time() - t0 < DURATION:
            self.pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.005)

        self.send_zero()
        time.sleep(0.1)

        pos_after = self.read_position()
        if pos_after is None:
            return

        delta = pos_after[joint_idx] - pos_before[joint_idx]
        direction = '+' if delta > 0 else '-'

        print(f'  Pos after:  {pos_after[joint_idx]:+.4f} rad')
        print(f'  Delta:      {delta:+.4f} rad')
        print(f'  结论: +1Nm → 编码器{direction}方向, URDF axis={axis}')
        if (delta > 0 and '+' in axis) or (delta < 0 and '-' in axis):
            print(f'  ✓ 方向一致')
        elif abs(delta) < 0.0005:
            print(f'  ? 几乎没动（摩擦太大或力矩未到电机）')
        else:
            print(f'  ✗ 方向相反! 需要翻转符号')
        print()


def main():
    rclpy.init()
    node = TorqueTestNode()

    print('Waiting for /joint_states ...')
    for _ in range(50):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.latest_js is not None:
            break
    else:
        print('[ERROR] No /joint_states. Is hardware_interface_node running?')
        return

    print(f'OK, got {len(node.latest_js.position)} joints\n')
    node.send_zero()

    for i in range(6):
        print(f'==== {JOINT_NAMES[i]} (URDF axis: {URDF_AXIS[i]}) ====')
        input(f'  按 Enter 测试 {JOINT_NAMES[i]}，用手扶住准备...')
        node.test_joint(i)
        time.sleep(0.3)

    print('====== 全部完成 ======')
    node.send_zero()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
