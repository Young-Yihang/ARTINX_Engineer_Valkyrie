#!/usr/bin/env python3
"""
ARV_V1 重力补偿全自动标定

模型: tau_J2 = A*cos(q2) + C*cos(q2+q3) + D*sin(q2+q3)

全自动流程:
  1. 发 /joint_position_target 移臂到目标 (q2,q3)
  2. 等静止 (J2/J3 速度 < 阈值持续 2s)
  3. 采集 2s 的 /effort_controller/commands J2 分量
  4. 下一个点
  5. 最小二乘拟合 A,C,D

前提: 系统已启动，ARMED hold 模式

用法:
  python3 gravity_calibration.py           # 全自动
  python3 gravity_calibration.py --manual  # 手动输入已有数据
"""

import argparse
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

Q2_RANGE = (0.95, 2.80)
Q3_RANGE = (-0.80, 0.70)


def make_grid():
    q2s = np.linspace(Q2_RANGE[0] + 0.15, Q2_RANGE[1] - 0.15, 6)
    q3s = np.linspace(Q3_RANGE[0] + 0.1, Q3_RANGE[1] - 0.1, 5)
    grid = []
    for q2 in q2s:
        for q3 in q3s:
            if q2 < 0.8 and q3 < -1.0:
                continue
            if q2 > 2.8 and q3 > 1.0:
                continue
            grid.append((q2, q3))
    return grid


class CalibNode(Node):
    def __init__(self):
        super().__init__("gravity_calibration")
        self._js = None
        self._eff = None
        self._lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_subscription(Float64MultiArray, "/effort_controller/commands", self._eff_cb, 10)
        self.target_pub = self.create_publisher(Float64MultiArray, "/joint_position_target", 10)

    def _js_cb(self, msg):
        with self._lock:
            self._js = msg

    def _eff_cb(self, msg):
        with self._lock:
            self._eff = list(msg.data) if msg.data else None

    def get(self):
        with self._lock:
            return self._js, self._eff

    def send_target(self, q2, q3):
        js, _ = self.get()
        if js is None or len(js.position) < 6:
            return
        msg = Float64MultiArray()
        msg.data = [
            js.position[0],  # J1: 保持当前
            q2,
            q3,
            0.0,             # J4: 固定 0
            0.0,             # J5: 固定 0
            js.position[5],  # J6: 保持当前
        ]
        self.target_pub.publish(msg)


def wait_stable(node, duration=2.0, vel_thresh=0.05):
    stable_since = None
    while True:
        rclpy.spin_once(node, timeout_sec=0.02)
        js, _ = node.get()
        if js is None or len(js.velocity) < 3:
            time.sleep(0.01)
            continue
        if abs(js.velocity[1]) < vel_thresh and abs(js.velocity[2]) < vel_thresh:
            if stable_since is None:
                stable_since = time.monotonic()
            elif time.monotonic() - stable_since > duration:
                return
        else:
            stable_since = None
        time.sleep(0.01)


def sample(node, duration=2.0):
    q2s, q3s, taus = [], [], []
    t0 = time.monotonic()
    while time.monotonic() - t0 < duration:
        rclpy.spin_once(node, timeout_sec=0.02)
        js, eff = node.get()
        if js and eff and len(js.position) >= 3 and len(eff) >= 2:
            q2s.append(js.position[1])
            q3s.append(js.position[2])
            taus.append(eff[1])
        time.sleep(0.005)
    if len(q2s) < 20:
        return None
    return np.mean(q2s), np.mean(q3s), np.mean(taus)


def collect_auto():
    rclpy.init()
    node = CalibNode()
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()
    time.sleep(1.0)

    # 等待 joint_states 就绪
    print("等待 /joint_states...", end="", flush=True)
    while True:
        js, _ = node.get()
        if js and len(js.position) >= 6:
            break
        time.sleep(0.1)
    print(" ✓\n")

    grid = make_grid()
    print(f"全自动标定：{len(grid)} 个测量点")
    print(f"臂会自动移动到每个目标位置，请确保 ARMED 模式\n")

    input("按 Enter 开始标定（确保周围安全）...")
    print()

    data = []
    for i, (q2t, q3t) in enumerate(grid):
        print(f"[{i+1}/{len(grid)}] → q2={q2t:.2f}({np.degrees(q2t):.0f}°) q3={q3t:.2f}({np.degrees(q3t):.0f}°)")

        # 持续发目标直到到位
        print("  移动中...", end="", flush=True)
        t_move = time.monotonic()
        while True:
            node.send_target(q2t, q3t)
            time.sleep(0.05)
            js, _ = node.get()
            if js and len(js.position) >= 3:
                err2 = abs(js.position[1] - q2t)
                err3 = abs(js.position[2] - q3t)
                if err2 < 0.05 and err3 < 0.05:
                    break
            if time.monotonic() - t_move > 15.0:
                print(" 超时!", flush=True)
                break
        print(" 到位", flush=True)

        # 等静止
        print("  等待静止...", end="", flush=True)
        # 继续发目标保持 hold
        def keep_target():
            while getattr(keep_target, 'running', True):
                node.send_target(q2t, q3t)
                time.sleep(0.1)
        keep_target.running = True
        t = threading.Thread(target=keep_target, daemon=True)
        t.start()

        wait_stable(node)
        print(" ✓", flush=True)

        # 采集
        print("  采集 2s...", end="", flush=True)
        result = sample(node)
        keep_target.running = False

        if result is None:
            print(" ⚠ 跳过")
            continue
        q2, q3, tau = result
        data.append((q2, q3, tau))
        print(f" ✓ q2={q2:.3f} q3={q3:.3f} tau={tau:.3f} Nm")

    rclpy.shutdown()
    return data


def collect_manual():
    print("手动输入: q2 q3 tau_J2（空格分隔），空行结束\n")
    data = []
    while True:
        line = input(f"  [{len(data)+1}] > ").strip()
        if not line:
            break
        try:
            vals = [float(x) for x in line.split()]
            assert len(vals) == 3
            data.append(tuple(vals))
        except (ValueError, AssertionError):
            print("  格式: q2 q3 tau_J2")
    return data


def fit_and_report(data):
    n = len(data)
    if n < 3:
        print("至少需要 3 个点！")
        return

    H = np.zeros((n, 3))
    tau = np.zeros(n)
    for i, (q2, q3, t) in enumerate(data):
        H[i] = [np.cos(q2), np.cos(q2 + q3), np.sin(q2 + q3)]
        tau[i] = t

    coeff, _, _, _ = np.linalg.lstsq(H, tau, rcond=None)
    A, C, D = coeff
    pred = H @ coeff
    err = tau - pred
    rmse = np.sqrt(np.mean(err**2))

    print(f"\n{'='*60}")
    print(f"拟合结果（{n} 个点, RMSE={rmse:.4f} Nm）")
    print(f"{'='*60}")
    print(f"  kG1A = {A:.3f}")
    print(f"  kG1C = {C:.3f}")
    print(f"  kG1D = {D:.3f}")
    print()
    print(f"  对比: kG1A {A:.3f} (老 20.607 Δ{A-20.607:+.3f})")
    print(f"        kG1C {C:.3f} (老 1.302  Δ{C-1.302:+.3f})")
    print(f"        kG1D {D:.3f} (老 5.610  Δ{D-5.610:+.3f})")
    print()

    print(f"{'q2':>7s} {'q3':>7s} {'实测':>8s} {'拟合':>8s} {'误差':>8s}")
    for i, (q2, q3, t) in enumerate(data):
        print(f"{q2:7.3f} {q3:7.3f} {t:8.3f} {pred[i]:8.3f} {err[i]:+8.4f}")

    print(f"\n{'='*60}")
    print("下位机 (ArmJointConfig.hpp):")
    print(f"{'='*60}")
    print(f"static constexpr float kG1A = {A:.3f}f;")
    print(f"static constexpr float kG1C = {C:.3f}f;")
    print(f"static constexpr float kG1D = {D:.3f}f;")
    print(f"static constexpr float kG2C = {C:.3f}f;")
    print(f"static constexpr float kG2D = {D:.3f}f;")

    print(f"\n上位机 URDF 反推参考（基于新臂 L2=440mm）:")
    print(f"  m2*Lc2 + m34*L2 = {A/9.81:.4f} kg·m")
    print(f"  m34*Lc34_perp   = {C/9.81:.4f} kg·m")
    print(f"  m34*Lc34_along  = {D/9.81:.4f} kg·m")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manual", action="store_true")
    args = parser.parse_args()
    data = collect_manual() if args.manual else collect_auto()
    if data:
        fit_and_report(data)


if __name__ == "__main__":
    main()

