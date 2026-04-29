#!/usr/bin/env python3
"""
ARV_V1 Mission Panel — tkinter GUI for operating the headless mission_executor.

Usage:
  ros2 run arv_v1_moveit mission_panel.py
"""

import math
import threading
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8, Int32
from arv_v1_interfaces.srv import GripperControl, ListTrajectories, LoadTrajectory

MODE_NAMES = {0: "RELAX", 1: "FREEDRIVE", 2: "ARMED"}
MODE_COLORS = {0: "#cc3333", 1: "#cc9900", 2: "#33aa33"}
ARM_STATES = {0x00: "READY", 0x01: "EXEC", 0x02: "HOLD", 0x05: "RELAX", 0x06: "FREE"}

BG = "#1a1a2e"
FG = "#e0e0e0"
ACCENT = "#0f3460"
BTN_BG = "#16213e"
ENTRY_BG = "#0f3460"
LOG_BG = "#0d1117"
WARN_FG = "#e2b93d"
ERR_FG = "#f85149"
OK_FG = "#3fb950"


def rpy_from_quat(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


# ── ROS2 Node ──────────────────────────────────────────

class PanelNode(Node):
    def __init__(self):
        super().__init__("mission_panel")
        self.joints = None
        self.pose = None
        self.control_mode = 0
        self.arm_state = 0
        self._lock = threading.Lock()
        self.log_lines = []
        self._log_lock = threading.Lock()
        self._task_seq = 0

        best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_subscription(PoseStamped, "/cartesian_controller/current_pose", self._pose_cb, 10)
        self.create_subscription(UInt8, "/control_mode", self._mode_cb, 10)
        self.create_subscription(UInt8, "/arm_state", self._arm_state_cb, 10)
        self.create_subscription(Log, "/rosout", self._log_cb, best_effort)

        self.mode_pub = self.create_publisher(UInt8, "/control_mode", 10)
        self.task_pub = self.create_publisher(Int32, "/task_command", 10)
        self.gripper_cli = self.create_client(GripperControl, "/gripper_control")
        self.list_cli = self.create_client(ListTrajectories, "/list_trajectories")
        self.load_cli = self.create_client(LoadTrajectory, "/load_trajectory")

    def _js_cb(self, msg):
        with self._lock:
            self.joints = list(msg.position[:7]) if len(msg.position) >= 6 else None

    def _pose_cb(self, msg):
        with self._lock:
            self.pose = msg.pose

    def _mode_cb(self, msg):
        with self._lock:
            self.control_mode = msg.data

    def _arm_state_cb(self, msg):
        with self._lock:
            self.arm_state = msg.data

    def _log_cb(self, msg):
        sev = {10: "DBG", 20: "INF", 30: "WRN", 40: "ERR", 50: "FTL"}.get(msg.level, "???")
        name = msg.name.split(".")[-1][:16] if msg.name else "?"
        line = f"[{sev}] {name}: {msg.msg}"
        with self._log_lock:
            self.log_lines.append((line, sev))
            if len(self.log_lines) > 500:
                self.log_lines = self.log_lines[-300:]

    def pub_mode(self, mode):
        m = UInt8()
        m.data = mode
        self.mode_pub.publish(m)

    def pub_task(self, cmd, param=0):
        self._task_seq = (self._task_seq + 1) % 256
        m = Int32()
        m.data = (cmd << 16) | (param << 8) | self._task_seq
        self.task_pub.publish(m)

    def get_state(self):
        with self._lock:
            return dict(joints=list(self.joints) if self.joints else None,
                        pose=self.pose, mode=self.control_mode, arm_state=self.arm_state)

    def get_new_logs(self, since):
        with self._log_lock:
            return self.log_lines[since:]

    @staticmethod
    def _wait_future(fut, timeout=3.0):
        """Poll future without spin (executor already spinning in bg thread)."""
        import time
        t0 = time.monotonic()
        while not fut.done() and (time.monotonic() - t0) < timeout:
            time.sleep(0.05)
        return fut.result() if fut.done() else None

    def call_gripper(self, force):
        if not self.gripper_cli.wait_for_service(timeout_sec=0.5):
            return "service not ready"
        req = GripperControl.Request()
        req.torque = float(force)
        res = self._wait_future(self.gripper_cli.call_async(req), 3.0)
        return ("OK" if res.success else res.message) if res else "timeout"

    def call_list_trajectories(self):
        if not self.list_cli.wait_for_service(timeout_sec=0.5):
            return []
        req = ListTrajectories.Request()
        res = self._wait_future(self.list_cli.call_async(req), 3.0)
        return list(res.names) if res else []

    def call_execute_trajectory(self, name):
        if not self.load_cli.wait_for_service(timeout_sec=0.5):
            return "service not ready"
        req = LoadTrajectory.Request()
        req.name = name
        req.execute = True
        res = self._wait_future(self.load_cli.call_async(req), 120.0)
        return ("OK" if res.success else res.message) if res else "timeout"


# ── GUI ────────────────────────────────────────────────

class MissionPanel:
    def __init__(self):
        rclpy.init()
        self._node = PanelNode()
        threading.Thread(target=lambda: rclpy.spin(self._node), daemon=True).start()

        self._log_offset = 0
        self._trajs = []
        self._build_gui()
        self._root.after(500, self._refresh_trajs)
        self._tick()

    def _build_gui(self):
        root = tk.Tk()
        root.title("ARV_V1 Mission Panel")
        root.configure(bg=BG)
        root.geometry("520x720")
        root.minsize(480, 600)
        self._root = root

        style = ttk.Style()
        style.theme_use("clam")
        style.configure(".", background=BG, foreground=FG, fieldbackground=ENTRY_BG,
                        borderwidth=0, font=("Segoe UI", 10))
        style.configure("TLabelframe", background=BG, foreground=FG)
        style.configure("TLabelframe.Label", background=BG, foreground=FG,
                        font=("Segoe UI", 10, "bold"))
        style.configure("TButton", background=BTN_BG, foreground=FG, padding=(8, 4))
        style.map("TButton", background=[("active", ACCENT)])
        style.configure("Accent.TButton", background="#e63946", foreground="white",
                        font=("Segoe UI", 10, "bold"))
        style.map("Accent.TButton", background=[("active", "#c1121f")])
        style.configure("Mode.TButton", padding=(10, 6), font=("Segoe UI", 10, "bold"))

        # ── Mode ──
        mf = ttk.LabelFrame(root, text="Control Mode")
        mf.pack(fill="x", padx=8, pady=(8, 4))

        top = ttk.Frame(mf)
        top.pack(fill="x", padx=4, pady=4)

        self._mode_label = tk.Label(top, text="RELAX", font=("JetBrains Mono", 16, "bold"),
                                    fg="white", bg="#cc3333", width=10, anchor="center")
        self._mode_label.pack(side="left", padx=(0, 8))

        self._arm_label = tk.Label(top, text="READY", font=("JetBrains Mono", 12),
                                   fg=OK_FG, bg=BG)
        self._arm_label.pack(side="left")

        btn_row = ttk.Frame(mf)
        btn_row.pack(fill="x", padx=4, pady=(0, 4))
        for val, name in MODE_NAMES.items():
            ttk.Button(btn_row, text=name, style="Mode.TButton",
                       command=lambda m=val: self._node.pub_mode(m)).pack(side="left", padx=2, expand=True, fill="x")

        # ── Gripper + Commands (side by side) ──
        row = ttk.Frame(root)
        row.pack(fill="x", padx=8, pady=4)

        gf = ttk.LabelFrame(row, text="Gripper")
        gf.pack(side="left", fill="both", expand=True, padx=(0, 4))
        for txt, force in [("GRIP", 70.0), ("RELEASE", -70.0), ("STOP", 0.0)]:
            ttk.Button(gf, text=txt, command=lambda f=force: self._async(
                lambda: self._node.call_gripper(f))).pack(fill="x", padx=4, pady=1)

        cf = ttk.LabelFrame(row, text="Commands")
        cf.pack(side="left", fill="both", expand=True, padx=(4, 0))
        cmds = [("ESTOP", 0x01, "Accent.TButton"), ("Reset", 0x02, "TButton"),
                ("Next", 0x30, "TButton"), ("Abort", 0x31, "TButton")]
        for txt, cmd, sty in cmds:
            ttk.Button(cf, text=txt, style=sty,
                       command=lambda c=cmd: self._node.pub_task(c)).pack(fill="x", padx=4, pady=1)

        # ── Trajectories ──
        tf = ttk.LabelFrame(root, text="Trajectories")
        tf.pack(fill="x", padx=8, pady=4)

        list_frame = ttk.Frame(tf)
        list_frame.pack(fill="x", padx=4, pady=2)

        self._traj_list = tk.Listbox(list_frame, height=5, font=("JetBrains Mono", 10),
                                     bg=ENTRY_BG, fg=FG, selectbackground=ACCENT,
                                     selectforeground="white", borderwidth=0, highlightthickness=0)
        scroll = ttk.Scrollbar(list_frame, orient="vertical", command=self._traj_list.yview)
        self._traj_list.configure(yscrollcommand=scroll.set)
        self._traj_list.pack(side="left", fill="x", expand=True)
        scroll.pack(side="right", fill="y")

        tb = ttk.Frame(tf)
        tb.pack(fill="x", padx=4, pady=(0, 4))
        ttk.Button(tb, text="Refresh", command=self._refresh_trajs).pack(side="left", padx=2)
        ttk.Button(tb, text="Execute", command=self._exec_traj).pack(side="left", padx=2)
        self._traj_status = tk.Label(tb, text="", font=("Segoe UI", 9), fg=FG, bg=BG)
        self._traj_status.pack(side="left", padx=6)

        # ── State ──
        sf = ttk.LabelFrame(root, text="Robot State")
        sf.pack(fill="x", padx=8, pady=4)

        mono = ("JetBrains Mono", 10)
        self._joints_lbl = tk.Label(sf, text="Joints: --", font=mono, fg=FG, bg=BG, anchor="w")
        self._joints_lbl.pack(fill="x", padx=6)
        self._pose_lbl = tk.Label(sf, text="Pose:   --", font=mono, fg=FG, bg=BG, anchor="w")
        self._pose_lbl.pack(fill="x", padx=6, pady=(0, 4))

        # ── Log ──
        lf = ttk.LabelFrame(root, text="Log (/rosout)")
        lf.pack(fill="both", expand=True, padx=8, pady=(4, 8))

        self._log_text = tk.Text(lf, font=("JetBrains Mono", 9), state="disabled",
                                 wrap="none", bg=LOG_BG, fg="#8b949e",
                                 insertbackground=FG, borderwidth=0, highlightthickness=0)
        ls = ttk.Scrollbar(lf, orient="vertical", command=self._log_text.yview)
        self._log_text.configure(yscrollcommand=ls.set)
        self._log_text.pack(side="left", fill="both", expand=True)
        ls.pack(side="right", fill="y")

        self._log_text.tag_configure("WRN", foreground=WARN_FG)
        self._log_text.tag_configure("ERR", foreground=ERR_FG)
        self._log_text.tag_configure("FTL", foreground="#ff0000", underline=True)
        self._log_text.tag_configure("INF", foreground="#8b949e")

    # ── Helpers ──

    def _async(self, fn, callback=None):
        def wrapper():
            res = fn()
            if callback:
                self._root.after(0, lambda: callback(res))
        threading.Thread(target=wrapper, daemon=True).start()

    def _refresh_trajs(self):
        def done(names):
            self._trajs = names
            self._traj_list.delete(0, tk.END)
            for n in names:
                self._traj_list.insert(tk.END, n)
            self._traj_status.configure(text=f"{len(names)} found", fg=FG)
        self._async(self._node.call_list_trajectories, done)

    def _exec_traj(self):
        sel = self._traj_list.curselection()
        if not sel:
            self._traj_status.configure(text="Select first", fg=WARN_FG)
            return
        name = self._trajs[sel[0]]
        self._traj_status.configure(text=f"Exec: {name}...", fg=FG)

        def done(res):
            color = OK_FG if res == "OK" else ERR_FG
            self._traj_status.configure(text=f"{name}: {res}", fg=color)
        self._async(lambda: self._node.call_execute_trajectory(name), done)

    # ── Periodic Update ──

    def _tick(self):
        s = self._node.get_state()

        mode = s["mode"]
        self._mode_label.configure(text=MODE_NAMES.get(mode, "?"),
                                   bg=MODE_COLORS.get(mode, "#888"))
        self._arm_label.configure(text=ARM_STATES.get(s["arm_state"], f"0x{s['arm_state']:02X}"))

        if s["joints"]:
            jstr = " ".join(f"{v:+7.3f}" for v in s["joints"][:6])
            self._joints_lbl.configure(text=f"Joints: {jstr}")

        if s["pose"]:
            p = s["pose"]
            r, pi, ya = rpy_from_quat(p.orientation.x, p.orientation.y,
                                       p.orientation.z, p.orientation.w)
            self._pose_lbl.configure(
                text=f"Pose:   ({p.position.x:+.3f} {p.position.y:+.3f} {p.position.z:+.3f})"
                     f"  rpy({r:+.2f} {pi:+.2f} {ya:+.2f})")

        new = self._node.get_new_logs(self._log_offset)
        if new:
            self._log_offset += len(new)
            self._log_text.configure(state="normal")
            for line, sev in new:
                tag = sev if sev in ("WRN", "ERR", "FTL", "INF") else None
                self._log_text.insert(tk.END, line + "\n", tag)
            self._log_text.see(tk.END)
            self._log_text.configure(state="disabled")

        self._root.after(100, self._tick)

    def run(self):
        self._root.mainloop()

    def shutdown(self):
        try:
            rclpy.shutdown()
        except Exception:
            pass


def main():
    panel = MissionPanel()
    try:
        panel.run()
    finally:
        panel.shutdown()


if __name__ == "__main__":
    main()
