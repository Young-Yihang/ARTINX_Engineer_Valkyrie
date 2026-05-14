#!/usr/bin/env python3
"""
ARV_V1 Mission Panel — modern dashboard for the headless mission_executor.

Usage:
  ros2 run arv_v1_moveit mission_panel.py

Prerequisites:
  pip3 install customtkinter
"""

import math
import threading
import time
import tkinter as tk

import customtkinter as ctk
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rcl_interfaces.msg import Log
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt8, Int32
import datetime
from arv_v1_interfaces.srv import (
    ComposeTrajectory,
    GripperControl,
    ListTrajectories,
    LoadTrajectory,
    SaveLastTrajectory,
    SaveTrajectory,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ctk.set_appearance_mode("dark")

MODE_NAMES = {0: "RELAX", 1: "FREEDRIVE", 2: "ARMED"}
MODE_COLORS = {0: "#e06c75", 1: "#d19a66", 2: "#7ec699"}
ARM_STATES = {0x00: "READY", 0x01: "EXECUTING", 0x02: "HOLDING", 0x05: "RELAX", 0x06: "FREEDRIVE"}
JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]

FONT_MONO = "JetBrains Mono"

# Catppuccin Mocha palette
BASE = "#1e1e2e"
MANTLE = "#181825"
CRUST = "#11111b"
SURFACE0 = "#313244"
SURFACE1 = "#45475a"
TEXT = "#cdd6f4"
SUBTEXT = "#a6adc8"
OVERLAY = "#6c7086"
LAVENDER = "#b4befe"
MAUVE = "#cba6f7"
PINK = "#f5c2e7"
RED = "#f38ba8"
PEACH = "#fab387"
GREEN = "#a6da95"
TEAL = "#94e2d5"
BLUE = "#89b4fa"
YELLOW = "#f9e2af"
ROSEWATER = "#f5e0dc"

LOG_BG = CRUST
CARD_FG = MANTLE

# Button palette — high contrast: dark purple bg + white text
BTN = "#5b4a9e"
BTN_HOVER = "#7c6bbf"
BTN_ALT = "#3b3778"
BTN_ALT_HOVER = "#504b94"
BTN_DANGER = "#c53b53"
BTN_DANGER_HOVER = "#d95f73"
BTN_OK = "#3a7d5c"
BTN_OK_HOVER = "#4fa676"
BTN_TEXT = "#ffffff"


def rpy_from_quat(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


# ── ROS2 Node ─────────────────────────────────────────

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
        self.create_subscription(JointState, "/joint_states", self._js_cb, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, "/cartesian_controller/current_pose", self._pose_cb, 10)
        self.create_subscription(UInt8, "/control_mode", self._mode_cb, 10)
        self.create_subscription(UInt8, "/arm_state", self._arm_state_cb, 10)
        self.create_subscription(Log, "/rosout", self._log_cb, best_effort)
        self.mode_pub = self.create_publisher(UInt8, "/control_mode", 10)
        self.task_pub = self.create_publisher(Int32, "/task_command", 10)
        self.gripper_cli = self.create_client(GripperControl, "/gripper_control")
        self.list_cli = self.create_client(ListTrajectories, "/list_trajectories")
        self.load_cli = self.create_client(LoadTrajectory, "/load_trajectory")
        self.save_last_cli = self.create_client(SaveLastTrajectory, "/save_last_trajectory")
        self.save_cli = self.create_client(SaveTrajectory, "/save_trajectory")
        self.compose_cli = self.create_client(ComposeTrajectory, "/compose_trajectory")

        # 示教录制 (50Hz 降采样)
        self.teach_recording = False
        self.teach_buffer = []
        self.teach_gripper_events = []  # [(t_offset, "open"/"close"/"stop")]
        self.teach_start_wall = 0.0
        self.teach_last_t = 0.0
        self.TEACH_INTERVAL_S = 0.02

    def _js_cb(self, msg):
        with self._lock:
            self.joints = list(msg.position[:7]) if len(msg.position) >= 6 else None
            # 示教录制: 50Hz 降采样 buffer 当前 6 关节角
            if self.teach_recording and len(msg.position) >= 6:
                now = time.time()
                if now - self.teach_last_t >= self.TEACH_INTERVAL_S:
                    self.teach_buffer.append((now - self.teach_start_wall, list(msg.position[:6])))
                    self.teach_last_t = now
    def _pose_cb(self, msg):
        with self._lock: self.pose = msg.pose
    def _mode_cb(self, msg):
        with self._lock: self.control_mode = msg.data
    def _arm_state_cb(self, msg):
        with self._lock: self.arm_state = msg.data
    def _log_cb(self, msg):
        sev = {10: "DBG", 20: "INF", 30: "WRN", 40: "ERR", 50: "FTL"}.get(msg.level, "???")
        name = msg.name.split(".")[-1][:16] if msg.name else "?"
        line = f"[{sev}] {name}: {msg.msg}"
        with self._log_lock:
            self.log_lines.append((line, sev))
            if len(self.log_lines) > 1000:
                self.log_lines = self.log_lines[-500:]

    def pub_mode(self, mode):
        m = UInt8(); m.data = mode; self.mode_pub.publish(m)
    def pub_task(self, cmd, param=0):
        self._task_seq = (self._task_seq + 1) % 256
        m = Int32(); m.data = (cmd << 16) | (param << 8) | self._task_seq
        self.task_pub.publish(m)
    def get_state(self):
        with self._lock:
            return dict(joints=list(self.joints) if self.joints else None,
                        pose=self.pose, mode=self.control_mode, arm_state=self.arm_state)
    def get_new_logs(self):
        with self._log_lock:
            res = self.log_lines
            self.log_lines = []
            return res

    @staticmethod
    def _wait_future(fut, timeout=3.0):
        t0 = time.monotonic()
        while not fut.done() and (time.monotonic() - t0) < timeout:
            time.sleep(0.05)
        return fut.result() if fut.done() else None

    def call_gripper(self, force):
        if not self.gripper_cli.wait_for_service(timeout_sec=0.5): return "service not ready"
        # 录制中: buffer gripper 事件 (跟 LoadTrajectory.srv 字段对齐)
        if self.teach_recording:
            cmd = "close" if force > 0.5 else ("open" if force < -0.5 else "stop")
            t = time.time() - self.teach_start_wall
            with self._lock:
                self.teach_gripper_events.append((t, cmd))
        req = GripperControl.Request(); req.torque = float(force)
        res = self._wait_future(self.gripper_cli.call_async(req), 3.0)
        return ("OK" if res.success else res.message) if res else "timeout"

    def call_list_trajectories(self):
        if not self.list_cli.wait_for_service(timeout_sec=0.5): return []
        res = self._wait_future(self.list_cli.call_async(ListTrajectories.Request()), 3.0)
        return list(res.names) if res else []

    def call_execute_trajectory(self, name):
        if not self.load_cli.wait_for_service(timeout_sec=0.5): return "service not ready"
        req = LoadTrajectory.Request(); req.name = name; req.execute = True
        res = self._wait_future(self.load_cli.call_async(req), 120.0)
        return ("OK" if res.success else res.message) if res else "timeout"

    def call_save_last_trajectory(self, name, description=""):
        if not self.save_last_cli.wait_for_service(timeout_sec=0.5): return "service not ready"
        req = SaveLastTrajectory.Request(); req.name = name; req.description = description
        res = self._wait_future(self.save_last_cli.call_async(req), 5.0)
        return ("OK: " + res.saved_path if res.success else res.message) if res else "timeout"

    def call_compose_trajectory(self, name, waypoints_q, speed=0.3, description="",
                                 gripper_actions=None):
        """waypoints_q: List[List[float]] (6 joints each). gripper_actions: List[(t, cmd)]."""
        if not self.compose_cli.wait_for_service(timeout_sec=0.5):
            return "service not ready (move_group running?)"
        req = ComposeTrajectory.Request()
        req.name = name
        req.description = description
        req.speed_factor = float(speed)
        for q in waypoints_q:
            js = JointState()
            js.name = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
            js.position = list(q)
            req.waypoints.append(js)
        if gripper_actions:
            req.gripper_action_times = [float(t) for t, _ in gripper_actions]
            req.gripper_action_commands = [c for _, c in gripper_actions]
        res = self._wait_future(self.compose_cli.call_async(req), 30.0)
        if not res:
            return "compose timeout"
        return (f"OK: {res.saved_path} ({res.point_count} pts, {res.duration:.2f}s)"
                if res.success else res.message)

    def teach_start(self):
        with self._lock:
            self.teach_buffer = []
            self.teach_gripper_events = []
            self.teach_start_wall = time.time()
            self.teach_last_t = 0.0
            self.teach_recording = True

    def teach_stop_and_save(self):
        """停录 → 自动命名 teach_MMDD_HHMM 并保存 (含 gripper actions)"""
        with self._lock:
            self.teach_recording = False
            buf = list(self.teach_buffer)
            g_events = list(self.teach_gripper_events)

        if len(buf) < 2:
            return "no points captured"

        now = datetime.datetime.now()
        name = f"teach_{now.strftime('%m%d_%H%M')}"

        traj = JointTrajectory()
        traj.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        for t, q in buf:
            pt = JointTrajectoryPoint()
            pt.positions = q
            pt.time_from_start.sec = int(t)
            pt.time_from_start.nanosec = int((t - int(t)) * 1e9)
            traj.points.append(pt)

        if not self.save_cli.wait_for_service(timeout_sec=2.0):
            return "save_trajectory service unavailable"
        req = SaveTrajectory.Request()
        req.name = name
        req.description = f"teach @ {now.isoformat(timespec='seconds')}, {len(buf)} pts, {len(g_events)} grip"
        req.trajectory = traj
        req.gripper_action_times = [t for t, _ in g_events]
        req.gripper_action_commands = [c for _, c in g_events]
        res = self._wait_future(self.save_cli.call_async(req), 5.0)
        if not res:
            return "save timeout"
        return (f"OK: {name} ({len(buf)} pts, {len(g_events)} grip)"
                if res.success else res.message)


# ── GUI ────────────────────────────────────────────────

class MissionPanel:
    def __init__(self):
        rclpy.init()
        self._node = PanelNode()
        threading.Thread(target=lambda: rclpy.spin(self._node), daemon=True).start()
        self._trajs = []
        self._build_gui()
        self._root.after(500, self._refresh_trajs)
        self._tick()

    def _card(self, parent, **grid_kw):
        f = ctk.CTkFrame(parent, corner_radius=12, fg_color=CARD_FG)
        f.grid(**grid_kw)
        return f

    def _build_gui(self):
        self._root = ctk.CTk()
        self._root.title("ARV_V1 Mission Panel")
        # 自适应屏幕分辨率: 75% 宽 × 80% 高, 但 clamp 到合理范围
        sw = self._root.winfo_screenwidth()
        sh = self._root.winfo_screenheight()
        w = max(800, min(int(sw * 0.75), 1800))
        h = max(600, min(int(sh * 0.80), 1100))
        x = (sw - w) // 2
        y = (sh - h) // 2
        self._root.geometry(f"{w}x{h}+{x}+{y}")
        self._root.minsize(800, 600)
        self._root.configure(fg_color=BASE)

        # DPI scaling
        sc = self._root._get_window_scaling()
        self._sc = sc

        # Responsive grid: 3 columns, 3 rows
        self._root.grid_columnconfigure(0, weight=1, minsize=220)
        self._root.grid_columnconfigure(1, weight=1, minsize=220)
        self._root.grid_columnconfigure(2, weight=2, minsize=340)
        self._root.grid_rowconfigure(0, weight=0)
        self._root.grid_rowconfigure(1, weight=3)
        self._root.grid_rowconfigure(2, weight=2)

        pad = {"padx": 6, "pady": 6, "sticky": "nsew"}

        # ═══════ ROW 0: Mode + Pose overview ═══════

        # Mode card (big indicator + buttons)
        mc = self._card(self._root, row=0, column=0, **pad)
        self._mode_pill = ctk.CTkLabel(mc, text="RELAX", corner_radius=8,
                                       fg_color=RED, text_color=CRUST,
                                       font=ctk.CTkFont(family=FONT_MONO, size=22, weight="bold"),
                                       height=50)
        self._mode_pill.pack(fill="x", padx=10, pady=(10, 6))
        self._arm_pill = ctk.CTkLabel(mc, text="READY", fg_color=SURFACE0, corner_radius=6,
                                      text_color=TEXT,
                                      font=ctk.CTkFont(family=FONT_MONO, size=14), height=30)
        self._arm_pill.pack(fill="x", padx=10, pady=(0, 6))
        mr = ctk.CTkFrame(mc, fg_color="transparent")
        mr.pack(fill="x", padx=8, pady=(0, 10))
        for val, name in MODE_NAMES.items():
            ctk.CTkButton(mr, text=name, fg_color=BTN, hover_color=BTN_HOVER,
                          text_color=BTN_TEXT,
                          height=36, font=ctk.CTkFont(size=13, weight="bold"),
                          command=lambda m=val: self._node.pub_mode(m)
                          ).pack(side="left", padx=2, expand=True, fill="x")

        # Pose card (xyz + rpy as labeled values)
        pc = self._card(self._root, row=0, column=1, columnspan=2, **pad)
        ctk.CTkLabel(pc, text="END EFFECTOR", font=ctk.CTkFont(size=14, weight="bold"),
                     text_color=OVERLAY).pack(anchor="w", padx=12, pady=(10, 4))

        pose_grid = ctk.CTkFrame(pc, fg_color="transparent")
        pose_grid.pack(fill="x", padx=12, pady=(0, 6))
        self._pose_vals = {}
        for col, (lbl, unit) in enumerate([("X", "m"), ("Y", "m"), ("Z", "m"),
                                            ("Roll", "rad"), ("Pitch", "rad"), ("Yaw", "rad")]):
            f = ctk.CTkFrame(pose_grid, fg_color="transparent")
            f.pack(side="left", expand=True, fill="x", padx=2)
            ctk.CTkLabel(f, text=lbl, font=ctk.CTkFont(size=13), text_color=OVERLAY).pack()
            v = ctk.CTkLabel(f, text="--", text_color=TEXT,
                           font=ctk.CTkFont(family=FONT_MONO, size=20, weight="bold"))
            v.pack()
            ctk.CTkLabel(f, text=unit, font=ctk.CTkFont(size=10), text_color=SURFACE1).pack()
            self._pose_vals[lbl] = v

        # Joint bars row
        jf = ctk.CTkFrame(pc, fg_color="transparent")
        jf.pack(fill="x", padx=12, pady=(2, 10))
        self._joint_bars = []
        for i in range(6):
            col_f = ctk.CTkFrame(jf, fg_color="transparent")
            col_f.pack(side="left", expand=True, fill="x", padx=1)
            ctk.CTkLabel(col_f, text=JOINT_NAMES[i], font=ctk.CTkFont(size=12),
                         text_color=OVERLAY).pack()
            bar = ctk.CTkProgressBar(col_f, height=12, corner_radius=5,
                                     progress_color=MAUVE, fg_color=SURFACE0)
            bar.set(0.5)
            bar.pack(fill="x", padx=2)
            val_lbl = ctk.CTkLabel(col_f, text="0.00", text_color=TEXT,
                                  font=ctk.CTkFont(family=FONT_MONO, size=13))
            val_lbl.pack()
            self._joint_bars.append((bar, val_lbl))

        # ═══════ ROW 1: Gripper + Commands + Trajectories ═══════

        # Gripper
        gc = self._card(self._root, row=1, column=0, **pad)
        ctk.CTkLabel(gc, text="GRIPPER", font=ctk.CTkFont(size=14, weight="bold"),
                     text_color=OVERLAY).pack(anchor="w", padx=12, pady=(10, 6))
        for txt, force in [("Grip  (70 N)", 70.0), ("Release (-70 N)", -70.0), ("Stop  (0 N)", 0.0)]:
            ctk.CTkButton(gc, text=txt, fg_color=BTN_ALT, hover_color=BTN_ALT_HOVER,
                          text_color=BTN_TEXT, height=40, font=ctk.CTkFont(size=14),
                          command=lambda f=force: self._async(lambda: self._node.call_gripper(f))
                          ).pack(fill="x", padx=12, pady=3)
        ctk.CTkFrame(gc, height=4, fg_color="transparent").pack()

        # Commands
        cc = self._card(self._root, row=1, column=1, **pad)
        ctk.CTkLabel(cc, text="COMMANDS", font=ctk.CTkFont(size=14, weight="bold"),
                     text_color=OVERLAY).pack(anchor="w", padx=12, pady=(10, 6))
        ctk.CTkButton(cc, text="E-STOP", fg_color=BTN_DANGER, hover_color=BTN_DANGER_HOVER,
                      text_color=BTN_TEXT,
                      height=56, font=ctk.CTkFont(size=20, weight="bold"),
                      command=lambda: self._node.pub_task(0x01)).pack(fill="x", padx=12, pady=(0, 6))
        cr = ctk.CTkFrame(cc, fg_color="transparent")
        cr.pack(fill="x", padx=10, pady=(0, 6))
        for txt, cmd in [("Reset", 0x02), ("Next", 0x30), ("Abort", 0x31)]:
            ctk.CTkButton(cr, text=txt, fg_color=BTN, hover_color=BTN_HOVER,
                          text_color=BTN_TEXT, height=38, font=ctk.CTkFont(size=14),
                          command=lambda c=cmd: self._node.pub_task(c)
                          ).pack(side="left", padx=2, expand=True, fill="x")

        # Trajectories — use internal grid so buttons never get clipped
        tc = self._card(self._root, row=1, column=2, **pad)
        tc.grid_rowconfigure(1, weight=1)
        tc.grid_columnconfigure(0, weight=1)

        th = ctk.CTkFrame(tc, fg_color="transparent")
        th.grid(row=0, column=0, sticky="ew", padx=12, pady=(10, 4))
        ctk.CTkLabel(th, text="TRAJECTORIES", font=ctk.CTkFont(size=14, weight="bold"),
                     text_color=OVERLAY).pack(side="left")
        self._traj_status = ctk.CTkLabel(th, text="", font=ctk.CTkFont(size=13))
        self._traj_status.pack(side="right")

        self._traj_list = tk.Listbox(tc, font=(FONT_MONO, 14),
                                     bg=CRUST, fg=TEXT, selectbackground=MAUVE,
                                     selectforeground=CRUST, borderwidth=0,
                                     highlightthickness=0, relief="flat", activestyle="none")
        self._traj_list.grid(row=1, column=0, sticky="nsew", padx=12, pady=(0, 4))

        tb = ctk.CTkFrame(tc, fg_color="transparent")
        tb.grid(row=2, column=0, sticky="ew", padx=12, pady=(4, 10))
        ctk.CTkButton(tb, text="Refresh", width=80, height=36,
                      fg_color=BTN_ALT, hover_color=BTN_ALT_HOVER, text_color=BTN_TEXT,
                      font=ctk.CTkFont(size=13),
                      command=self._refresh_trajs).pack(side="left", padx=(0, 6))
        ctk.CTkButton(tb, text="Execute", width=80, height=36,
                      font=ctk.CTkFont(size=13, weight="bold"),
                      fg_color=BTN_OK, hover_color=BTN_OK_HOVER, text_color=BTN_TEXT,
                      command=self._exec_traj).pack(side="left", padx=(0, 6))
        ctk.CTkButton(tb, text="Save Last", width=90, height=36,
                      font=ctk.CTkFont(size=13),
                      fg_color=BTN_ALT, hover_color=BTN_ALT_HOVER, text_color=BTN_TEXT,
                      command=self._save_last_traj).pack(side="left")
        self._teach_btn = ctk.CTkButton(tb, text="Teach", width=80, height=36,
                                        font=ctk.CTkFont(size=13),
                                        fg_color=BTN_ALT, hover_color=BTN_ALT_HOVER,
                                        text_color=BTN_TEXT,
                                        command=self._toggle_teach)
        self._teach_btn.pack(side="left", padx=(6, 0))
        ctk.CTkButton(tb, text="Compose", width=90, height=36,
                      font=ctk.CTkFont(size=13),
                      fg_color=BTN_ALT, hover_color=BTN_ALT_HOVER, text_color=BTN_TEXT,
                      command=self._open_compose).pack(side="left", padx=(6, 0))

        # ═══════ ROW 2: Log (full width) ═══════

        log_card = self._card(self._root, row=2, column=0, columnspan=3, **pad)
        ctk.CTkLabel(log_card, text="LOG", font=ctk.CTkFont(size=14, weight="bold"),
                     text_color=OVERLAY).pack(anchor="w", padx=12, pady=(8, 2))

        log_wrap = ctk.CTkFrame(log_card, fg_color="transparent")
        log_wrap.pack(fill="both", expand=True, padx=10, pady=(0, 10))
        log_wrap.grid_rowconfigure(0, weight=1)
        log_wrap.grid_columnconfigure(0, weight=1)

        self._log_text = tk.Text(log_wrap, font=(FONT_MONO, 13), state="disabled",
                                 wrap="none", bg=LOG_BG, fg=OVERLAY,
                                 insertbackground=TEXT, borderwidth=0, highlightthickness=0)
        ls = ctk.CTkScrollbar(log_wrap, command=self._log_text.yview)
        self._log_text.configure(yscrollcommand=ls.set)
        self._log_text.grid(row=0, column=0, sticky="nsew")
        ls.grid(row=0, column=1, sticky="ns")

        self._log_text.tag_configure("WRN", foreground=YELLOW)
        self._log_text.tag_configure("ERR", foreground=RED)
        self._log_text.tag_configure("FTL", foreground=PINK, underline=True)
        self._log_text.tag_configure("INF", foreground=OVERLAY)

    # ── Helpers ──

    def _async(self, fn, callback=None):
        def wrapper():
            res = fn()
            if callback: self._root.after(0, lambda: callback(res))
        threading.Thread(target=wrapper, daemon=True).start()

    def _refresh_trajs(self):
        def done(names):
            self._trajs = names
            self._traj_list.delete(0, tk.END)
            for n in names: self._traj_list.insert(tk.END, f"  {n}")
            self._traj_status.configure(text=f"{len(names)} saved", text_color=OVERLAY)
        self._async(self._node.call_list_trajectories, done)

    def _exec_traj(self):
        sel = self._traj_list.curselection()
        if not sel:
            self._traj_status.configure(text="select first", text_color=YELLOW)
            return
        name = self._trajs[sel[0]]
        self._traj_status.configure(text=f"executing...", text_color=LAVENDER)
        def done(res):
            color = GREEN if res == "OK" else RED
            self._traj_status.configure(text=f"{name}: {res}", text_color=color)
        self._async(lambda: self._node.call_execute_trajectory(name), done)

    def _toggle_teach(self):
        if not self._node.teach_recording:
            # 开始录制: 切按钮红色
            self._node.teach_start()
            self._teach_btn.configure(text="Stop", fg_color=BTN_DANGER, hover_color=BTN_DANGER_HOVER)
            self._traj_status.configure(text="recording...", text_color=YELLOW)
        else:
            # 停录 + 保存
            self._teach_btn.configure(text="Teach", fg_color=BTN_ALT, hover_color=BTN_ALT_HOVER)
            self._traj_status.configure(text="saving...", text_color=LAVENDER)
            def done(res):
                color = GREEN if str(res).startswith("OK") else RED
                self._traj_status.configure(text=res, text_color=color)
                if str(res).startswith("OK"):
                    self._refresh_trajs()
            self._async(self._node.teach_stop_and_save, done)

    def _open_compose(self):
        """Compose dialog: capture 2+ waypoints from current joint state, plan & save via MoveIt."""
        win = ctk.CTkToplevel(self._root)
        win.title("Compose Trajectory")
        win.geometry("520x460")
        win.configure(fg_color=BASE)
        win.transient(self._root)

        captured = []  # List[List[float]]

        ctk.CTkLabel(win, text="COMPOSE TRAJECTORY",
                     font=ctk.CTkFont(size=15, weight="bold"),
                     text_color=OVERLAY).pack(anchor="w", padx=14, pady=(12, 6))
        ctk.CTkLabel(win, text="拖臂(FREEDRIVE)到目标点 → Capture, 至少 2 个",
                     font=ctk.CTkFont(size=11), text_color=SUBTEXT).pack(anchor="w", padx=14)

        # Waypoint list (scrollable)
        wp_card = ctk.CTkFrame(win, fg_color=CARD_FG, corner_radius=8)
        wp_card.pack(fill="both", expand=True, padx=14, pady=8)
        wp_list = tk.Listbox(wp_card, font=(FONT_MONO, 11), bg=CRUST, fg=TEXT,
                             selectbackground=MAUVE, selectforeground=CRUST,
                             borderwidth=0, highlightthickness=0, height=6)
        wp_list.pack(fill="both", expand=True, padx=8, pady=8)

        def refresh_list():
            wp_list.delete(0, tk.END)
            for i, q in enumerate(captured):
                wp_list.insert(tk.END,
                               f" {i}: " + ", ".join(f"{v:+.3f}" for v in q))

        def do_capture():
            s = self._node.get_state()
            if not s["joints"]:
                status.configure(text="no joint state yet", text_color=YELLOW); return
            q = list(s["joints"][:6])
            captured.append(q)
            refresh_list()
            status.configure(text=f"captured #{len(captured) - 1}", text_color=GREEN)

        def do_remove():
            sel = wp_list.curselection()
            if not sel: return
            captured.pop(sel[0])
            refresh_list()

        # Capture / remove buttons
        br = ctk.CTkFrame(win, fg_color="transparent")
        br.pack(fill="x", padx=14, pady=(0, 6))
        ctk.CTkButton(br, text="Capture", width=100, height=32,
                      fg_color=BTN, hover_color=BTN_HOVER, text_color=BTN_TEXT,
                      command=do_capture).pack(side="left")
        ctk.CTkButton(br, text="Remove selected", width=140, height=32,
                      fg_color=BTN_ALT, hover_color=BTN_ALT_HOVER, text_color=BTN_TEXT,
                      command=do_remove).pack(side="left", padx=8)

        # Name + speed + gripper hints
        form = ctk.CTkFrame(win, fg_color="transparent")
        form.pack(fill="x", padx=14, pady=(4, 4))
        ctk.CTkLabel(form, text="Name:", text_color=TEXT,
                     font=ctk.CTkFont(size=12)).grid(row=0, column=0, sticky="e", padx=4, pady=4)
        name_var = tk.StringVar()
        ctk.CTkEntry(form, textvariable=name_var, width=200).grid(row=0, column=1, sticky="w",
                                                                   padx=4, pady=4)
        ctk.CTkLabel(form, text="Speed:", text_color=TEXT,
                     font=ctk.CTkFont(size=12)).grid(row=0, column=2, sticky="e", padx=4, pady=4)
        speed_var = tk.DoubleVar(value=0.3)
        ctk.CTkEntry(form, textvariable=speed_var, width=60).grid(row=0, column=3, sticky="w",
                                                                    padx=4, pady=4)

        status = ctk.CTkLabel(win, text="", font=ctk.CTkFont(size=11), text_color=OVERLAY)
        status.pack(anchor="w", padx=14)

        def do_plan_save():
            if len(captured) < 2:
                status.configure(text="need ≥ 2 waypoints", text_color=YELLOW); return
            name = name_var.get().strip()
            if not name:
                status.configure(text="name required", text_color=YELLOW); return
            try:
                speed = float(speed_var.get())
            except Exception:
                status.configure(text="invalid speed", text_color=YELLOW); return

            # v1: 不带 gripper 动作 — Compose 出 YAML 后可手动编辑 meta.gripper_actions 添加
            # 后续: srv 加 waypoint-index 基础的 gripper trigger, server 推断实际时间
            status.configure(text="planning...", text_color=LAVENDER)
            btn_save.configure(state="disabled")

            def done(res):
                btn_save.configure(state="normal")
                color = GREEN if str(res).startswith("OK") else RED
                status.configure(text=str(res), text_color=color)
                if str(res).startswith("OK"):
                    self._refresh_trajs()
                    win.after(1500, win.destroy)

            self._async(lambda: self._node.call_compose_trajectory(
                name, captured, speed=speed), done)

        # Actions
        ar = ctk.CTkFrame(win, fg_color="transparent")
        ar.pack(fill="x", padx=14, pady=(4, 14))
        ctk.CTkButton(ar, text="Cancel", width=100, height=36,
                      fg_color=BTN_ALT, hover_color=BTN_ALT_HOVER, text_color=BTN_TEXT,
                      command=win.destroy).pack(side="right")
        btn_save = ctk.CTkButton(ar, text="Plan & Save", width=140, height=36,
                                 font=ctk.CTkFont(size=13, weight="bold"),
                                 fg_color=BTN_OK, hover_color=BTN_OK_HOVER, text_color=BTN_TEXT,
                                 command=do_plan_save)
        btn_save.pack(side="right", padx=8)

    def _save_last_traj(self):
        dialog = ctk.CTkInputDialog(text="Trajectory name:", title="Save Last Trajectory")
        name = dialog.get_input()
        if not name or not name.strip():
            return
        name = name.strip()
        self._traj_status.configure(text="saving...", text_color=LAVENDER)
        def done(res):
            color = GREEN if res.startswith("OK") else RED
            self._traj_status.configure(text=res, text_color=color)
            if res.startswith("OK"):
                self._refresh_trajs()
        self._async(lambda: self._node.call_save_last_trajectory(name), done)

    # ── Periodic Update ──

    JOINT_LIMITS = [(-1.22, 1.22), (0.49, 3.14), (-0.90, 0.70),
                    (-2.975, 3.14), (-1.57, 1.57), (-3.14, 3.14)]

    def _tick(self):
        s = self._node.get_state()

        mode = s["mode"]
        self._mode_pill.configure(text=MODE_NAMES.get(mode, "?"),
                                  fg_color=MODE_COLORS.get(mode, SURFACE0))
        self._arm_pill.configure(text=ARM_STATES.get(s["arm_state"], f"0x{s['arm_state']:02X}"))

        if s["joints"]:
            for i in range(min(6, len(s["joints"]))):
                lo, hi = self.JOINT_LIMITS[i]
                frac = (s["joints"][i] - lo) / (hi - lo) if hi != lo else 0.5
                frac = max(0.0, min(1.0, frac))
                bar, lbl = self._joint_bars[i]
                bar.set(frac)
                lbl.configure(text=f"{s['joints'][i]:+.2f}")

        if s["pose"]:
            p = s["pose"]
            r, pi, ya = rpy_from_quat(p.orientation.x, p.orientation.y,
                                       p.orientation.z, p.orientation.w)
            for key, val in [("X", p.position.x), ("Y", p.position.y), ("Z", p.position.z),
                             ("Roll", r), ("Pitch", pi), ("Yaw", ya)]:
                self._pose_vals[key].configure(text=f"{val:+.3f}")

        new = self._node.get_new_logs()
        if new:
            self._log_text.configure(state="normal")
            for line, sev in new:
                tag = sev if sev in ("WRN", "ERR", "FTL", "INF") else None
                self._log_text.insert(tk.END, line + "\n", tag)
            
            # Limit the Tkinter text widget length to prevent memory leak
            idx_end = self._log_text.index("end-1c")
            line_count = int(float(idx_end))
            if line_count > 500:
                self._log_text.delete("1.0", f"{line_count - 500 + 1}.0")
                
            self._log_text.see(tk.END)
            self._log_text.configure(state="disabled")

        self._root.after(100, self._tick)

    def run(self):
        self._root.mainloop()

    def shutdown(self):
        try: rclpy.shutdown()
        except Exception: pass


def main():
    panel = MissionPanel()
    try: panel.run()
    finally: panel.shutdown()

if __name__ == "__main__":
    main()
