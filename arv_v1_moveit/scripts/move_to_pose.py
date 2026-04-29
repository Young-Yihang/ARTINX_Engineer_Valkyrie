#!/usr/bin/env python3
"""
ARV_V1 Precision Motion Planning Tool — tkinter GUI + moveit_py.

Usage:
  # Launch MoveIt + RViz first, then:
  ros2 run arv_v1_moveit move_to_pose.py

  # CLI mode (no GUI):
  ros2 run arv_v1_moveit move_to_pose.py --joints 0 2.17 -0.94 -1.33 1.50 -1.68
  ros2 run arv_v1_moveit move_to_pose.py --pose 0.35 0.0 0.4 0.0 1.57 0.0
  ros2 run arv_v1_moveit move_to_pose.py --preset escape
"""

import argparse
import math
import os
import sys
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox, simpledialog

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from moveit_configs_utils import MoveItConfigsBuilder
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
JOINT_LIMITS = [
    (-1.2217, 1.2217), (0.49, 3.14), (-0.90, 0.70),
    (-2.975, 3.14), (-1.5708, 1.5708), (-3.14, 3.14),
]
PLANNING_GROUP = "ARM"
EE_LINK = "tcp"
REF_FRAME = "base_link"

POS_STEPS = [0.001, 0.005, 0.01]
ANG_STEPS = [0.01, 0.05, 0.1]

PLANNERS = {
    "LIN (Pilz)": ("pilz_industrial_motion_planner", "LIN"),
    "PTP (Pilz)": ("pilz_industrial_motion_planner", "PTP"),
    "RRTConnect": ("ompl", "RRTConnect"),
    "RRT*": ("ompl", "RRTstar"),
    "STOMP": ("stomp", ""),
    "CHOMP": ("chomp", ""),
}

PRESETS_PATH = os.path.expanduser(
    "~/ros2_ws/src/arv_v1_moveit/config/planning_presets.yaml"
)


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def rpy_from_quat(x, y, z, w):
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw


class StateListener(Node):
    def __init__(self):
        super().__init__("planning_tool_listener")
        self.joints = None
        self.js_lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

    def _js_cb(self, msg):
        if len(msg.position) >= 6:
            with self.js_lock:
                self.joints = list(msg.position[:6])

    def get_joints(self):
        with self.js_lock:
            return list(self.joints) if self.joints else None


class ArmPlanningTool:
    def __init__(self):
        self._last_plan_result = None
        self._busy = False

        rclpy.init()
        self._listener = StateListener()
        self._spin_thread = threading.Thread(
            target=lambda: rclpy.spin(self._listener), daemon=True
        )
        self._spin_thread.start()

        self._init_moveit()
        self._presets = self._load_presets()
        self._build_gui()
        self._schedule_display_update()

    def _init_moveit(self):
        config = MoveItConfigsBuilder(
            "arv_v1_model", package_name="arv_v1_moveit"
        ).to_moveit_configs()
        self._moveit = MoveItPy(
            node_name="moveit_planning_tool", config_dict=config.to_dict()
        )
        self._arm = self._moveit.get_planning_component(PLANNING_GROUP)
        self._model = self._moveit.get_robot_model()

    # ── GUI ────────────────────────────────────────────────

    def _build_gui(self):
        self._root = tk.Tk()
        self._root.title("ARV_V1 Motion Planning Tool")
        self._root.resizable(False, False)

        nb = ttk.Notebook(self._root)
        nb.pack(fill="x", padx=5, pady=(5, 0))

        cart_frame = ttk.Frame(nb)
        joint_frame = ttk.Frame(nb)
        nb.add(cart_frame, text=" Cartesian ")
        nb.add(joint_frame, text=" Joint Space ")
        self._notebook = nb

        self._cart_vars = {}
        for i, (label, steps) in enumerate(
            [("X", POS_STEPS), ("Y", POS_STEPS), ("Z", POS_STEPS),
             ("Roll", ANG_STEPS), ("Pitch", ANG_STEPS), ("Yaw", ANG_STEPS)]
        ):
            var, step_var = self._build_input_row(cart_frame, label, steps, i)
            self._cart_vars[label] = (var, step_var)

        self._joint_vars = []
        for i in range(6):
            var, step_var = self._build_input_row(
                joint_frame, f"J{i+1}", ANG_STEPS, i
            )
            self._joint_vars.append((var, step_var))

        ctrl = ttk.Frame(self._root)
        ctrl.pack(fill="x", padx=5, pady=5)

        ttk.Label(ctrl, text="Planner:").grid(row=0, column=0, sticky="w")
        self._planner_var = tk.StringVar(value="PTP (Pilz)")
        planner_cb = ttk.Combobox(
            ctrl, textvariable=self._planner_var,
            values=list(PLANNERS.keys()), state="readonly", width=14
        )
        planner_cb.grid(row=0, column=1, padx=(2, 10))

        ttk.Label(ctrl, text="Vel:").grid(row=0, column=2)
        self._vel_var = tk.DoubleVar(value=0.3)
        vel_scale = ttk.Scale(
            ctrl, from_=0.05, to=1.0, variable=self._vel_var,
            orient="horizontal", length=100
        )
        vel_scale.grid(row=0, column=3)
        self._vel_label = ttk.Label(ctrl, text="0.30", width=4)
        self._vel_label.grid(row=0, column=4)
        vel_scale.configure(command=lambda v: self._vel_label.configure(
            text=f"{float(v):.2f}"
        ))

        btn_frame = ttk.Frame(self._root)
        btn_frame.pack(fill="x", padx=5)

        ttk.Button(btn_frame, text="Read Current", command=self._on_read_current).pack(
            side="left", padx=2
        )
        ttk.Button(btn_frame, text="Plan", command=self._on_plan).pack(
            side="left", padx=2
        )
        ttk.Button(btn_frame, text="Execute", command=self._on_execute).pack(
            side="left", padx=2
        )
        ttk.Button(btn_frame, text="Plan+Exec", command=self._on_plan_execute).pack(
            side="left", padx=2
        )
        self._stop_btn = ttk.Button(
            btn_frame, text="STOP", command=self._on_stop
        )
        self._stop_btn.pack(side="left", padx=2)

        preset_frame = ttk.Frame(self._root)
        preset_frame.pack(fill="x", padx=5, pady=5)

        ttk.Label(preset_frame, text="Preset:").pack(side="left")
        self._preset_var = tk.StringVar()
        self._preset_cb = ttk.Combobox(
            preset_frame, textvariable=self._preset_var,
            values=list(self._presets.keys()), state="readonly", width=18
        )
        self._preset_cb.pack(side="left", padx=2)
        ttk.Button(preset_frame, text="Load", command=self._on_load_preset).pack(
            side="left", padx=2
        )
        ttk.Button(preset_frame, text="Save As...", command=self._on_save_preset).pack(
            side="left", padx=2
        )

        status_frame = ttk.LabelFrame(self._root, text="Status")
        status_frame.pack(fill="x", padx=5, pady=(0, 5))

        self._pose_label = ttk.Label(status_frame, text="Pose: waiting...", font=("monospace", 9))
        self._pose_label.pack(anchor="w", padx=3)
        self._joints_label = ttk.Label(status_frame, text="Joints: waiting...", font=("monospace", 9))
        self._joints_label.pack(anchor="w", padx=3)
        self._status_label = ttk.Label(status_frame, text="Ready", foreground="green")
        self._status_label.pack(anchor="w", padx=3, pady=(0, 3))

    def _build_input_row(self, parent, label, steps, row):
        ttk.Label(parent, text=label, width=5).grid(row=row, column=0, sticky="e", padx=2)
        var = tk.DoubleVar(value=0.0)
        entry = ttk.Entry(parent, textvariable=var, width=10, justify="right")
        entry.grid(row=row, column=1, padx=2, pady=1)

        step_var = tk.DoubleVar(value=steps[0])

        ttk.Button(
            parent, text="−", width=2,
            command=lambda: var.set(round(var.get() - step_var.get(), 6))
        ).grid(row=row, column=2)
        ttk.Button(
            parent, text="+", width=2,
            command=lambda: var.set(round(var.get() + step_var.get(), 6))
        ).grid(row=row, column=3)

        ttk.Label(parent, text="Step:").grid(row=row, column=4, padx=(6, 0))
        for i, s in enumerate(steps):
            rb = ttk.Radiobutton(
                parent, text=str(s), variable=step_var, value=s
            )
            rb.grid(row=row, column=5 + i)

        return var, step_var

    # ── Display Update ─────────────────────────────────────

    def _schedule_display_update(self):
        self._update_display()
        self._root.after(200, self._schedule_display_update)

    def _update_display(self):
        try:
            psm = self._moveit.get_planning_scene_monitor()
            with psm.read_only() as scene:
                state = scene.current_state
                joints = state.get_joint_group_positions(PLANNING_GROUP)
                pose = state.get_pose(EE_LINK)
        except Exception:
            joints = self._listener.get_joints()
            pose = None

        if joints is not None:
            jstr = ", ".join(f"{v:.3f}" for v in joints[:6])
            self._joints_label.configure(text=f"Joints: [{jstr}]")

        if pose is not None:
            p = pose.position
            r, pi, ya = rpy_from_quat(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w
            )
            self._pose_label.configure(
                text=f"Pose: xyz=({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) "
                     f"rpy=({r:.3f}, {pi:.3f}, {ya:.3f})"
            )

    def _set_status(self, text, color="black"):
        self._root.after(0, lambda: self._status_label.configure(
            text=text, foreground=color
        ))

    # ── Planning / Execution ───────────────────────────────

    def _get_active_tab(self):
        return "cartesian" if self._notebook.index("current") == 0 else "joint"

    def _set_goal_from_gui(self):
        self._arm.set_start_state_to_current_state()

        if self._get_active_tab() == "joint":
            jvals = {JOINT_NAMES[i]: self._joint_vars[i][0].get() for i in range(6)}
            state = RobotState(self._model)
            state.set_joint_group_positions(
                PLANNING_GROUP, np.array([jvals[n] for n in JOINT_NAMES])
            )
            state.update()
            self._arm.set_goal_state(robot_state=state)
        else:
            pose = PoseStamped()
            pose.header.frame_id = REF_FRAME
            pose.pose.position.x = self._cart_vars["X"][0].get()
            pose.pose.position.y = self._cart_vars["Y"][0].get()
            pose.pose.position.z = self._cart_vars["Z"][0].get()
            q = quat_from_rpy(
                self._cart_vars["Roll"][0].get(),
                self._cart_vars["Pitch"][0].get(),
                self._cart_vars["Yaw"][0].get(),
            )
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            self._arm.set_goal_state(pose_stamped_msg=pose, pose_link=EE_LINK)

    def _build_plan_params(self):
        label = self._planner_var.get()
        pipeline, planner_id = PLANNERS.get(label, ("ompl", "RRTConnect"))
        vel = self._vel_var.get()

        # Pilz LIN requires Cartesian goal — if joint tab active, auto-convert
        if planner_id == "LIN" and self._get_active_tab() == "joint":
            self._set_status("LIN needs Cartesian goal — computing FK...", "orange")
            state = RobotState(self._model)
            jvals = [self._joint_vars[i][0].get() for i in range(6)]
            state.set_joint_group_positions(PLANNING_GROUP, np.array(jvals))
            state.update()
            fk_pose = state.get_pose(EE_LINK)
            if fk_pose is None:
                self._set_status("FK failed — switch to PTP or Cartesian tab", "red")
                return None
            pose = PoseStamped()
            pose.header.frame_id = REF_FRAME
            pose.pose = fk_pose
            self._arm.set_start_state_to_current_state()
            self._arm.set_goal_state(pose_stamped_msg=pose, pose_link=EE_LINK)

        params = PlanRequestParameters(self._moveit, pipeline)
        params.planner_id = planner_id
        params.max_velocity_scaling_factor = vel
        params.max_acceleration_scaling_factor = vel
        params.planning_time = 5.0
        params.planning_attempts = 1 if pipeline.startswith("pilz") else 5
        return params

    def _do_plan(self):
        self._set_status("Planning...", "blue")
        try:
            self._set_goal_from_gui()
            params = self._build_plan_params()
            if params is None:
                return False
            result = self._arm.plan(plan_parameters=params)
            if result:
                self._last_plan_result = result
                traj = result.trajectory
                n_pts = len(traj.joint_trajectory.points) if hasattr(traj, 'joint_trajectory') else 0
                dur = traj.joint_trajectory.points[-1].time_from_start.sec + \
                      traj.joint_trajectory.points[-1].time_from_start.nanosec * 1e-9 \
                    if n_pts > 0 else 0
                self._set_status(
                    f"Plan OK — {n_pts} pts, {dur:.2f}s  (preview in RViz)", "green"
                )
                return True
            else:
                self._last_plan_result = None
                self._set_status("Planning FAILED", "red")
                return False
        except Exception as e:
            self._set_status(f"Plan error: {e}", "red")
            return False

    def _do_execute(self):
        if self._last_plan_result is None:
            self._set_status("No trajectory — Plan first", "red")
            return
        self._set_status("Executing...", "blue")
        try:
            self._moveit.execute(self._last_plan_result.trajectory, controllers=[])
            self._set_status("Execution complete", "green")
        except Exception as e:
            self._set_status(f"Execution error: {e}", "red")

    def _run_in_thread(self, fn):
        if self._busy:
            self._set_status("Busy...", "orange")
            return
        self._busy = True

        def wrapper():
            try:
                fn()
            finally:
                self._busy = False

        threading.Thread(target=wrapper, daemon=True).start()

    def _on_plan(self):
        self._run_in_thread(self._do_plan)

    def _on_execute(self):
        self._run_in_thread(self._do_execute)

    def _on_plan_execute(self):
        def plan_then_exec():
            if self._do_plan():
                self._do_execute()
        self._run_in_thread(plan_then_exec)

    def _on_stop(self):
        try:
            self._moveit.get_trajectory_execution_manager().stop_execution()
            self._set_status("STOPPED", "red")
        except Exception as e:
            self._set_status(f"Stop error: {e}", "red")

    # ── Read Current ───────────────────────────────────────

    def _on_read_current(self):
        try:
            psm = self._moveit.get_planning_scene_monitor()
            with psm.read_only() as scene:
                state = scene.current_state
                joints = list(state.get_joint_group_positions(PLANNING_GROUP))
                pose = state.get_pose(EE_LINK)
        except Exception:
            joints = self._listener.get_joints()
            pose = None

        if joints:
            for i in range(6):
                self._joint_vars[i][0].set(round(joints[i], 4))

        if pose is not None:
            self._cart_vars["X"][0].set(round(pose.position.x, 4))
            self._cart_vars["Y"][0].set(round(pose.position.y, 4))
            self._cart_vars["Z"][0].set(round(pose.position.z, 4))
            r, p, y = rpy_from_quat(
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w
            )
            self._cart_vars["Roll"][0].set(round(r, 4))
            self._cart_vars["Pitch"][0].set(round(p, 4))
            self._cart_vars["Yaw"][0].set(round(y, 4))
            self._set_status("Loaded current pose", "green")
        elif joints:
            self._set_status("Loaded joints (no FK pose)", "orange")
        else:
            self._set_status("No joint data available", "red")

    # ── Presets ────────────────────────────────────────────

    def _load_presets(self):
        defaults = {
            "escape": {"type": "joint", "joint_values": [0.0, 2.1746, -0.937, -1.326, 1.5028, -1.6796]},
            "start": {"type": "joint", "joint_values": [0.0, 2.6343, -1.0785, 0.0, 0.0, 0.0]},
        }
        if os.path.exists(PRESETS_PATH):
            try:
                with open(PRESETS_PATH) as f:
                    data = yaml.safe_load(f) or {}
                defaults.update(data.get("presets", {}))
            except Exception:
                pass
        return defaults

    def _save_presets(self):
        os.makedirs(os.path.dirname(PRESETS_PATH), exist_ok=True)
        with open(PRESETS_PATH, "w") as f:
            yaml.dump({"presets": self._presets}, f, default_flow_style=False)

    def _on_load_preset(self):
        name = self._preset_var.get()
        if not name or name not in self._presets:
            return
        p = self._presets[name]
        if p.get("type") == "joint" and "joint_values" in p:
            for i, v in enumerate(p["joint_values"][:6]):
                self._joint_vars[i][0].set(round(v, 4))
            self._notebook.select(1)
        elif p.get("type") == "cartesian":
            for k, v in zip(["X", "Y", "Z"], p.get("xyz", [0, 0, 0])):
                self._cart_vars[k][0].set(round(v, 4))
            for k, v in zip(["Roll", "Pitch", "Yaw"], p.get("rpy", [0, 0, 0])):
                self._cart_vars[k][0].set(round(v, 4))
            self._notebook.select(0)
        self._set_status(f"Loaded preset: {name}", "green")

    def _on_save_preset(self):
        name = simpledialog.askstring("Save Preset", "Preset name:", parent=self._root)
        if not name:
            return
        if self._get_active_tab() == "joint":
            self._presets[name] = {
                "type": "joint",
                "joint_values": [self._joint_vars[i][0].get() for i in range(6)],
            }
        else:
            self._presets[name] = {
                "type": "cartesian",
                "xyz": [self._cart_vars[k][0].get() for k in ["X", "Y", "Z"]],
                "rpy": [self._cart_vars[k][0].get() for k in ["Roll", "Pitch", "Yaw"]],
            }
        self._save_presets()
        self._preset_cb["values"] = list(self._presets.keys())
        self._preset_var.set(name)
        self._set_status(f"Saved preset: {name}", "green")

    # ── Lifecycle ──────────────────────────────────────────

    def run(self):
        self._root.mainloop()

    def shutdown(self):
        try:
            self._moveit.shutdown()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


def cli_plan_execute(moveit, arm, model, joints=None, pose_xyzrpy=None,
                     planner="PTP (Pilz)", vel=0.3):
    """CLI mode: plan and execute without GUI."""
    arm.set_start_state_to_current_state()

    if joints is not None:
        state = RobotState(model)
        state.set_joint_group_positions(PLANNING_GROUP, np.array(joints))
        state.update()
        arm.set_goal_state(robot_state=state)
        print(f"Goal (joint): {[f'{v:.4f}' for v in joints]}")
    elif pose_xyzrpy is not None:
        pose = PoseStamped()
        pose.header.frame_id = REF_FRAME
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = pose_xyzrpy[:3]
        q = quat_from_rpy(*pose_xyzrpy[3:])
        pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
        pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
        arm.set_goal_state(pose_stamped_msg=pose, pose_link=EE_LINK)
        print(f"Goal (Cartesian): xyz={pose_xyzrpy[:3]} rpy={pose_xyzrpy[3:]}")

    pipeline, planner_id = PLANNERS.get(planner, ("ompl", "RRTConnect"))
    params = PlanRequestParameters(moveit, pipeline)
    params.planner_id = planner_id
    params.max_velocity_scaling_factor = vel
    params.max_acceleration_scaling_factor = vel
    params.planning_time = 5.0
    params.planning_attempts = 1 if pipeline.startswith("pilz") else 5

    print(f"Planning with {planner}...")
    result = arm.plan(plan_parameters=params)
    if not result:
        print("Planning FAILED")
        return False

    print("Plan OK. Executing...")
    moveit.execute(result.trajectory, controllers=[])
    print("Done.")
    return True


def main():
    parser = argparse.ArgumentParser(description="ARV_V1 Precision Motion Planning Tool")
    parser.add_argument("--joints", nargs=6, type=float, metavar="J",
                        help="6 joint angles in rad")
    parser.add_argument("--pose", nargs=6, type=float, metavar="V",
                        help="Cartesian target: x y z roll pitch yaw")
    parser.add_argument("--preset", type=str, help="Load a named preset and execute")
    parser.add_argument("--planner", type=str, default="PTP (Pilz)",
                        help=f"Planner: {', '.join(PLANNERS.keys())}")
    parser.add_argument("--vel", type=float, default=0.3, help="Velocity scaling (0.05-1.0)")
    args = parser.parse_args()

    cli_mode = args.joints or args.pose or args.preset

    if cli_mode:
        rclpy.init()
        config = MoveItConfigsBuilder(
            "arv_v1_model", package_name="arv_v1_moveit"
        ).to_moveit_configs()
        moveit = MoveItPy(node_name="moveit_planning_tool_cli", config_dict=config.to_dict())
        arm = moveit.get_planning_component(PLANNING_GROUP)
        model = moveit.get_robot_model()
        time.sleep(1.0)  # wait for planning scene sync

        joints = args.joints
        pose = args.pose
        if args.preset:
            presets = ArmPlanningTool._load_presets(None)  # static-ish call
            # Load presets manually for CLI
            defaults = {
                "escape": {"type": "joint", "joint_values": [0.0, 2.1746, -0.937, -1.326, 1.5028, -1.6796]},
                "start": {"type": "joint", "joint_values": [0.0, 2.6343, -1.0785, 0.0, 0.0, 0.0]},
            }
            if os.path.exists(PRESETS_PATH):
                try:
                    with open(PRESETS_PATH) as f:
                        data = yaml.safe_load(f) or {}
                    defaults.update(data.get("presets", {}))
                except Exception:
                    pass
            p = defaults.get(args.preset)
            if not p:
                print(f"Unknown preset: {args.preset}. Available: {list(defaults.keys())}")
                return
            if p["type"] == "joint":
                joints = p["joint_values"]
            else:
                pose = p["xyz"] + p["rpy"]

        cli_plan_execute(moveit, arm, model, joints=joints, pose_xyzrpy=pose,
                         planner=args.planner, vel=args.vel)
        moveit.shutdown()
        rclpy.shutdown()
    else:
        tool = ArmPlanningTool()
        try:
            tool.run()
        finally:
            tool.shutdown()


if __name__ == "__main__":
    main()
