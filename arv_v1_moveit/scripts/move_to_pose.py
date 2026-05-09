#!/usr/bin/env python3
"""
ARV_V1 Precision Motion Planning Tool — customtkinter + moveit_py.

Usage:
  ros2 run arv_v1_moveit move_to_pose.py
  ros2 run arv_v1_moveit move_to_pose.py --joints 0 2.17 -0.94 -1.33 1.50 -1.68
  ros2 run arv_v1_moveit move_to_pose.py --pose 0.35 0.0 0.4 0.0 1.57 0.0
  ros2 run arv_v1_moveit move_to_pose.py --preset escape

Prerequisites:
  pip3 install customtkinter
"""

import argparse
import math
import os
import threading
import time
import tkinter as tk

import customtkinter as ctk
import numpy as np
import vtk
from vtk.tk.vtkTkRenderWindowInteractor import vtkTkRenderWindowInteractor
import yaml

ctk.set_appearance_mode("dark")

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from moveit_configs_utils import MoveItConfigsBuilder
from moveit.planning import MoveItPy, PlanRequestParameters
from moveit.core.robot_state import RobotState

JOINT_NAMES = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
PLANNING_GROUP = "ARM"
EE_LINK = "tcp"
REF_FRAME = "base_link"

POS_STEPS = [0.001, 0.005, 0.01]
ANG_STEPS = [0.01, 0.05, 0.1]

PLANNERS = {
    "Direct LIN": ("direct", ""),
    "LIN (Pilz)": ("pilz_industrial_motion_planner", "LIN"),
    "PTP (Pilz)": ("pilz_industrial_motion_planner", "PTP"),
    "RRTConnect": ("ompl", "RRTConnect"),
    "RRT*": ("ompl", "RRTstar"),
    "STOMP": ("stomp", ""),
    "CHOMP": ("chomp", ""),
}

MESH_LINKS = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6",
              "link_gripper1", "link_gripper2"]
MESH_COLORS_RGB = [
    (0.58, 0.60, 0.70),  # base_link — overlay2
    (0.54, 0.71, 0.98),  # link1 — blue
    (0.45, 0.78, 0.93),  # link2 — sapphire
    (0.58, 0.89, 0.83),  # link3 — teal
    (0.65, 0.89, 0.63),  # link4 — green
    (0.98, 0.89, 0.69),  # link5 — yellow
    (0.98, 0.70, 0.53),  # link6 — peach
    (0.80, 0.80, 0.85),  # gripper1 — light gray
    (0.80, 0.80, 0.85),  # gripper2 — light gray
]
GHOST_COLOR_RGB = (0.65, 0.89, 0.63)
VISUAL_ORIGINS = {
    "link2": (-0.03215, -0.131, 0.0),
}
FK_CHAIN = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6",
            "link_gripper1", "link_gripper2", "tcp"]
MESHES_DIR = os.path.expanduser("~/ros2_ws/src/arv_v1_model/meshes")

PRESETS_PATH = os.path.expanduser(
    "~/ros2_ws/src/arv_v1_moveit/config/planning_presets.yaml"
)

# Catppuccin Mocha
BASE = "#1e1e2e"
MANTLE = "#181825"
CRUST = "#11111b"
SURFACE0 = "#313244"
SURFACE1 = "#45475a"
TEXT = "#cdd6f4"
SUBTEXT = "#a6adc8"
OVERLAY = "#6c7086"
MAUVE = "#cba6f7"
LAVENDER = "#b4befe"
RED = "#f38ba8"
GREEN = "#a6da95"
YELLOW = "#f9e2af"
BLUE = "#89b4fa"

BTN = "#5b4a9e"
BTN_HOVER = "#7c6bbf"
BTN_DANGER = "#c53b53"
BTN_DANGER_HOVER = "#d95f73"
BTN_OK = "#3a7d5c"
BTN_OK_HOVER = "#4fa676"


def quat_from_rpy(r, p, y):
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [sr*cp*cy - cr*sp*sy, cr*sp*cy + sr*cp*sy,
            cr*cp*sy - sr*sp*cy, cr*cp*cy + sr*sp*sy]


def rpy_from_quat(x, y, z, w):
    roll = math.atan2(2.0*(w*x + y*z), 1.0 - 2.0*(x*x + y*y))
    sinp = 2.0*(w*y - z*x)
    pitch = math.copysign(math.pi/2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    yaw = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0*(y*y + z*z))
    return roll, pitch, yaw


def hex2rgb(h):
    h = h.lstrip("#")
    return tuple(int(h[i:i+2], 16) / 255.0 for i in (0, 2, 4))


def load_vtk_stl(path):
    reader = vtk.vtkSTLReader()
    reader.SetFileName(path)
    reader.Update()
    decimate = vtk.vtkDecimatePro()
    decimate.SetInputConnection(reader.GetOutputPort())
    decimate.SetTargetReduction(0.85)
    decimate.PreserveTopologyOn()
    decimate.Update()
    normals = vtk.vtkPolyDataNormals()
    normals.SetInputConnection(decimate.GetOutputPort())
    normals.ComputePointNormalsOn()
    normals.Update()
    return normals.GetOutput()


def init_moveit(node_name="moveit_planning_tool"):
    config = MoveItConfigsBuilder("arv_v1_model", package_name="arv_v1_moveit").to_moveit_configs()
    d = config.to_dict()
    # MoveItConfigsBuilder produces planning_pipelines as a flat list,
    # but MoveItCpp C++ side reads planning_pipelines.pipeline_names (nested dict).
    pipelines = d.get("planning_pipelines", [])
    if isinstance(pipelines, list):
        d["planning_pipelines"] = {"pipeline_names": pipelines, "namespace": ""}
    d["default_planning_request_adapters.fix_start_state"] = True
    moveit = MoveItPy(node_name=node_name, config_dict=d)
    return moveit


class StateListener(Node):
    def __init__(self):
        super().__init__("planning_tool_listener")
        self.joints = None
        self._lock = threading.Lock()
        self.create_subscription(JointState, "/joint_states", self._cb, 10)
        self.cart_pub = self.create_publisher(PoseStamped, "/cartesian_target_pose", 10)

    def _cb(self, msg):
        if len(msg.position) >= 6:
            with self._lock:
                self.joints = list(msg.position[:6])

    def get_joints(self):
        with self._lock:
            return list(self.joints) if self.joints else None


# ── GUI ────────────────────────────────────────────────

class ArmPlanningTool:
    def __init__(self):
        self._last_plan_result = None
        self._busy = False

        rclpy.init()
        self._listener = StateListener()
        threading.Thread(target=lambda: rclpy.spin(self._listener), daemon=True).start()

        self._moveit = init_moveit()
        self._arm = self._moveit.get_planning_component(PLANNING_GROUP)
        self._model = self._moveit.get_robot_model()
        self._presets = load_presets()
        self._current_joints = None
        self._goal_joints = None
        self._goal_xyz = None
        self._goal_rpy = None
        self._plan_xyz = None
        self._trail_positions = None
        self._3d_dirty = True
        self._build_gui()
        self._tick()

    def _build_gui(self):
        self._root = ctk.CTk()
        self._root.title("ARV_V1 Motion Planning")
        self._root.geometry("1120x640")
        self._root.minsize(960, 560)
        self._root.configure(fg_color=BASE)

        main = ctk.CTkFrame(self._root, fg_color="transparent")
        main.pack(fill="both", expand=True, padx=0, pady=0)
        main.columnconfigure(0, weight=0)
        main.columnconfigure(1, weight=1)
        main.rowconfigure(0, weight=1)

        left = ctk.CTkFrame(main, fg_color="transparent", width=500)
        left.grid(row=0, column=0, sticky="ns", padx=(12, 0), pady=12)

        # ── 3D Preview (right) ──
        right = ctk.CTkFrame(main, fg_color=MANTLE, corner_radius=10)
        right.grid(row=0, column=1, sticky="nsew", padx=12, pady=12)
        self._build_3d_panel(right)

        # ── Tabs ──
        self._tabview = ctk.CTkTabview(left, height=260,
                                       fg_color=MANTLE, segmented_button_selected_color=BTN)
        self._tabview.pack(fill="x", pady=(0, 0))
        self._tabview.add("Cartesian")
        self._tabview.add("Joint Space")

        self._cart_vars = {}
        for i, (label, steps) in enumerate(
            [("X", POS_STEPS), ("Y", POS_STEPS), ("Z", POS_STEPS),
             ("Yaw", ANG_STEPS), ("Pitch", ANG_STEPS), ("Roll", ANG_STEPS)]
        ):
            var, step_var = self._input_row(self._tabview.tab("Cartesian"), label, steps, i)
            self._cart_vars[label] = (var, step_var)

        self._joint_vars = []
        for i in range(6):
            var, step_var = self._input_row(self._tabview.tab("Joint Space"), f"J{i+1}", ANG_STEPS, i)
            self._joint_vars.append((var, step_var))

        # ── Planner row ──
        ctrl = ctk.CTkFrame(left, fg_color="transparent")
        ctrl.pack(fill="x", pady=8)

        ctk.CTkLabel(ctrl, text="Planner:", text_color=TEXT,
                     font=ctk.CTkFont(size=14)).pack(side="left")
        self._planner_var = tk.StringVar(value="PTP (Pilz)")
        ctk.CTkOptionMenu(ctrl, variable=self._planner_var, values=list(PLANNERS.keys()),
                          width=150, fg_color=BTN, button_color=BTN, button_hover_color=BTN_HOVER,
                          font=ctk.CTkFont(size=13)).pack(side="left", padx=(4, 14))

        ctk.CTkLabel(ctrl, text="Vel:", text_color=TEXT,
                     font=ctk.CTkFont(size=14)).pack(side="left")
        self._vel_var = tk.DoubleVar(value=0.3)
        self._vel_lbl = ctk.CTkLabel(ctrl, text="0.30", text_color=LAVENDER, width=40,
                                     font=ctk.CTkFont(family="JetBrains Mono", size=14))
        ctk.CTkSlider(ctrl, from_=0.05, to=1.0, variable=self._vel_var, width=130,
                      progress_color=MAUVE, button_color=LAVENDER,
                      command=lambda v: self._vel_lbl.configure(text=f"{v:.2f}")
                      ).pack(side="left", padx=4)
        self._vel_lbl.pack(side="left")

        # ── Action buttons ──
        btn = ctk.CTkFrame(left, fg_color="transparent")
        btn.pack(fill="x")

        for text, cmd, fg, hv, w in [
            ("Read Current", self._on_read, BTN, BTN_HOVER, 120),
            ("Plan", self._on_plan, BTN_OK, BTN_OK_HOVER, 70),
            ("Execute", self._on_execute, BTN, BTN_HOVER, 80),
            ("Plan+Exec", self._on_plan_execute, BTN, BTN_HOVER, 100),
            ("STOP", self._on_stop, BTN_DANGER, BTN_DANGER_HOVER, 60),
        ]:
            ctk.CTkButton(btn, text=text, command=cmd, width=w, height=36,
                          fg_color=fg, hover_color=hv, text_color="#ffffff",
                          font=ctk.CTkFont(size=13, weight="bold")
                          ).pack(side="left", padx=3)

        # ── Presets ──
        pf = ctk.CTkFrame(left, fg_color="transparent")
        pf.pack(fill="x", pady=8)

        ctk.CTkLabel(pf, text="Preset:", text_color=TEXT,
                     font=ctk.CTkFont(size=14)).pack(side="left")
        self._preset_var = tk.StringVar()
        self._preset_menu = ctk.CTkOptionMenu(
            pf, variable=self._preset_var, width=170,
            values=list(self._presets.keys()) or ["(none)"],
            fg_color=SURFACE0, button_color=SURFACE1, font=ctk.CTkFont(size=13))
        self._preset_menu.pack(side="left", padx=4)

        for text, cmd, fg in [("Load", self._on_load_preset, BTN),
                               ("Go", self._on_preset_go, MAUVE),
                               ("Save As", self._on_save_preset, SURFACE1)]:
            ctk.CTkButton(pf, text=text, width=65, height=32, fg_color=fg, hover_color=BTN_HOVER,
                          text_color="#ffffff", font=ctk.CTkFont(size=13, weight="bold"),
                          command=cmd).pack(side="left", padx=2)

        # ── Status ──
        sf = ctk.CTkFrame(left, corner_radius=10, fg_color=MANTLE)
        sf.pack(fill="x", pady=(0, 0))

        mono = ctk.CTkFont(family="JetBrains Mono", size=13)
        self._pose_lbl = ctk.CTkLabel(sf, text="Pose: --", font=mono, text_color=TEXT, anchor="w")
        self._pose_lbl.pack(fill="x", padx=10, pady=(8, 0))
        self._joints_lbl = ctk.CTkLabel(sf, text="Joints: --", font=mono, text_color=TEXT, anchor="w")
        self._joints_lbl.pack(fill="x", padx=10)
        self._status_lbl = ctk.CTkLabel(sf, text="Ready", text_color=GREEN, anchor="w",
                                        font=ctk.CTkFont(size=14, weight="bold"))
        self._status_lbl.pack(fill="x", padx=10, pady=(0, 8))

    def _make_repeat_btn(self, parent, text, action, col):
        btn = ctk.CTkButton(parent, text=text, width=32, height=32,
                            fg_color=BTN, hover_color=BTN_HOVER,
                            text_color="#ffffff", font=ctk.CTkFont(size=16),
                            command=action)
        btn.grid(row=0, column=0, padx=1)  # placeholder, repositioned below
        btn._repeat_id = None

        def on_press(e):
            action()
            def repeat():
                action()
                btn._repeat_id = btn.after(60, repeat)
            btn._repeat_id = btn.after(350, repeat)

        def on_release(e):
            if btn._repeat_id:
                btn.after_cancel(btn._repeat_id)
                btn._repeat_id = None

        btn.bind("<ButtonPress-1>", on_press)
        btn.bind("<ButtonRelease-1>", on_release)
        btn.grid(row=0, column=col, padx=1)
        return btn

    def _input_row(self, parent, label, steps, row):
        ctk.CTkLabel(parent, text=label, width=50, anchor="e", text_color=TEXT,
                     font=ctk.CTkFont(size=14)).grid(row=row, column=0, padx=(0, 4), pady=3)
        var = tk.DoubleVar(value=0.0)
        ctk.CTkEntry(parent, textvariable=var, width=110, justify="right",
                     font=ctk.CTkFont(family="JetBrains Mono", size=14),
                     fg_color=SURFACE0, text_color=TEXT, border_color=SURFACE1
                     ).grid(row=row, column=1, padx=2, pady=3)

        step_var = tk.DoubleVar(value=steps[0])

        def nudge(sign):
            var.set(round(var.get() + sign * step_var.get(), 6))
            self._update_goal_preview()

        bf = ctk.CTkFrame(parent, fg_color="transparent")
        bf.grid(row=row, column=2, columnspan=2, padx=1)
        self._make_repeat_btn(bf, "−", lambda: nudge(-1), 0)
        self._make_repeat_btn(bf, "+", lambda: nudge(+1), 1)

        step_labels = [str(s) for s in steps]
        seg = ctk.CTkSegmentedButton(parent, values=step_labels, width=180,
                                     font=ctk.CTkFont(size=12),
                                     selected_color=MAUVE, selected_hover_color=LAVENDER,
                                     unselected_color=SURFACE0, unselected_hover_color=SURFACE1,
                                     command=lambda v: step_var.set(float(v)))
        seg.set(step_labels[0])
        seg.grid(row=row, column=4, padx=(8, 0), pady=3)

        return var, step_var

    # ── 3D Preview (VTK) ──

    def _build_3d_panel(self, parent):
        frame = tk.Frame(parent, bg=CRUST)
        frame.pack(fill="both", expand=True, padx=2, pady=2)

        self._vtk_renwin = vtk.vtkRenderWindow()
        self._vtk_renwin.SetNumberOfLayers(2)
        self._vtk_iren = vtkTkRenderWindowInteractor(frame, rw=self._vtk_renwin,
                                                      width=500, height=500)

        self._vtk_renderer = vtk.vtkRenderer()
        self._vtk_renderer.SetBackground(*hex2rgb(CRUST))
        self._vtk_renderer.SetLayer(0)
        self._vtk_renwin.AddRenderer(self._vtk_renderer)

        self._vtk_iren.pack(fill="both", expand=True)
        self._vtk_iren.Initialize()

        style = vtk.vtkInteractorStyleTrackballCamera()
        self._vtk_iren.SetInteractorStyle(style)

        self._vtk_link_actors = {}
        self._vtk_ghost_actors = {}
        self._vtk_trail_actor = None
        self._vtk_goal_sphere = None

        self._load_vtk_meshes()
        self._add_grid()
        self._setup_camera()

    def _load_vtk_meshes(self):
        for i, name in enumerate(MESH_LINKS):
            path = os.path.join(MESHES_DIR, f"{name}.STL")
            if not os.path.exists(path):
                continue
            polydata = load_vtk_stl(path)
            if name in VISUAL_ORIGINS:
                t = vtk.vtkTransform()
                t.Translate(*VISUAL_ORIGINS[name])
                tf = vtk.vtkTransformPolyDataFilter()
                tf.SetInputData(polydata)
                tf.SetTransform(t)
                tf.Update()
                polydata = tf.GetOutput()

            for ghost in (False, True):
                mapper = vtk.vtkPolyDataMapper()
                mapper.SetInputData(polydata)
                actor = vtk.vtkActor()
                actor.SetMapper(mapper)
                if ghost:
                    actor.GetProperty().SetColor(*GHOST_COLOR_RGB)
                    actor.GetProperty().SetOpacity(0.3)
                    actor.SetVisibility(False)
                    self._vtk_ghost_actors[name] = actor
                else:
                    actor.GetProperty().SetColor(*MESH_COLORS_RGB[i])
                    actor.GetProperty().SetOpacity(1.0)
                    self._vtk_link_actors[name] = actor
                actor.GetProperty().SetSpecular(0.3)
                actor.GetProperty().SetSpecularPower(20)
                self._vtk_renderer.AddActor(actor)

    def _add_grid(self):
        grid_size, step = 1.0, 0.1
        pts = vtk.vtkPoints()
        lines = vtk.vtkCellArray()
        n = int(grid_size / step)
        idx = 0
        for i in range(-n, n + 1):
            x = i * step
            pts.InsertNextPoint(x, -grid_size, 0)
            pts.InsertNextPoint(x, grid_size, 0)
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, idx)
            line.GetPointIds().SetId(1, idx + 1)
            lines.InsertNextCell(line)
            idx += 2
            pts.InsertNextPoint(-grid_size, x, 0)
            pts.InsertNextPoint(grid_size, x, 0)
            line = vtk.vtkLine()
            line.GetPointIds().SetId(0, idx)
            line.GetPointIds().SetId(1, idx + 1)
            lines.InsertNextCell(line)
            idx += 2
        pd = vtk.vtkPolyData()
        pd.SetPoints(pts)
        pd.SetLines(lines)
        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputData(pd)
        actor = vtk.vtkActor()
        actor.SetMapper(mapper)
        actor.GetProperty().SetColor(*hex2rgb(SURFACE1))
        actor.GetProperty().SetOpacity(0.4)
        self._vtk_renderer.AddActor(actor)

    def _setup_camera(self):
        cam = self._vtk_renderer.GetActiveCamera()
        cam.SetPosition(1.0, -0.8, 0.7)
        cam.SetFocalPoint(0.0, 0.0, 0.35)
        cam.SetViewUp(0, 0, 1)
        self._vtk_renderer.ResetCameraClippingRange()

    def _fk_transforms(self, joint_vals):
        state = RobotState(self._model)
        state.set_joint_group_positions(PLANNING_GROUP, np.array(joint_vals))
        state.update()
        transforms = {}
        for link in FK_CHAIN:
            transforms[link] = state.get_global_link_transform(link)
        return transforms

    def _apply_transform(self, actor, T):
        m = vtk.vtkMatrix4x4()
        for r in range(4):
            for c in range(4):
                m.SetElement(r, c, T[r, c])
        actor.SetUserMatrix(m)

    def _update_3d(self, current_joints=None, goal_joints=None, goal_xyz=None, plan_xyz=None):
        if current_joints is not None:
            transforms = self._fk_transforms(current_joints)
            for name, actor in self._vtk_link_actors.items():
                if name in transforms:
                    self._apply_transform(actor, transforms[name])
                    actor.SetVisibility(True)

        if goal_joints is not None:
            transforms = self._fk_transforms(goal_joints)
            for name, actor in self._vtk_ghost_actors.items():
                if name in transforms:
                    self._apply_transform(actor, transforms[name])
                    actor.SetVisibility(True)
        else:
            for actor in self._vtk_ghost_actors.values():
                actor.SetVisibility(False)

        # Remove old goal actors
        if self._vtk_goal_sphere is not None:
            self._vtk_renderer.RemoveActor(self._vtk_goal_sphere)
            self._vtk_goal_sphere = None
        for a in getattr(self, '_vtk_goal_arrows', []):
            self._vtk_renderer.RemoveActor(a)
        self._vtk_goal_arrows = []

        if goal_xyz is not None:
            # Sphere — bigger, semi-transparent
            src = vtk.vtkSphereSource()
            src.SetCenter(*goal_xyz)
            src.SetRadius(0.035)
            src.SetPhiResolution(24)
            src.SetThetaResolution(24)
            src.Update()
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(src.GetOutputPort())
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(*hex2rgb(RED))
            actor.GetProperty().SetOpacity(0.5)
            self._vtk_renderer.AddActor(actor)
            self._vtk_goal_sphere = actor

            # Orientation arrows (RGB = XYZ)
            rpy = self._goal_rpy
            if rpy is not None:
                R = self._rpy_to_matrix(*rpy)
                arrow_len = 0.06
                colors = [(1, 0, 0), (0, 1, 0), (0, 0, 1)]  # X=red, Y=green, Z=blue
                for axis_idx, color in enumerate(colors):
                    direction = R[:, axis_idx]
                    arrow_src = vtk.vtkArrowSource()
                    arrow_src.SetTipResolution(12)
                    arrow_src.SetShaftResolution(12)
                    arrow_src.Update()
                    T = vtk.vtkTransform()
                    T.Translate(*goal_xyz)
                    # vtk arrow points along +X, rotate to desired direction
                    fwd = direction / (np.linalg.norm(direction) + 1e-9)
                    # Rotation from [1,0,0] to fwd
                    cross = np.cross([1, 0, 0], fwd)
                    dot = np.dot([1, 0, 0], fwd)
                    if np.linalg.norm(cross) > 1e-6:
                        angle = math.degrees(math.acos(np.clip(dot, -1, 1)))
                        T.RotateWXYZ(angle, *cross)
                    elif dot < 0:
                        T.RotateWXYZ(180, 0, 1, 0)
                    T.Scale(arrow_len, arrow_len * 0.3, arrow_len * 0.3)
                    tf = vtk.vtkTransformPolyDataFilter()
                    tf.SetTransform(T)
                    tf.SetInputConnection(arrow_src.GetOutputPort())
                    tf.Update()
                    m = vtk.vtkPolyDataMapper()
                    m.SetInputConnection(tf.GetOutputPort())
                    a = vtk.vtkActor()
                    a.SetMapper(m)
                    a.GetProperty().SetColor(*color)
                    self._vtk_renderer.AddActor(a)
                    self._vtk_goal_arrows.append(a)

        # Plan result sphere (green, shows where planner actually goes)
        if getattr(self, '_vtk_plan_sphere', None) is not None:
            self._vtk_renderer.RemoveActor(self._vtk_plan_sphere)
            self._vtk_plan_sphere = None
        if plan_xyz is not None:
            src = vtk.vtkSphereSource()
            src.SetCenter(*plan_xyz)
            src.SetRadius(0.025)
            src.SetPhiResolution(16)
            src.SetThetaResolution(16)
            src.Update()
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputConnection(src.GetOutputPort())
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(*hex2rgb(GREEN))
            actor.GetProperty().SetOpacity(0.8)
            self._vtk_renderer.AddActor(actor)
            self._vtk_plan_sphere = actor

        if self._vtk_trail_actor is not None:
            self._vtk_renderer.RemoveActor(self._vtk_trail_actor)
            self._vtk_trail_actor = None
        if self._trail_positions is not None and len(self._trail_positions) > 1:
            pts_vtk = vtk.vtkPoints()
            for p in self._trail_positions:
                pts_vtk.InsertNextPoint(*p)
            lines = vtk.vtkCellArray()
            line = vtk.vtkPolyLine()
            line.GetPointIds().SetNumberOfIds(len(self._trail_positions))
            for i in range(len(self._trail_positions)):
                line.GetPointIds().SetId(i, i)
            lines.InsertNextCell(line)
            pd = vtk.vtkPolyData()
            pd.SetPoints(pts_vtk)
            pd.SetLines(lines)
            mapper = vtk.vtkPolyDataMapper()
            mapper.SetInputData(pd)
            actor = vtk.vtkActor()
            actor.SetMapper(mapper)
            actor.GetProperty().SetColor(*hex2rgb(MAUVE))
            actor.GetProperty().SetLineWidth(3)
            self._vtk_renderer.AddActor(actor)
            self._vtk_trail_actor = actor

        self._vtk_renwin.Render()

    @staticmethod
    def _rpy_to_matrix(r, p, y):
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        return np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]])

    def _update_goal_preview(self):
        if self._get_tab() == "joint":
            jvals = [self._joint_vars[i][0].get() for i in range(6)]
            self._goal_joints = jvals
            # FK to get tcp position for the goal sphere
            try:
                transforms = self._fk_transforms(jvals)
                tcp_T = transforms.get("tcp")
                if tcp_T is not None:
                    self._goal_xyz = [tcp_T[0, 3], tcp_T[1, 3], tcp_T[2, 3]]
                    R = tcp_T[:3, :3]
                    self._goal_rpy = [
                        math.atan2(R[2, 1], R[2, 2]),
                        math.atan2(-R[2, 0], math.sqrt(R[2, 1]**2 + R[2, 2]**2)),
                        math.atan2(R[1, 0], R[0, 0])]
                else:
                    self._goal_xyz = None
                    self._goal_rpy = None
            except Exception:
                self._goal_xyz = None
                self._goal_rpy = None
        else:
            self._goal_joints = None
            self._goal_xyz = [self._cart_vars[k][0].get() for k in ["X", "Y", "Z"]]
            self._goal_rpy = [self._cart_vars[k][0].get() for k in ["Roll", "Pitch", "Yaw"]]
        self._plan_xyz = None
        self._trail_positions = None
        self._3d_dirty = True

    # ── Display ──

    def _tick(self):
        try:
            psm = self._moveit.get_planning_scene_monitor()
            with psm.read_only() as scene:
                state = scene.current_state
                joints = list(state.get_joint_group_positions(PLANNING_GROUP))
                pose = state.get_pose(EE_LINK)
        except Exception:
            joints = self._listener.get_joints()
            pose = None

        if joints is not None:
            prev = self._current_joints
            self._current_joints = joints[:6]
            if prev is None or any(abs(a - b) > 0.005 for a, b in zip(prev, joints[:6])):
                self._3d_dirty = True
            jstr = " ".join(f"{v:+.3f}" for v in joints[:6])
            self._joints_lbl.configure(text=f"Joints: {jstr}")
        if pose is not None:
            p = pose.position
            r, pi, ya = rpy_from_quat(pose.orientation.x, pose.orientation.y,
                                       pose.orientation.z, pose.orientation.w)
            self._pose_lbl.configure(
                text=f"Pose: ({p.x:+.3f} {p.y:+.3f} {p.z:+.3f})  ypr({ya:+.2f} {pi:+.2f} {r:+.2f})")

        if self._3d_dirty:
            self._3d_dirty = False
            self._update_3d(self._current_joints, self._goal_joints, self._goal_xyz, self._plan_xyz)

        self._root.after(200, self._tick)

    def _set_status(self, text, color=TEXT):
        self._root.after(0, lambda: self._status_lbl.configure(text=text, text_color=color))

    # ── Planning ──

    def _get_tab(self):
        return "cartesian" if self._tabview.get() == "Cartesian" else "joint"

    def _set_goal(self):
        self._arm.set_start_state_to_current_state()
        if self._get_tab() == "joint":
            jvals = [self._joint_vars[i][0].get() for i in range(6)]
            state = RobotState(self._model)
            state.set_joint_group_positions(PLANNING_GROUP, np.array(jvals))
            state.update()
            self._arm.set_goal_state(robot_state=state)
            self._goal_joints = jvals
            self._goal_xyz = None
        else:
            gx = self._cart_vars["X"][0].get()
            gy = self._cart_vars["Y"][0].get()
            gz = self._cart_vars["Z"][0].get()
            pose = PoseStamped()
            pose.header.frame_id = REF_FRAME
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = gx, gy, gz
            q = quat_from_rpy(self._cart_vars["Roll"][0].get(),
                              self._cart_vars["Pitch"][0].get(),
                              self._cart_vars["Yaw"][0].get())
            pose.pose.orientation.x, pose.pose.orientation.y = q[0], q[1]
            pose.pose.orientation.z, pose.pose.orientation.w = q[2], q[3]
            self._arm.set_goal_state(pose_stamped_msg=pose, pose_link=EE_LINK)
            self._goal_joints = None
            self._goal_xyz = [gx, gy, gz]
        self._3d_dirty = True

    def _plan_params(self):
        label = self._planner_var.get()
        pipeline, pid = PLANNERS.get(label, ("ompl", "RRTConnect"))
        vel = self._vel_var.get()

        if pid == "LIN" and self._get_tab() == "joint":
            state = RobotState(self._model)
            state.set_joint_group_positions(PLANNING_GROUP,
                np.array([self._joint_vars[i][0].get() for i in range(6)]))
            state.update()
            fk = state.get_pose(EE_LINK)
            if fk is None:
                self._set_status("FK failed for LIN", RED)
                return None
            pose = PoseStamped()
            pose.header.frame_id = REF_FRAME
            pose.pose = fk
            self._arm.set_start_state_to_current_state()
            self._arm.set_goal_state(pose_stamped_msg=pose, pose_link=EE_LINK)

        p = PlanRequestParameters(self._moveit, pipeline)
        p.planning_pipeline = pipeline
        p.planner_id = pid
        p.max_velocity_scaling_factor = vel
        p.max_acceleration_scaling_factor = vel
        p.planning_time = 5.0
        p.planning_attempts = 1 if pipeline.startswith("pilz") else 5
        return p

    def _extract_trail(self, traj):
        n = len(traj)
        step = max(1, n // 30)
        pts = []
        for i in range(0, n, step):
            wp = traj[i]
            jvals = list(wp.get_joint_group_positions(PLANNING_GROUP))
            transforms = self._fk_transforms(jvals)
            pts.append(transforms["tcp"][:3, 3])
        return np.array(pts) if pts else None

    def _do_direct_ik(self):
        if self._get_tab() == "joint":
            self._set_status("Direct LIN: only works in Cartesian tab", RED)
            return False
        from arv_v1_interfaces.srv import MoveToCartesianRPY
        if not hasattr(self, '_cart_srv'):
            self._cart_srv = self._listener.create_client(MoveToCartesianRPY, "/move_to_cartesian_rpy")
        if not self._cart_srv.wait_for_service(timeout_sec=2.0):
            self._set_status("Service /move_to_cartesian_rpy not available", RED)
            return False
        req = MoveToCartesianRPY.Request()
        req.x = self._cart_vars["X"][0].get()
        req.y = self._cart_vars["Y"][0].get()
        req.z = self._cart_vars["Z"][0].get()
        req.roll = self._cart_vars["Roll"][0].get()
        req.pitch = self._cart_vars["Pitch"][0].get()
        req.yaw = self._cart_vars["Yaw"][0].get()
        req.velocity_scaling = self._vel_var.get()
        req.acceleration_scaling = self._vel_var.get()
        req.async_execution = False
        self._set_status("Direct LIN executing...", BLUE)
        future = self._cart_srv.call_async(req)
        # Node is already spinning in background thread; just wait on future
        import time
        deadline = time.monotonic() + 30.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done() or future.result() is None:
            self._set_status("Service call timeout", RED)
            return False
        res = future.result()
        if res.success:
            self._goal_xyz = [req.x, req.y, req.z]
            self._3d_dirty = True
            self._set_status(f"LIN OK — {res.trajectory_duration:.2f}s", GREEN)
            return True
        self._set_status(f"LIN FAIL: {res.message}", RED)
        return False

    def _do_plan(self):
        label = self._planner_var.get()
        if label == "Direct LIN":
            return self._do_direct_ik()
        self._set_status("Planning...", BLUE)
        try:
            self._set_goal()
            params = self._plan_params()
            if not params:
                return False
            result = self._arm.plan(single_plan_parameters=params)
            if result.error_code.val == 1:
                self._last_plan_result = result
                traj = result.trajectory
                n = len(traj)
                d = traj.duration
                self._trail_positions = self._extract_trail(traj)
                # Extract planned endpoint for ghost + green sphere (don't overwrite user's red target)
                last_wp = traj[n - 1] if n > 0 else None
                if last_wp is not None:
                    end_joints = list(last_wp.get_joint_group_positions(PLANNING_GROUP))
                    self._goal_joints = end_joints
                    end_T = self._fk_transforms(end_joints).get("tcp")
                    if end_T is not None:
                        self._plan_xyz = [end_T[0, 3], end_T[1, 3], end_T[2, 3]]
                    else:
                        self._plan_xyz = None
                else:
                    self._plan_xyz = None
                self._3d_dirty = True
                self._set_status(f"Plan OK — {n} pts, {d:.2f}s", GREEN)
                return True
            self._last_plan_result = None
            self._trail_positions = None
            code = result.error_code.val
            ERR_NAMES = {-1: "PLANNING_FAILED", -10: "START_IN_COLLISION",
                         -12: "GOAL_IN_COLLISION", -31: "NO_IK_SOLUTION",
                         -6: "TIMED_OUT", -27: "GOAL_STATE_INVALID"}
            name = ERR_NAMES.get(code, f"code={code}")
            self._set_status(f"Plan FAILED: {name}", RED)
            return False
        except Exception as e:
            self._set_status(f"Plan error: {e}", RED)
            return False

    def _do_execute(self):
        if not self._last_plan_result:
            self._set_status("No trajectory — Plan first", RED)
            return
        self._set_status("Executing...", BLUE)
        try:
            self._moveit.execute(self._last_plan_result.trajectory, controllers=[])
            self._goal_joints = None
            self._goal_xyz = None
            self._trail_positions = None
            self._3d_dirty = True
            self._set_status("Execution complete", GREEN)
        except Exception as e:
            self._set_status(f"Exec error: {e}", RED)

    def _run(self, fn):
        if self._busy:
            self._set_status("Busy...", YELLOW)
            return
        self._busy = True
        def w():
            try: fn()
            finally: self._busy = False
        threading.Thread(target=w, daemon=True).start()

    def _on_plan(self): self._run(self._do_plan)
    def _on_execute(self): self._run(self._do_execute)
    def _on_plan_execute(self):
        self._run(lambda: self._do_plan() and self._do_execute())
    def _on_stop(self):
        try:
            self._moveit.get_trajectory_execution_manager().stop_execution()
            self._set_status("STOPPED", RED)
        except Exception as e:
            self._set_status(f"Stop: {e}", RED)

    # ── Read Current ──

    def _on_read(self):
        try:
            psm = self._moveit.get_planning_scene_monitor()
            with psm.read_only() as scene:
                state = scene.current_state
                joints = list(state.get_joint_group_positions(PLANNING_GROUP))
                # get_global_link_transform returns 4x4 in model(world) frame
                T_tcp = state.get_global_link_transform(EE_LINK)
                T_base = state.get_global_link_transform("base_link")
                # tcp in base_link frame = T_base^{-1} * T_tcp
                T_rel = np.linalg.inv(T_base) @ T_tcp
                pose_xyz = T_rel[:3, 3]
                pose_rot = T_rel[:3, :3]
        except Exception:
            joints = self._listener.get_joints()
            pose_xyz = None
            pose_rot = None

        if joints:
            for i in range(6):
                self._joint_vars[i][0].set(round(joints[i], 4))
        if pose_xyz is not None:
            self._cart_vars["X"][0].set(round(float(pose_xyz[0]), 4))
            self._cart_vars["Y"][0].set(round(float(pose_xyz[1]), 4))
            self._cart_vars["Z"][0].set(round(float(pose_xyz[2]), 4))
            # RPY from rotation matrix (ZYX convention) — pick solution closest to current UI values
            r1 = math.atan2(pose_rot[2, 1], pose_rot[2, 2])
            p1 = math.atan2(-pose_rot[2, 0], math.sqrt(pose_rot[2, 1]**2 + pose_rot[2, 2]**2))
            y1 = math.atan2(pose_rot[1, 0], pose_rot[0, 0])
            # Equivalent second solution
            r2 = r1 + (math.pi if r1 < 0 else -math.pi)
            p2 = math.pi - p1 if p1 >= 0 else -math.pi - p1
            y2 = y1 + (math.pi if y1 < 0 else -math.pi)
            # Pick closer to current input
            r_cur = self._cart_vars["Roll"][0].get()
            p_cur = self._cart_vars["Pitch"][0].get()
            y_cur = self._cart_vars["Yaw"][0].get()
            cost1 = (r1 - r_cur)**2 + (p1 - p_cur)**2 + (y1 - y_cur)**2
            cost2 = (r2 - r_cur)**2 + (p2 - p_cur)**2 + (y2 - y_cur)**2
            r, p, y = (r1, p1, y1) if cost1 <= cost2 else (r2, p2, y2)
            self._cart_vars["Roll"][0].set(round(r, 4))
            self._cart_vars["Pitch"][0].set(round(p, 4))
            self._cart_vars["Yaw"][0].set(round(y, 4))
            self._set_status("Loaded current pose (base_link frame)", GREEN)
        elif joints:
            self._set_status("Loaded joints (no FK)", YELLOW)
        else:
            self._set_status("No data", RED)

    # ── Presets ──

    def _on_load_preset(self):
        name = self._preset_var.get()
        if not name or name not in self._presets:
            return
        p = self._presets[name]
        if p.get("type") == "joint" and "joint_values" in p:
            for i, v in enumerate(p["joint_values"][:6]):
                self._joint_vars[i][0].set(round(v, 4))
            self._tabview.set("Joint Space")
        elif p.get("type") == "cartesian":
            for k, v in zip(["X", "Y", "Z"], p.get("xyz", [0, 0, 0])):
                self._cart_vars[k][0].set(round(v, 4))
            for k, v in zip(["Roll", "Pitch", "Yaw"], p.get("rpy", [0, 0, 0])):
                self._cart_vars[k][0].set(round(v, 4))
            self._tabview.set("Cartesian")
        self._set_status(f"Loaded: {name}", GREEN)

    def _on_preset_go(self):
        self._on_load_preset()
        name = self._preset_var.get()
        if name and name in self._presets:
            self._set_status(f"Go → {name}", BLUE)
            self._on_plan_execute()

    def _on_save_preset(self):
        dialog = ctk.CTkInputDialog(text="Preset name:", title="Save Preset")
        name = dialog.get_input()
        if not name:
            return
        if self._get_tab() == "joint":
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
        save_presets(self._presets)
        self._preset_menu.configure(values=list(self._presets.keys()))
        self._preset_var.set(name)
        self._set_status(f"Saved: {name}", GREEN)

    def run(self):
        self._root.mainloop()

    def shutdown(self):
        try: self._moveit.shutdown()
        except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass


# ── Presets I/O ──

def load_presets():
    defaults = {
        "escape": {"type": "joint", "joint_values": [0.0, 2.1746, -0.89, -1.326, 1.5028, -1.6796]},
        "start": {"type": "joint", "joint_values": [0.0, 2.6343, -0.89, 0.0, 0.0, 0.0]},
    }
    if os.path.exists(PRESETS_PATH):
        try:
            with open(PRESETS_PATH) as f:
                data = yaml.safe_load(f) or {}
            defaults.update(data.get("presets", {}))
        except Exception:
            pass
    return defaults


def save_presets(presets):
    os.makedirs(os.path.dirname(PRESETS_PATH), exist_ok=True)
    with open(PRESETS_PATH, "w") as f:
        yaml.dump({"presets": presets}, f, default_flow_style=False)


# ── CLI Mode ──

def cli_plan_execute(joints=None, pose_xyzrpy=None, planner="PTP (Pilz)", vel=0.3, preset=None):
    rclpy.init()
    moveit = init_moveit("moveit_planning_tool_cli")
    arm = moveit.get_planning_component(PLANNING_GROUP)
    model = moveit.get_robot_model()
    time.sleep(1.0)

    if preset:
        presets = load_presets()
        p = presets.get(preset)
        if not p:
            print(f"Unknown preset: {preset}. Available: {list(presets.keys())}")
            moveit.shutdown(); rclpy.shutdown(); return
        joints = p["joint_values"] if p["type"] == "joint" else None
        pose_xyzrpy = (p["xyz"] + p["rpy"]) if p["type"] == "cartesian" else None

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

    pipeline, pid = PLANNERS.get(planner, ("ompl", "RRTConnect"))
    p = PlanRequestParameters(moveit, pipeline)
    p.planner_id = pid
    p.max_velocity_scaling_factor = vel
    p.max_acceleration_scaling_factor = vel
    p.planning_time = 5.0
    p.planning_attempts = 1 if pipeline.startswith("pilz") else 5

    print(f"Planning with {planner}...")
    result = arm.plan(plan_parameters=p)
    if not result:
        print("Planning FAILED")
    else:
        print("Plan OK. Executing...")
        moveit.execute(result.trajectory, controllers=[])
        print("Done.")

    moveit.shutdown()
    rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="ARV_V1 Motion Planning Tool")
    parser.add_argument("--joints", nargs=6, type=float, metavar="J")
    parser.add_argument("--pose", nargs=6, type=float, metavar="V",
                        help="x y z roll pitch yaw")
    parser.add_argument("--preset", type=str)
    parser.add_argument("--planner", type=str, default="PTP (Pilz)")
    parser.add_argument("--vel", type=float, default=0.3)
    args = parser.parse_args()

    if args.joints or args.pose or args.preset:
        cli_plan_execute(joints=args.joints, pose_xyzrpy=args.pose,
                         planner=args.planner, vel=args.vel, preset=args.preset)
    else:
        tool = ArmPlanningTool()
        try:
            tool.run()
        finally:
            tool.shutdown()


if __name__ == "__main__":
    main()
