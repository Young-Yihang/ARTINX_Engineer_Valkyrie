#!/usr/bin/env python3
"""
Hardware deployment launch — headless auto-start for NUC.

Launches core nodes (no RViz, no MuJoCo, no ncurses TUI).
mission_executor runs in headless mode to process MCU /task_command.
rosbag2 records key topics with zstd compression.
"""

import datetime
import glob
import os
import shutil

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    serial_arg = LaunchConfiguration("serial_port").perform(context)
    log_base = os.path.expanduser(LaunchConfiguration("log_dir").perform(context))
    enable_rosbag = LaunchConfiguration("enable_rosbag").perform(context)

    # --- Serial port auto-detection ---
    if serial_arg == "auto":
        candidates = sorted(glob.glob("/dev/ttyACM*")) + sorted(
            glob.glob("/dev/ttyUSB*")
        )
        serial_port = candidates[0] if candidates else "/dev/ttyACM0"
    else:
        serial_port = serial_arg

    # --- Session directory ---
    session_id = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    session_dir = os.path.join(log_base, session_id)
    os.makedirs(os.path.join(session_dir, "rosbag"), exist_ok=True)

    latest = os.path.join(log_base, "latest")
    if os.path.islink(latest):
        os.unlink(latest)
    os.symlink(session_dir, latest)

    # --- Parameter snapshot ---
    pkg_share = get_package_share_directory("arv_v1_moveit")
    config_dir = os.path.join(pkg_share, "config")
    for yaml_name in ["controller_params_hw.yaml", "cartesian_controller_param.yaml"]:
        src = os.path.join(config_dir, yaml_name)
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(session_dir, yaml_name))

    # --- MoveIt config ---
    moveit_config = MoveItConfigsBuilder(
        "arv_v1_model", package_name="arv_v1_moveit"
    ).to_moveit_configs()

    ws_src = os.path.expanduser("~/ros2_ws/src/arv_v1_moveit/config")
    controller_params = os.path.join(ws_src, "controller_params_hw.yaml")
    cartesian_params = os.path.join(ws_src, "cartesian_controller_param.yaml")
    trajectory_dir = os.path.join(ws_src, "trajectories")

    # --- Nodes ---

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        respawn=True,
        respawn_delay=2.0,
        parameters=[moveit_config.robot_description],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="both",
        respawn=True,
        respawn_delay=3.0,
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0},
            {"default_robot_padding": -0.005},
            {"jiggle_fraction": 0.05},
            {"default_planning_request_adapters.fix_start_state": True},
            {"publish_planning_scene": True},
            {"publish_geometry_updates": True},
            {"publish_state_updates": True},
            {"publish_transforms_updates": True},
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
        ],
    )

    torque_controller_node = Node(
        package="arv_v1_moveit",
        executable="torque_controller_node",
        name="torque_controller_action_server",
        output="both",
        respawn=True,
        respawn_delay=2.0,
        parameters=[controller_params],
    )

    mission_executor_node = Node(
        package="arv_v1_moveit",
        executable="mission_executor_node",
        name="mission_executor",
        output="both",
        respawn=True,
        respawn_delay=2.0,
    )

    hardware_interface_node = Node(
        package="arv_v1_moveit",
        executable="hardware_interface_node",
        name="hardware_interface",
        output="both",
        respawn=True,
        respawn_delay=3.0,
        parameters=[{"serial_port": serial_port}],
    )

    trajectory_manager_node = Node(
        package="arv_v1_moveit",
        executable="trajectory_manager_node",
        name="trajectory_manager",
        output="both",
        respawn=True,
        respawn_delay=2.0,
        parameters=[{"trajectory_dir": trajectory_dir}],
    )

    cartesian_controller_node = Node(
        package="arv_v1_moveit",
        executable="cartesian_controller_node",
        name="cartesian_controller",
        output="both",
        respawn=True,
        respawn_delay=2.0,
        parameters=[cartesian_params],
    )

    # --- rosbag2 recording ---
    rosbag_cmd = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "--storage",
            "mcap",
            "--max-bag-duration",
            "3600",
            "--compression-mode",
            "file",
            "--compression-format",
            "zstd",
            "--output",
            os.path.join(session_dir, "rosbag", "session"),
            "--topics",
            "/joint_states",
            "/effort_controller/commands",
            "/ARM_controller/follow_joint_trajectory/_action/status",
            "/cartesian_controller/current_pose",
            "/cartesian_target_pose",
            "/control_mode",
            "/arm_state",
            "/task_command",
            "/ARM_controller/joint_trajectory",
            "/hardware_link_diag",
            "/rosout",
            "/tf",
            "/tf_static",
            "/robot_description",
        ],
        condition=IfCondition(enable_rosbag),
        output="log",
    )

    # --- Launch order ---
    return [
        robot_state_publisher,
        move_group_node,
        TimerAction(period=2.0, actions=[torque_controller_node]),
        TimerAction(period=4.0, actions=[mission_executor_node]),
        TimerAction(period=5.0, actions=[hardware_interface_node]),
        TimerAction(period=5.0, actions=[trajectory_manager_node]),
        TimerAction(period=5.0, actions=[cartesian_controller_node]),
        TimerAction(period=7.0, actions=[rosbag_cmd]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "serial_port",
                default_value="auto",
                description="Serial port for hardware interface (auto = detect /dev/ttyACM*)",
            ),
            DeclareLaunchArgument(
                "log_dir",
                default_value="~/arv_v1_logs",
                description="Base directory for session logs and rosbag",
            ),
            DeclareLaunchArgument(
                "enable_rosbag",
                default_value="true",
                description="Enable rosbag2 recording",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
