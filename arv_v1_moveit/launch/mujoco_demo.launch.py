#!/usr/bin/env python3
"""
MuJoCo 仿真的自定义 Launch 文件
不启动 joint_state_broadcaster，避免与 mujoco_interface_node 冲突
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    # 构建 MoveIt 配置
    moveit_config = MoveItConfigsBuilder("arv_v1_model", package_name="arv_v1_moveit").to_moveit_configs()

    # 1. robot_state_publisher - 发布 TF 变换
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
        ],
    )

    # 2. move_group - MoveIt 规划器
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
        parameters=[
            moveit_config.to_dict(),
            {"trajectory_execution.allowed_execution_duration_scaling": 2.0},
            {"default_robot_padding": -0.005},  # 碰撞体内缩 5mm, 容忍微小位置偏差
            {"jiggle_fraction": 0.05},  # 起点 in collision 时微调 5% 逃出
            {"default_planning_request_adapters.fix_start_state": True},  # 限位边缘微量越界自动clamp回限内
            {"publish_planning_scene": True},
            {"publish_geometry_updates": True},
            {"publish_state_updates": True},
            {"publish_transforms_updates": True},
            {"publish_robot_description": True},
            {"publish_robot_description_semantic": True},
            # 限制 planning scene 发布频率，防止 1kHz 真机 joint state
            # 导致 FCL DynamicAABBTree::update() 并发竞态崩溃
            {"planning_scene_monitor_options.publish_planning_scene_hz": 25.0},
        ],
    )

    # 3. RViz - 可视化
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("arv_v1_moveit"), "config", "moveit.rviz"]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # 注意：不启动 controller_manager 和 joint_state_broadcaster
    # 因为 mujoco_interface_node 直接发布 /joint_states

    nodes_to_start = [
        robot_state_publisher,
        move_group_node,
        rviz_node,
    ]

    return nodes_to_start


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
